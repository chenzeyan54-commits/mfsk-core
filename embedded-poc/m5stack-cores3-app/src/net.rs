// SPDX-License-Identifier: GPL-3.0-or-later
//! Bringing the network up, once, for every receiver in this binary.
//!
//! ## Why this exists
//!
//! `wspr_app` and `fst4_app` each carried their own copy of the same
//! sequence — associate, install the UDP log sink, sync NTP, start the
//! HTTP config page, then park forever holding the handles whose
//! `Drop`s would tear all of it down. The copies had drifted in three
//! ways that turned out to be real, and one that was not:
//!
//! - **Retry policy.** `wspr_app` retries association forever;
//!   `fst4_app` tries four times and then leaves the radio alone for
//!   three minutes. That is not a style difference: while the driver
//!   is trying to reach an AP it cannot find, the decoder loses ~40 %
//!   of its throughput (measured 2026-08-22, `fst4_sync_search`
//!   711 → 1 395 ms per candidate), because the WiFi task runs at
//!   FreeRTOS priority 23, above anything an application creates.
//! - **Modem power save.** `fst4_app` sets `WIFI_PS_MIN_MODEM` because
//!   an *associated but idle* STA cost its candidate loop 33 → 53 s;
//!   `wspr_app` never set it. Whether WSPR pays the same price has not
//!   been measured, so this parameter keeps each app's existing
//!   behaviour rather than quietly changing a shipped receiver.
//! - **Where the NTP result goes.** Each app's own UI status line.
//! - The fourth difference was the log prefix.
//!
//! So the policy is a parameter and the sequence is not. FT4 is why
//! this got written rather than copied a third time: its decode budget
//! is 1 750 ms from the capture window closing to key-up
//! (`ft4_rx::TX_TURNAROUND_BUDGET_MS`), so an association campaign
//! preempting the decoder is not a slow log — it is a missed QSO.
//! Every mode here gets [`Policy::Campaign`] and power save.
//!
//! ## What is *not* shared
//!
//! Driver construction. `peripherals.modem` is consumed by value and
//! is not handed back on `Err`, so it has to happen in each app's own
//! `run` where the peripherals are, and before the app spends the
//! quiet window on task stacks.

use std::sync::{Arc, Mutex};

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use mfsk_app_shared::{http_config, ntp, settings, udp_log, wifi};

/// PSRAM-backed, like `http_config`'s httpd task: this task's working
/// set is the driver's, not its own stack, and every KiB of internal
/// DRAM it does not reserve is a KiB the decoder's small allocations
/// can still find (`fst4_app`'s `NETWORK_STACK` comment has the
/// measurement this inherited).
const NETWORK_STACK: u32 = 24 * 1024;
/// Below every decode, capture and display task in all three apps.
const NETWORK_PRIORITY: u32 = 2;
/// One attempt, after association. Generous for `pool.ntp.org` over a
/// home network.
const NTP_SYNC_TIMEOUT_MS: u32 = 20_000;

/// How hard to chase an AP that is not answering.
#[derive(Clone, Copy)]
pub enum Policy {
    /// Retry forever. What `wspr_app` needs on its own test AP, and
    /// what costs decode throughput while it runs.
    Unbounded,
    /// `attempts` tries, then leave the radio alone for `idle_ms`.
    /// Four is what `wifi.rs`'s bisection found beats the AP
    /// comeback-time coin-flip; more costs decode throughput.
    Campaign { attempts: u32, idle_ms: u32 },
}

/// **The radio stays up, and stopping it was tried.** An earlier
/// version of this module could `esp_wifi_stop` once NTP had set the
/// clock, on the theory that the WiFi driver task (FreeRTOS priority
/// 23) was what cost an FT4 slot 400-580 ms when the network was
/// associated. Measured on a CoreS3 2026-09-01: **stopping it changed
/// nothing** — 1 786-1 919 ms with the radio stopped against
/// 1 789-1 969 ms associated — because `esp_wifi_stop` silences the
/// receiver while every buffer the driver allocated stays allocated.
///
/// The cost was memory, not CPU. Task stacks sized by inheritance
/// rather than measurement (32 KB asks against 2.6-4.5 KB of actual
/// use) had left the largest free internal block at 31 744 B, so the
/// decoder's own allocations fell to PSRAM — 41 % slower on the
/// 2 304-point workspace (`FT4_BENCHMARK.md` §26.3). With the stacks
/// sized from `board::log_task_stacks`, the same slot runs
/// **1 295-1 474 ms with WiFi associated**, faster than it ever ran
/// without the network, and the whole stop/resync mechanism had
/// nothing left to buy.

/// The FT4/FST4 default: four attempts, then three minutes of quiet.
pub const DECODE_FIRST: Policy = Policy::Campaign {
    attempts: 4,
    idle_ms: 180_000,
};

pub struct Config {
    /// FreeRTOS task name, and the log prefix.
    pub name: &'static str,
    pub policy: Policy,
    /// `WIFI_PS_MIN_MODEM`. Off only for a build measuring what the
    /// association costs.
    pub power_save: bool,
    /// Called with whether NTP synced.
    pub on_ntp: fn(bool),
}

struct Ctx {
    driver: BlockingWifi<EspWifi<'static>>,
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
    cfg: Config,
}

extern "C" fn entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn` leaked exactly this pointer via `Box::into_raw`,
    // and this is the only place that reclaims it.
    let ctx = unsafe { Box::from_raw(arg as *mut Ctx) };
    run(*ctx);
}

/// Background the whole sequence against a driver the caller already
/// constructed. Boot does not wait on any of it.
pub fn spawn(
    driver: BlockingWifi<EspWifi<'static>>,
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
    cfg: Config,
) {
    let name = cfg.name;
    let ptr = Box::into_raw(Box::new(Ctx { driver, nvs, cfg })) as *mut core::ffi::c_void;
    let caps = esp_idf_svc::sys::MALLOC_CAP_SPIRAM | esp_idf_svc::sys::MALLOC_CAP_8BIT;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCoreWithCaps(
            Some(entry),
            c"net".as_ptr(),
            NETWORK_STACK,
            ptr,
            NETWORK_PRIORITY,
            core::ptr::null_mut(),
            1, // core 1 — never core 0, which carries capture.
            caps,
        )
    };
    if created != 1 {
        log::error!("{name}: failed to create the network task");
        // SAFETY: the task was not created, so nothing else holds it.
        drop(unsafe { Box::from_raw(ptr as *mut Ctx) });
    }
}

fn run(mut ctx: Ctx) -> ! {
    let tag = ctx.cfg.name;
    let info = loop {
        let attempts = match ctx.cfg.policy {
            Policy::Unbounded => None,
            Policy::Campaign { attempts, .. } => Some(attempts),
        };
        match wifi::connect_with_retry(
            &mut ctx.driver,
            crate::WIFI_SSID,
            crate::WIFI_PSK,
            attempts,
        ) {
            Ok(i) => break i,
            Err(e) => match ctx.cfg.policy {
                // `connect_with_retry` only returns `Err` on a
                // malformed SSID/PSK when it was told to retry
                // forever — checked once, not worth retrying.
                Policy::Unbounded => {
                    log::error!("{tag}: WiFi setup failed permanently: {e:#}");
                    loop {
                        FreeRtos::delay_ms(60_000);
                    }
                }
                Policy::Campaign { attempts, idle_ms } => {
                    log::warn!(
                        "{tag}: no association after {attempts} attempts ({e:#}) — leaving \
                         the radio alone for {} s so it stops preempting the decode",
                        idle_ms / 1000,
                    );
                    FreeRtos::delay_ms(idle_ms);
                }
            },
        }
    };
    log::info!("{tag}: WiFi up, ip {}", info.ip);

    if ctx.cfg.power_save {
        // The default is `WIFI_PS_NONE`, which keeps the receiver
        // on continuously and hands every broadcast frame on the
        // LAN to a priority-23 driver task. `MIN_MODEM` lets the
        // radio sleep between DTIM beacons.
        let r = unsafe {
            esp_idf_svc::sys::esp_wifi_set_ps(
                esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_MIN_MODEM,
            )
        };
        log::info!("{tag}: esp_wifi_set_ps(MIN_MODEM) -> {r}");
    }

    // UDP log sink. On this board it is not a convenience: the USB
    // host driver takes the serial console with it when it
    // installs.
    let target_ip: std::net::IpAddr =
        if crate::UDP_LOG_TARGET.is_empty() || crate::UDP_LOG_TARGET == "auto" {
            std::net::IpAddr::V4(info.subnet_broadcast)
        } else {
            match crate::UDP_LOG_TARGET.parse() {
                Ok(ip) => ip,
                Err(e) => {
                    log::warn!(
                        "{tag}: UDP_LOG_TARGET '{}' parse failed ({e}); using subnet broadcast",
                        crate::UDP_LOG_TARGET
                    );
                    std::net::IpAddr::V4(info.subnet_broadcast)
                }
            }
        };
    let addr =
        std::net::SocketAddr::new(target_ip, crate::UDP_LOG_PORT.parse().unwrap_or(9999));
    match udp_log::UdpLogSink::new(addr) {
        Ok(sink) => {
            if let Ok(mut slot) = crate::FANOUT.udp.try_lock() {
                *slot = Some(sink);
            }
            crate::FANOUT.drain_staging_to_udp();
            log::info!("{tag}: UDP log sink up -> {addr}");
        }
        Err(e) => log::warn!("{tag}: UDP socket bind failed: {e}"),
    }

    // NTP: one attempt, now that WiFi is confirmed up. Absolute
    // time cannot come from anywhere else on a cold start, and an
    // FT4 or FST4 slot grid is meaningless without it.
    let initial_settings = {
        let g = ctx.nvs.lock().expect("settings NVS mutex poisoned");
        settings::load(&g)
    };
    let (ntp_synced, _sntp_keepalive) = if initial_settings.ntp_enabled {
        match ntp::start(&initial_settings.ntp_server) {
            Ok(sntp) => {
                let synced = ntp::wait_synced(&sntp, NTP_SYNC_TIMEOUT_MS);
                (synced, Some(sntp))
            }
            Err(e) => {
                log::warn!("{tag}: NTP start failed: {e:#}");
                (false, None)
            }
        }
    } else {
        log::info!("{tag}: NTP sync disabled in settings");
        (false, None)
    };
    if !ntp_synced {
        log::warn!("{tag}: NTP never synced");
    }
    (ctx.cfg.on_ntp)(ntp_synced);
    // **Keep watching, because the 20 s window is not the answer.**
    //
    // `wait_synced` counts its own delays, so a slow first exchange is
    // not a starved poll — it is simply a sync that had not arrived
    // yet. It then decided the clock source for the whole session:
    // measured on a radio 2026-09-19, two consecutive host-mode boots
    // timed out at 20 s and ran the rest of the session as `grid=rtc`,
    // which turns off the sink's UTC phase tracking entirely — so the
    // grid walked off at the audio path's rate error and decodes fell
    // to nothing, from a board whose RTC had been set from NTP minutes
    // earlier. The same SNTP handle keeps retrying underneath; nothing
    // was reading it.
    let mut watching_ntp = !ntp_synced && _sntp_keepalive.is_some();
    if watching_ntp {
        log::info!("{tag}: NTP still retrying underneath — watching for it");
    }

    let _http_server = match http_config::start(ctx.nvs.clone()) {
        Ok(s) => {
            log::info!("{tag}: HTTP config server up");
            Some(s)
        }
        Err(e) => {
            log::warn!("{tag}: HTTP config server failed: {e:#}");
            None
        }
    };

    // `ctx.driver` and `_http_server` are never read again but must
    // outlive this task — their `Drop`s tear down the association and
    // stop listening — so this task never returns.
    loop {
        FreeRtos::delay_ms(if watching_ntp { 5_000 } else { 60_000 });
        if !watching_ntp {
            continue;
        }
        let Some(sntp) = _sntp_keepalive.as_ref() else {
            watching_ntp = false;
            continue;
        };
        // One status read at priority 2, every 5 s, until it lands. The
        // promotion itself is `ntp::note_sync_completed` — the same
        // `note_clock_from_ntp` the initial wait would have called, so
        // the RTC write and the sink's phase tracking come up exactly
        // as they do on a fast first exchange.
        if ntp::note_sync_completed(sntp) {
            watching_ntp = false;
            log::info!("{tag}: NTP synced on a later attempt — UTC now owns the slot phase");
            (ctx.cfg.on_ntp)(true);
        }
    }
}
