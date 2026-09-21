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
//! ## Everything, including the driver and the decision
//!
//! Driver construction used to be listed here as the one part that
//! could not be shared — `peripherals.modem` is consumed by value and
//! is not handed back on `Err`, so it lived in each app's own `run`.
//! `boot` takes the modem out of `Peripherals` before it dispatches
//! into a receiver, so [`bring_up`] now owns the whole thing: the
//! three-way decision (the CONFIG page's `wifi_pref`, an empty
//! `WIFI_SSID`, a receiver that asked for no radio), the driver, and
//! the task above.
//!
//! That decision existed in **four** copies for one day (#381). The
//! FT8 controller had a fifth implementation of the *sequence* as
//! well, hand-rolled in `main.rs` on its own thread: `connect_sta`,
//! its own UDP-sink install with a 30-attempt retry, its own NTP wait
//! and its own late-sync watch. The retry was the one thing it had
//! that this did not, and it is here now, so nothing was lost by
//! deleting it.

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
    /// `attempts` tries, then stop trying for this boot.
    ///
    /// What the FT8 controller did before it came here: `connect_sta`
    /// gives up after `CONNECT_MAX_ATTEMPTS` and its caller had no
    /// retry around it. Kept as its own variant rather than folded
    /// into a long `idle_ms`, because the two say different things to
    /// the next reader — this one is "there is no AP here", and the
    /// board will not spend priority-23 time discovering that again.
    Once { attempts: u32 },
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

/// How far [`bring_up`] takes the radio. Anything but [`Bringup::Connect`]
/// is a diagnostic build.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Bringup {
    /// Init the driver and hand it to the network task. Every shipped
    /// build.
    Connect,
    /// Init the driver and never associate — `fst4_app`'s
    /// `MFSK_FST4_APP_NO_CONNECT`, which separates the driver's
    /// *memory* from its *CPU*. `stop_radio` additionally calls
    /// `esp_wifi_stop`, which silences the receiver while every buffer
    /// the driver allocated stays allocated — that asymmetry is the
    /// measurement.
    DriverOnly { stop_radio: bool },
}

pub struct Config {
    /// FreeRTOS task name, and the log prefix.
    pub name: &'static str,
    pub policy: Policy,
    /// `WIFI_PS_MIN_MODEM`. Off only for a build measuring what the
    /// association costs, and for the FT8 controller, which has never
    /// been measured with it (see this module's `power_save` note).
    pub power_save: bool,
    /// Whether to sync NTP once the association is up.
    ///
    /// False for a receiver taking its slot phase off the air
    /// (`GridSource::AirDt`): starting NTP there spends the timeout and
    /// then disciplines a clock the grid is deliberately not following.
    pub ntp: bool,
    /// What this receiver loses when the radio stays down, named in the
    /// one warning that says so — "no NTP, no UDP log, no config page"
    /// for FT4, wsprnet for WSPR. One line per receiver, one place.
    pub without: &'static str,
    pub bringup: Bringup,
    /// Whether to serve the HTTP config page.
    ///
    /// False for the FT8 controller, which has never had one: its
    /// network sequence was hand-rolled in `main.rs` and stopped at
    /// NTP. Unifying the sequence would have handed it an httpd task
    /// as a side effect, and a refactor that adds a listening socket to
    /// a shipped receiver is not a refactor. One line, when it is
    /// wanted on purpose and with the memory measured.
    pub http: bool,
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

/// The one WiFi decision and the one bring-up, for every receiver.
///
/// `cfg: None` is a receiver that asked for no radio at all
/// (`fst4_app`'s `MFSK_FST4_APP_NO_WIFI`); the other two ways to end up
/// without one — the CONFIG page's `WIFI: OFF` and an empty
/// `WIFI_SSID` — are decided here, so they are decided once. Each says
/// what is lost via [`Config::without`], because "no WiFi" costs a
/// WSPR receiver its wsprnet upload and an FT8 controller its only
/// console, and a warning that does not say which is a warning nobody
/// can act on.
///
/// Consumes the modem either way: there is no second chance at it this
/// boot, and pretending otherwise would invite a caller to retry into
/// a panic.
pub fn bring_up<M>(
    modem: M,
    nvs_part: esp_idf_svc::nvs::EspDefaultNvsPartition,
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
    cfg: Option<Config>,
) where
    M: esp_idf_svc::hal::modem::WifiModemPeripheral + 'static,
{
    let Some(cfg) = cfg else {
        // A receiver that wants no radio has already said why, in its
        // own words — `fst4_app`'s diagnostic flag, or a mode with no
        // use for a network. Nothing to add.
        return;
    };
    let tag = cfg.name;
    if !crate::wifi_pref().enabled() {
        // The CONFIG page's choice. Issue #381: this used to be four
        // copies of the same `if`, one per receiver, and before that it
        // was not asked at all — a board told to take its phase off the
        // air still ran an association campaign over the slots a cold
        // acquisition needs.
        log::warn!("{tag}: WIFI: OFF (CONFIG page) — {}", cfg.without);
        return;
    }
    if crate::WIFI_SSID.is_empty() {
        log::warn!("{tag}: WIFI_SSID empty (no cfg.toml) — {}", cfg.without);
        return;
    }
    let sysloop = match esp_idf_svc::eventloop::EspSystemEventLoop::take() {
        Ok(s) => s,
        Err(e) => {
            log::error!("{tag}: sysloop take failed: {e:#} — no WiFi this boot");
            return;
        }
    };
    let driver = match wifi::wifi_driver_init(modem, sysloop, Some(nvs_part)) {
        Ok(d) => d,
        Err(e) => {
            log::error!("{tag}: WiFi driver init failed (permanent this boot): {e:#}");
            return;
        }
    };
    // **Synchronous, and deliberately here.** The driver's own
    // internal-DRAM claim is what the caller ordered its task stacks
    // around; only the slow half (associate/DHCP/NTP) is backgrounded.
    crate::log_free_internal("post-wifi-driver-init");
    match cfg.bringup {
        Bringup::Connect => {
            // `uac` waits for a log sink only when one is coming. The
            // flag is set here, where the last thing that could stop it
            // has already not happened, rather than from a caller's
            // guess about what this function will decide.
            crate::WIFI_ENABLED.store(true, std::sync::atomic::Ordering::Release);
            spawn(driver, nvs, cfg)
        }
        Bringup::DriverOnly { stop_radio } => {
            log::warn!("{tag}: driver up, no association (diagnostic build)");
            if stop_radio {
                // SAFETY: no arguments; the driver above is live.
                let r = unsafe { esp_idf_svc::sys::esp_wifi_stop() };
                log::warn!("{tag}: esp_wifi_stop() -> {r} (radio silenced, memory retained)");
            }
            // The handles must outlive this function or `Drop` tears
            // the driver down, which is the opposite of the experiment.
            core::mem::forget(driver);
        }
    }
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
            Policy::Campaign { attempts, .. } | Policy::Once { attempts } => Some(attempts),
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
                Policy::Once { attempts } => {
                    // No log sink, so this line only reaches a serial
                    // console — which in UAC mode does not exist. The
                    // panel is what is left, and it says `wifi` is down.
                    log::warn!(
                        "{tag}: no association after {attempts} attempts ({e:#}) — not \
                         trying again this boot"
                    );
                    loop {
                        FreeRtos::delay_ms(60_000);
                    }
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
    // **Retried, because a single bind is not enough.** The FT8
    // controller's own copy of this learned it: the socket can refuse
    // the bind for a moment after DHCP returns, and `FANOUT.udp` is a
    // `try_lock` that can lose a race with a task already logging. One
    // attempt leaves a board with no console at all in UAC mode, where
    // this sink *is* the console. Six seconds of 200 ms retries cost
    // nothing on a board that has one.
    let mut installed = false;
    for attempt in 0..30u32 {
        match udp_log::UdpLogSink::new(addr) {
            Ok(sink) => match crate::FANOUT.udp.try_lock() {
                Ok(mut slot) => {
                    *slot = Some(sink);
                    installed = true;
                }
                Err(_) => {
                    // Someone is writing a line right now. Drop this
                    // socket and build another on the next pass.
                }
            },
            Err(e) => {
                if attempt == 0 {
                    log::warn!("{tag}: UDP socket bind failed: {e} — retrying");
                }
            }
        }
        if installed {
            crate::FANOUT.drain_staging_to_udp();
            log::info!("{tag}: UDP log sink up -> {addr} (attempt {})", attempt + 1);
            break;
        }
        FreeRtos::delay_ms(200);
    }
    if !installed {
        log::error!("{tag}: UDP log sink never installed — this board may now be silent");
    }

    // NTP: one attempt, now that WiFi is confirmed up. Absolute
    // time cannot come from anywhere else on a cold start, and an
    // FT4 or FST4 slot grid is meaningless without it.
    let initial_settings = {
        let g = ctx.nvs.lock().expect("settings NVS mutex poisoned");
        settings::load(&g)
    };
    let (ntp_synced, _sntp_keepalive) = if !ctx.cfg.ntp {
        // `GridSource::AirDt`: the phase comes from the band, so a
        // sync would spend its timeout and then discipline a clock the
        // grid is not following. The RTC still holds the minute for the
        // log — see `grid_src`'s module docs.
        log::info!("{tag}: NTP not started — grid source is the air (CONFIG page)");
        (false, None)
    } else if initial_settings.ntp_enabled {
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

    let _http_server = if !ctx.cfg.http {
        None
    } else {
        match http_config::start(ctx.nvs.clone()) {
            Ok(s) => {
                log::info!("{tag}: HTTP config server up");
                Some(s)
            }
            Err(e) => {
                log::warn!("{tag}: HTTP config server failed: {e:#}");
                None
            }
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
