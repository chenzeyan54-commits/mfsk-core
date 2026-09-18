// SPDX-License-Identifier: GPL-3.0-or-later
//! The CoreS3 application's library half.
//!
//! Everything the binary is made of lives here so that **other bins in
//! this crate can reach it too** — `ft4-demo` needs `board` and
//! `display`, and a bin cannot see another bin's modules. Before this
//! split there was no lib target and each bin was an island, which is
//! why `audio_out` and the FT4 receiver could be written but not wired
//! to anything that draws a screen.
//!
//! `main.rs` is now only `fn main()`: boot mode, log fanout, WiFi, and
//! the dispatch into `apps`.

pub mod apps;
pub mod audio_out;
pub mod board;
pub mod coredump;
pub mod decode_pipeline;
pub mod display;
pub mod esp_log_bridge;
pub mod log_slot;
pub mod net;
pub mod pmic;
pub mod rtc;
pub mod spot_panel;
pub mod touch;
pub mod uac;

use esp_idf_svc::sys::{
    heap_caps_get_free_size, heap_caps_get_largest_free_block, MALLOC_CAP_8BIT, MALLOC_CAP_INTERNAL,
};
use log::LevelFilter;

use mfsk_app_shared::log_sink::{FanoutLogger, LogFanout};

pub fn log_free_internal(label: &str) {
    let caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    let free = unsafe { heap_caps_get_free_size(caps) };
    let largest = unsafe { heap_caps_get_largest_free_block(caps) };
    log::info!("[mem] {label} free_internal={free} largest={largest}");
}

pub static FANOUT: LogFanout = LogFanout::new();

/// この起動で WiFi を立ち上げるか。`display` が「ログ送信先を待つか」の
/// 判断に使う — 来ない sink を45秒待つのは、起動が45秒遅い受信機に
/// なるだけ。Refs #163.
pub static WIFI_ENABLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Where this boot takes the slot grid's phase from — the CONFIG page's
/// setting, read once at startup and published here so the picker can
/// mark it and `main` can act on it without re-opening NVS.
static GRID_SOURCE: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(0);

pub fn set_grid_source(src: mfsk_app_shared::grid_src::GridSource) {
    GRID_SOURCE.store(
        match src {
            mfsk_app_shared::grid_src::GridSource::Ntp => 0,
            mfsk_app_shared::grid_src::GridSource::AirDt => 1,
        },
        std::sync::atomic::Ordering::Release,
    );
}

pub fn grid_source() -> mfsk_app_shared::grid_src::GridSource {
    match GRID_SOURCE.load(std::sync::atomic::Ordering::Acquire) {
        1 => mfsk_app_shared::grid_src::GridSource::AirDt,
        _ => mfsk_app_shared::grid_src::GridSource::Ntp,
    }
}

/// Persist a CONFIG-page choice and restart into it.
///
/// Same shape as `boot_mode::commit_and_restart`, and for the same
/// reason: the panel tasks run on PSRAM stacks and an NVS write
/// disables the flash cache, which a PSRAM stack must not be holding.
pub fn commit_grid_src_and_restart(
    nvs: std::sync::Arc<std::sync::Mutex<esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>>>,
    src: mfsk_app_shared::grid_src::GridSource,
) {
    struct Req {
        nvs: std::sync::Arc<
            std::sync::Mutex<esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>>,
        >,
        src: mfsk_app_shared::grid_src::GridSource,
    }

    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `commit_grid_src_and_restart` leaked exactly this pointer.
        let req = unsafe { Box::from_raw(arg as *mut Req) };
        match req.nvs.lock() {
            Ok(nvs) => match mfsk_app_shared::grid_src::write(&nvs, req.src) {
                Ok(()) => log::warn!("grid source: committed {} — restarting", req.src.label()),
                Err(e) => log::error!("grid source write failed: {e} — not restarting"),
            },
            Err(e) => log::error!("grid source: NVS lock poisoned: {e} — not restarting"),
        }
        drop(req);
        // Let the line reach the log sink; in UAC mode that is the only
        // channel out of this board.
        unsafe { esp_idf_svc::sys::vTaskDelay(40) };
        // SAFETY: no arguments, does not return.
        unsafe { esp_idf_svc::sys::esp_restart() };
    }

    let ptr = Box::into_raw(Box::new(Req { nvs, src })) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(entry),
            c"grid_src_save".as_ptr(),
            4096,
            ptr,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("could not spawn the grid-source save task");
        drop(unsafe { Box::from_raw(ptr as *mut Req) });
    }
}

pub fn wifi_enabled_for_this_boot() -> bool {
    WIFI_ENABLED.load(std::sync::atomic::Ordering::Acquire)
}
pub static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, LevelFilter::Info);

pub const WIFI_SSID: &str = env!("WIFI_SSID");
pub const WIFI_PSK: &str = env!("WIFI_PSK");
pub const UDP_LOG_TARGET: &str = env!("UDP_LOG_TARGET");
pub const UDP_LOG_PORT: &str = env!("UDP_LOG_PORT");
pub const BOOT_MODE_DEFAULT: &str = env!("BOOT_MODE_DEFAULT");

/// SNTP server for the FT8 controller. WSPR and FST4 take theirs from
/// NVS settings, which this app has no page for; `pool.ntp.org` is what
/// their own default is.
pub const NTP_SERVER: &str = "pool.ntp.org";
/// Long enough for a first sync over WiFi, short enough that a boot
/// with no route still reaches the decode loop.
pub const NTP_SYNC_TIMEOUT_MS: u32 = 20_000;
