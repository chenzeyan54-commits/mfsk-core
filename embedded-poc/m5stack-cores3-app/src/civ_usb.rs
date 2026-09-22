//! CAT over USB: the IC-705's CI-V port (activator plan, stage 3).
//!
//! The IC-705 enumerates as one composite device, VID `0c26` PID
//! `0036`, carrying **two** CDC-ACM functions — control interfaces 0
//! and 2 (data 1 and 3) — beside the PCM2901 audio behind the same
//! internal hub (descriptor dump, `logs/udp_enumdump2_2026-09-20.log`).
//! Which of the two is CI-V is not in the descriptors; the radio's
//! manual calls them "USB (A)" and "USB (B)", and (B) is the one whose
//! function the operator chooses (RTTY decode / DV data / GPS out).
//!
//! This first step is a **probe**, gated at build time
//! (`MFSK_CORES3_CIV_PROBE=1`): it opens each port in turn, sends only
//! *reads* — `03` (frequency) and `26 00` (mode) — and logs every
//! frame that comes back, so one session answers which port talks
//! CI-V and whether transceive frames arrive when the dial turns.
//!
//! It never touches DTR or RTS. The IC-705's "USB SEND" / "USB Keying"
//! settings can map either line to PTT or CW keying, and
//! `cdc_acm_host_open` leaves both alone unless asked; so does this.

use esp_idf_svc::sys;
use esp_idf_svc::sys::cdc_acm as cdc;
use mfsk_app_shared::civ_frame::{self, IC705_ADDR};
use std::sync::Mutex;

const IC705_VID: u16 = 0x0c26;
const IC705_PID: u16 = 0x0036;
/// The two CDC **data** interfaces in the descriptor dump, opened
/// directly rather than through their control interfaces (0 and 2).
///
/// The ESP32-S3's host controller has eight channels
/// (`OTG_NUM_HOST_CHAN`), one per open pipe. The IC-705 already takes
/// seven: EP0 for the hub, the Icom function and the PCM2901, the
/// hub's interrupt pipe, audio IN and audio OUT. Opening a control
/// interface adds its notification pipe beside the two bulk ones, and
/// on 2026-09-23 that starved the PCM2901, which then failed to
/// enumerate ("No more HCD channels available", `EXT_PORT: [1:4] Port
/// disabled`) and the board rebooted every ~33 s. Given the data
/// interface, `cdc_parse_interface_descriptor` finds no interrupt
/// endpoint and does not treat it as CDC-compliant, so only the two
/// bulk pipes are allocated. The cost is that the driver offers no
/// line-coding / control-line calls on the handle — which this does
/// not want anyway (see the module doc on DTR/RTS).
const PORTS: [u8; 2] = [1, 3];

/// Build-time gate, as `MFSK_CORES3_TX_PROBE` is for the audio OUT
/// probe: the normal image carries no CDC client until the port is
/// known.
pub const PROBE_ENABLED: bool = option_env!("MFSK_CORES3_CIV_PROBE").is_some();

/// Below the audio path (`uac.rs` runs it at 8) — a CI-V reply that
/// waits a few milliseconds costs nothing, a missed isochronous frame
/// costs audio. Core 1, away from the USB interrupt and the decoder.
const DRIVER_TASK_PRIORITY: u32 = 5;
const DRIVER_TASK_STACK: usize = 4096;
const DRIVER_TASK_CORE: i32 = 1;

/// One frame reassembler per port: a bulk transfer is not promised to
/// carry exactly one frame.
/// Set by the RX callback when a frame from the radio parses.
static ANSWERED: [core::sync::atomic::AtomicBool; 2] =
    [core::sync::atomic::AtomicBool::new(false), core::sync::atomic::AtomicBool::new(false)];

static READERS: [Mutex<civ_frame::Reader>; 2] =
    [Mutex::new(civ_frame::Reader::new()), Mutex::new(civ_frame::Reader::new())];

/// Install the CDC-ACM class driver and start the probe. Call after
/// `usb_host_install` (i.e. after `uac::start_host` succeeds).
pub fn start_probe() {
    if !PROBE_ENABLED {
        return;
    }
    let cfg = cdc::cdc_acm_host_driver_config_t {
        driver_task_stack_size: DRIVER_TASK_STACK,
        driver_task_priority: DRIVER_TASK_PRIORITY,
        xCoreID: DRIVER_TASK_CORE,
        new_dev_cb: None,
    };
    // SAFETY: the host library is installed; `cfg` outlives the call.
    let err = unsafe { cdc::cdc_acm_host_install(&cfg) };
    if err != sys::ESP_OK {
        log::warn!("civ: cdc_acm_host_install failed err={err:#x}");
        return;
    }
    log::warn!("civ: CDC-ACM driver installed — probing IC-705 ports {PORTS:?}");
    let spawned = crate::uac::spawn_psram_thread(c"civ_probe", 4096, Some(4), None, probe_task);
    if let Err(e) = spawned {
        log::warn!("civ: probe thread spawn failed: {e}");
    }
}

unsafe extern "C" fn on_data(data: *const u8, len: usize, arg: *mut core::ffi::c_void) -> bool {
    let port = arg as usize;
    // SAFETY: the driver hands a buffer of `len` bytes valid for the
    // duration of the callback.
    let bytes = unsafe { core::slice::from_raw_parts(data, len) };
    let Some(reader) = READERS.get(port) else {
        return true;
    };
    let Ok(mut reader) = reader.lock() else {
        return true;
    };
    for &b in bytes {
        if let Some(frame) = reader.push(b) {
            let mut hex: heapless::String<{ civ_frame::MAX_FRAME * 3 }> = heapless::String::new();
            for x in frame.iter() {
                use core::fmt::Write as _;
                let _ = write!(&mut hex, "{x:02X} ");
            }
            // The radio echoes what the controller sent on USB, so a
            // frame from E0 is our own and `parse` returns None for it.
            let ev = civ_frame::parse(&frame, IC705_ADDR);
            if ev.is_some() {
                ANSWERED[port].store(true, core::sync::atomic::Ordering::Relaxed);
            }
            log::warn!("civ: port {} rx [{}] -> {:?}", PORTS[port], hex.trim_end(), ev);
        }
    }
    true
}

unsafe extern "C" fn on_event(ev: *const cdc::cdc_acm_host_dev_event_data_t, arg: *mut core::ffi::c_void) {
    let port = PORTS.get(arg as usize).copied().unwrap_or(0xff);
    // SAFETY: the driver passes a valid event for the callback's span.
    let ty = unsafe { (*ev).type_ };
    log::warn!("civ: port {port} event {ty}");
}

fn open(idx: usize) -> Option<cdc::cdc_acm_dev_hdl_t> {
    let cfg = cdc::cdc_acm_host_open_config_t {
        vid: IC705_VID,
        pid: IC705_PID,
        interface_idx: PORTS[idx],
        dev_addr: 0,
        connection_timeout_ms: 10_000,
        out_buffer_size: 64,
        in_buffer_size: 0,
        event_cb: Some(on_event),
        data_cb: Some(on_data),
        user_arg: idx as *mut core::ffi::c_void,
    };
    let mut hdl: cdc::cdc_acm_dev_hdl_t = core::ptr::null_mut();
    // SAFETY: `cfg` outlives the call; `hdl` is written on success.
    let err = unsafe { cdc::cdc_acm_host_open_v2(&cfg, &mut hdl) };
    if err != sys::ESP_OK {
        log::warn!("civ: open iface {} failed err={err:#x}", PORTS[idx]);
        return None;
    }
    log::warn!("civ: opened iface {}", PORTS[idx]);
    Some(hdl)
}

fn send(hdl: cdc::cdc_acm_dev_hdl_t, port: u8, frame: &[u8]) {
    // SAFETY: `hdl` is open; `frame` outlives the blocking call.
    let err = unsafe { cdc::cdc_acm_host_data_tx_blocking(hdl, frame.as_ptr(), frame.len(), 200) };
    if err != sys::ESP_OK {
        log::warn!("civ: port {port} tx failed err={err:#x}");
    }
}

fn probe_task() {
    // Audio first: open only once the PCM2901's IN stream has
    // enumerated, so a channel shortage can fail this probe and never
    // the receiver.
    let mut waited = 0u32;
    while crate::uac::RX_IFACE_SEEN.load(core::sync::atomic::Ordering::Relaxed) < 0 {
        if waited >= 60 {
            log::warn!("civ: no audio IN after {waited} s — probing anyway");
            break;
        }
        std::thread::sleep(std::time::Duration::from_secs(1));
        waited += 1;
    }
    // One port at a time, so at most two bulk pipes are ever held.
    for idx in 0..PORTS.len() {
        let Some(h) = open(idx) else { continue };
        // Six polls 5 s apart, then keep the port that answered.
        for round in 0..6 {
            log::warn!("civ: round {round} port {} tx read_freq + read_mode", PORTS[idx]);
            send(h, PORTS[idx], &civ_frame::read_freq(IC705_ADDR));
            send(h, PORTS[idx], &civ_frame::read_mode(IC705_ADDR));
            std::thread::sleep(std::time::Duration::from_secs(5));
        }
        if ANSWERED[idx].load(core::sync::atomic::Ordering::Relaxed) {
            log::warn!("civ: port {} answers CI-V; listening for transceive frames", PORTS[idx]);
            return;
        }
        log::warn!("civ: port {} never answered; closing", PORTS[idx]);
        // SAFETY: opened above, not used after this.
        let err = unsafe { cdc::cdc_acm_host_close(h) };
        if err != sys::ESP_OK {
            log::warn!("civ: close port {} failed err={err:#x}", PORTS[idx]);
        }
    }
    log::warn!("civ: no port answered");
}
