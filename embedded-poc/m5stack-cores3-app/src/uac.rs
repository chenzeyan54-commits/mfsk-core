//! USB Audio Class host capture — Phase 1 iso IN streaming (#163).
//!
//! `start_host()` installs the ESP-IDF USB host stack + the
//! `espressif/usb_host_uac` class driver, registers a driver event
//! callback that forwards hot-plug events to an app task, spawns the
//! USB events pump, and falls through. From there:
//!
//! - **driver_event_cb** (class-driver task ctx) pushes `RxConnected` /
//!   `TxConnected` onto a `std::sync::mpsc::channel`.
//! - **app_task** (`uac_app`, std::thread) consumes events; on the first
//!   `RxConnected` it calls `uac_host_device_open` + `uac_host_device_start`
//!   with the IC-705's fixed `48 kHz / stereo / 16-bit` config and
//!   spawns the reader thread.
//! - **reader_thread** (`uac_reader`, std::thread) polls
//!   `uac_host_device_read` into a 4 KB stack buffer, accumulates stats,
//!   and logs `bytes/packets/errors` to UDP every ~1 s.
//!
//! The reader resamples 48 k stereo → 12 k mono and pushes into
//! whichever [`AudioSink`] the running binary registered via
//! [`set_audio_sink`] — `main.rs`'s FT8 controller wires
//! [`set_chunk_q`] (its pre-existing chunk-queue sink, now
//! [`Ft8ChunkSink`] under the hood), `wspr_app.rs` wires its own DDC
//! push sink. Samples are dropped on the floor only while no sink is
//! registered yet (race window at boot, bounded). Disconnect and
//! re-open are handled: `DISCONNECTED` sets [`READER_STOP_REQUESTED`],
//! a stall watchdog covers the overflow case that ends the stream
//! without ever returning an error, and the re-open gate retries the
//! interface. What is left is telling "same device returned" from
//! "new device" in [`app_task`] — no issue filed for it.
//!
//! `TxConnected` (the IC-705's USB audio OUT interface) enumerates
//! today and is otherwise ignored — FT8 RX never needed it. Phase T0
//! of the TX/QSO feasibility work adds an opt-in probe,
//! `handle_tx_connected`, gated on `TX_PROBE_ENABLED`
//! (`MFSK_CORES3_TX_PROBE` at build time): open the interface, write
//! digital silence a fixed number of times, close it again. Silence
//! only — see that function's doc comment for why a nonzero test tone
//! is not safe to send without knowing the radio's PTT-source setting.
//!
//! ## 接続方式 (確定済)
//!
//! - Component: `espressif/usb_host_uac@^1.4` を `Cargo.toml` の
//!   `extra_components` 経由で managed component として取得。
//!   bindings は `esp_idf_svc::sys::uac::*` に生成。
//! - IC-705 USB Audio は **固定 48 kHz / stereo / 16-bit** (16 kHz は
//!   selectable ではない)。S3 側で stereo → mono (L ch 抽出 / R ch 破棄)
//!   + 48 kHz → 12 kHz の 4:1 decimation を `embedded_shared` 経由で
//!   行う。
//! - Reference example は esp-usb 上流の
//!   `host/class/uac/usb_host_uac/examples/audio_player/main/main.c`。
//!   init 順序 (`usb_host_install` → events task spawn →
//!   `uac_host_install` → `RX_CONNECTED` 通知で device_open →
//!   device_start) をそのまま Rust に移植する。
//!
//! ## OTG 排他
//!
//! `usb_host_install()` を呼んだ瞬間に USB-Serial-JTAG endpoint が
//! detach されるため、`BootMode::Uac` では WiFi STA + UDP log を必ず
//! 起動させる (`main.rs` dispatch arm で強制)。
//!
//! ## 進捗
//!
//! - [x] managed component + bindings
//! - [x] host install + hot-plug callback
//! - [x] iso IN streaming + stats logging (このファイル)
//! - [x] 48 kHz stereo → 12 kHz mono resampler + sink push (FT8
//!   side; `wspr_app.rs` wires its own DDC-push sink the same way)
//! - [x] verification on hardware — #163, closed 2026-08-23: an
//!   IC-705 sustained 192,512 B/s for ten minutes (125 MB, 30,520
//!   packets, zero errors) with the FT8 decode pipeline running slots
//!   off the live stream. `wspr_app` / `fst4_app` share this file but
//!   have not been run against a radio (#313).
//! - [x] disconnect/reconnect — the DISCONNECTED signal, the stall
//!   watchdog and the re-open gate all shipped with #163; "same device
//!   returned" detection in [`app_task`] is what remains.

use std::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use std::sync::mpsc::{channel, Sender};
use std::sync::{Mutex, OnceLock};

use anyhow::{anyhow, Result};
use embedded_shared::pipeline::{send_box, ChunkMsg, CHUNK_LEN};
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::sys;
use mfsk_core::engine::dsp::resample::LinearResamplerI16To12k;

/// Spawn a named `std::thread` with its stack allocated from PSRAM
/// instead of internal DRAM.
///
/// Root-caused during the `wspr_app` crash-loop investigation
/// (2026-08-16): `spawn_network_task`'s own doc comment already
/// records that `wifi_driver_init` alone drives internal DRAM from
/// ~57 KB free down to ~10 KB free / 7 KB largest contiguous block —
/// and that measurement predates the `fst4-bench` factory-partition
/// growth the same day. A real device log
/// (`wspr_app_restore_2026-08-16f.log`) showed **2 KB** free after
/// `wifi_driver_init`, and every one of this module's three
/// `std::thread::Builder` spawns (`uac_app` 4 KiB, `usb_events`
/// 4 KiB, `uac_reader` 10 KiB) allocated its stack from that same
/// internal-DRAM pool via the ESP-IDF pthread compat layer — the
/// first (`uac_app`) failed outright (`Failed to create task!` /
/// `Not enough space`), which was already a known, gracefully-logged
/// condition. What wasn't understood until this investigation: with
/// internal DRAM this tight, small (<4 KiB —
/// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096` forces anything below
/// that size into internal DRAM regardless of PSRAM availability)
/// allocations made later by `scan_loop`/`ddc_loop` on the *other*
/// tasks can fail too, and on this esp-idf-svc std target an alloc
/// failure surfaces as a genuine Rust panic rather than an abort —
/// which then poisons whichever `std::sync::Mutex` it held
/// (`BASEBAND_BUFS`/`DDC_READY_IDX`/`WSPR_UI`/`ctx.nvs`), so the next
/// task to touch that lock panics too. That's the double-panic crash
/// loop, and it explains why the observed backtrace lands in a
/// different task on different boots — it's whichever task loses the
/// race to be second.
///
/// Mirrors `spawn_network_task`'s existing PSRAM-stack fix (same
/// `MALLOC_CAP_SPIRAM`/`CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y`
/// mechanism, just via `std::thread`'s `esp_pthread_cfg_t` hook
/// instead of `xTaskCreatePinnedToCoreWithCaps` since these three are
/// `std::thread`s, not raw FreeRTOS tasks) rather than inventing a
/// new one. `ThreadSpawnConfiguration::set()` only affects spawns
/// made by *this* calling thread (it's per-caller state in the IDF
/// pthread layer, not global), so this is reset back to the default
/// config immediately after spawning — the calling thread's own
/// stack, and anything it spawns later without going through this
/// helper, is unaffected.
fn spawn_psram_thread<F>(
    name: &'static core::ffi::CStr,
    stack_size: usize,
    priority: Option<u8>,
    pin_to_core: Option<esp_idf_svc::hal::cpu::Core>,
    f: F,
) -> std::io::Result<std::thread::JoinHandle<()>>
where
    F: FnOnce() + Send + 'static,
{
    // `name` goes into the spawn configuration, not into
    // `Builder::name()`. The latter is Rust-side only, so every task
    // here reported as 'pthread' — and a coredump that says
    // `task 'pthread'` cannot tell four candidate threads apart, which
    // is where a stack-overflow hunt stalls. Refs #163.
    let default_cfg_for_prio = ThreadSpawnConfiguration::default();
    let psram_cfg = ThreadSpawnConfiguration {
        name: Some(name),
        stack_size,
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        priority: priority.unwrap_or(default_cfg_for_prio.priority),
        pin_to_core: pin_to_core.or(default_cfg_for_prio.pin_to_core),
        ..default_cfg_for_prio
    };
    if let Err(e) = psram_cfg.set() {
        log::warn!("uac: ThreadSpawnConfiguration::set (PSRAM stack) failed for {name:?}: {e:?} — falling back to internal-DRAM stack");
    }
    let result = std::thread::Builder::new().stack_size(stack_size).spawn(f);
    // Restore the default (internal-DRAM) config regardless of
    // whether the spawn above succeeded, so this calling thread's
    // own subsequent spawns (if any) aren't silently left on PSRAM.
    let default_cfg = ThreadSpawnConfiguration::default();
    if let Err(e) = default_cfg.set() {
        log::warn!("uac: ThreadSpawnConfiguration::set (restore default) failed: {e:?}");
    }
    result
}

/// Newtype around the IDF `uac_host_device_handle_t` (`*mut uac_interface`)
/// to assert thread-safety for the `move` into the reader thread. The
/// IDF UAC driver documents the handle as safe to call from any task
/// once `uac_host_device_start` returns ESP_OK.
struct DeviceHandle(sys::uac::uac_host_device_handle_t);
// SAFETY: per usb_host_uac docs, the handle is opaque to callers and
// the IDF synchronises internal state. We never mutate the pointer or
// dereference its target on the Rust side — every use goes through
// `uac_host_device_*` IDF calls.
unsafe impl Send for DeviceHandle {}

/// USB host event-pump task stack — modest budget; the loop just
/// blocks on `usb_host_lib_handle_events` and dispatches flags.
const USB_EVENTS_TASK_STACK: usize = 4096;

/// UAC class-driver background-task config. 1.4.x supports
/// `tskNO_AFFINITY` but pinning to core 0 (PRO_CPU) matches the upstream
/// audio_player example and keeps the decoder's core 1 (APP_CPU) free.
const UAC_DRIVER_TASK_STACK: usize = 4096;
/// **6, the same as [`AUDIO_TASK_PRIORITY`] and for the same reason.**
///
/// This was 5 while the reader was raised to 6 on 2026-09-19, which
/// raised one half of the audio path and left the half feeding it
/// sharing core 0 with the decode thread at equal priority — 100 Hz
/// round-robin, so the class driver got about half a core for as long
/// as a decode ran. That is visible in the honest priority argument the
/// reader's constant already makes: a 16 KB ring is 85 ms of audio, and
/// nothing in the path may stall longer than that.
const UAC_DRIVER_TASK_PRIORITY: usize = 6;
const UAC_DRIVER_TASK_CORE: sys::BaseType_t = 0;

/// `uac_app` task stack. Just runs `recv()` → device_open/start →
/// spawn reader. 4 KB is overkill but keeps headroom for the OnceLock
/// + sender state and any future device-cleanup paths.
const APP_TASK_STACK: usize = 4096;

/// Priority of whichever task is feeding the [`AudioSink`] — the USB
/// reader on a radio, the `MFSK_CORES3_SIM` feeder without one.
///
/// **6, above the decode thread's 5**, for the reason `stage1_inc`
/// takes 6 above `dsp_worker`'s 5: the audio path must be able to
/// preempt the decoder, because its work cannot be deferred and then
/// caught up on.
///
/// Both were priority 5 until 2026-09-19, and pthreads take
/// `CONFIG_PTHREAD_TASK_CORE_DEFAULT` = no affinity, so the audio task
/// and the decode task time-sliced one core at 100 Hz whenever the
/// decoder ran long. Cold acquisition runs ~13 s on the decode thread
/// with two `vTaskDelay(1)`s in it, and the sink's slot-boundary
/// publishes fell **4.06 s** behind over one
/// (`logs/sim_slotlen_rerun_noclock_offset3000_2026-09-19.log`, the
/// per-slot `hint_err` field) — enough to make
/// `decode_pipeline::slot_end_hint` name the wrong slot's boundary.
///
/// On the SIM feeder that is lag and nothing worse; it paces itself
/// and the audio is a `&'static [u8]`. On a radio it is loss: the
/// reader drains the IDF UAC ring, [`STREAM_BUFFER_BYTES`] = 16 KB =
/// **85 ms** at 48 kHz stereo, and `pipeline::send_box` into a
/// four-chunk (400 ms) queue blocks rather than dropping, so a
/// starved reader stops reading and the ring overruns. That is the
/// case a busy band brings on — more candidates, a longer decode —
/// and it is why this is a priority rather than a tuning knob.
const AUDIO_TASK_PRIORITY: u8 = 6;

/// `uac_reader` task stack.
///
/// **Measured, finally, rather than estimated.** This constant went
/// 4 KB -> 8 KB -> 10 KB, each step a review comment saying the
/// previous looked tight (PR #98), and each step still an estimate.
/// With a radio actually streaming, `uxTaskGetStackHighWaterMark`
/// reported **36 bytes** free at the low-water point: the true usage
/// is ~10.2 KB against a 10,240 B stack.
///
/// That is what was rebooting the board 40-76 s into every capture
/// session on 2026-08-23. It never showed as a stack overflow because
/// nothing was checking — until `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK`
/// turned it into an immediate panic naming this task, instead of a
/// silent write past the end of the stack. Whether a given boot
/// survived came down to how deeply interrupts happened to be nested
/// at the next context switch, which is why it looked intermittent.
///
/// What actually lives here: `READER_BUFFER_BYTES` (4 KB) plus
/// `left_scratch` (2 KB) plus `dst_scratch` (1 KB) as stack arrays,
/// the resampler, and the 1 Hz log line — `format_args!` through the
/// fanout is far heavier on Xtensa than it looks, and it is on this
/// path once a second.
///
/// 24 KB, and it is free: this task is spawned through
/// [`spawn_psram_thread`], so its stack is PSRAM and none of this
/// competes with the internal DRAM the USB host stack needs.
/// Re-check `[stack] uac_reader hw=` in the log before trimming it.
const READER_TASK_STACK: usize = 24576;

/// Read buffer size per `uac_host_device_read` call. 4 KB = 1024
/// stereo i16 samples = ~21 ms at 48 kHz stereo — short enough that
/// disconnect detection latency stays under one FT8 symbol period
/// (160 ms), large enough that we're not paying ring-buffer overhead
/// per-sample. Sizing it to the sink's own chunk geometry would cut
/// one round of intermediate buffering; unmeasured, and the geometry
/// it would have to match is per-sink now.
const READER_BUFFER_BYTES: usize = 4096;

/// Read call timeout — short enough that a disconnect surfaces quickly
/// (`ESP_ERR_TIMEOUT` is routine ringbuf-empty and continues; every
/// other error ends the session and routes to the re-open gate),
/// long enough that
/// the loop doesn't poll-spin when the IDF ringbuf is briefly empty.
/// 100 ms ≈ half an FT8 symbol period; matches the audio_player
/// reference example's default.
const READER_READ_TIMEOUT_MS: u32 = 100;

/// IC-705 USB Audio stream config. Fixed by the IC-705 firmware;
/// the device descriptor reports a single supported alt-setting at
/// 48 kHz stereo 16-bit. Embedded 16 kHz path mentioned in early
/// design notes does not exist on this radio.
const STREAM_CHANNELS: u8 = 2;
const STREAM_BIT_RESOLUTION: u8 = 16;
const STREAM_SAMPLE_FREQ_HZ: u32 = 48_000;

/// Class-driver-side ringbuf the IDF code copies iso IN packets into
/// before `uac_host_device_read` drains them. Sized for ~85 ms of
/// audio (48 k × stereo × 2 B × 0.085 ≈ 16 KB) — plenty of slack for
/// us to lag a render frame without losing packets.
const STREAM_BUFFER_BYTES: u32 = 16 * 1024;

/// Threshold the IDF driver uses to decide when to fire `RX_DONE`
/// callbacks. Half the buffer is the canonical setting from the
/// audio_player reference. We don't currently consume the callback
/// (the reader polls), so this only affects how the IDF schedules
/// internal copies; tuning it doesn't change our latency budget.
const STREAM_BUFFER_THRESHOLD: u32 = STREAM_BUFFER_BYTES / 2;

/// Hot-plug event reified for cross-task delivery. Driver events are
/// translated by `driver_event_cb` (which runs in the class-driver
/// background task context and can't block) into one of these and
/// pushed onto the channel that `app_task` reads.
#[derive(Debug, Clone, Copy)]
enum DriverEvent {
    /// A streaming-IN interface enumerated on `addr.iface_num`.
    /// We open + start the first RxConnected we see; subsequent ones
    /// (e.g. multi-channel devices) are logged but ignored.
    RxConnected { addr: u8, iface_num: u8 },
    /// A streaming-OUT interface enumerated. IC-705 exposes one for
    /// CW/mod injection; we don't use it for FT8 RX.
    TxConnected { addr: u8, iface_num: u8 },
}

/// Sender half of the driver→app channel. Populated by `start_host`
/// before `uac_host_install` registers the callback. `OnceLock`
/// (not `OnceCell`) so the C callback context can safely read it.
static EVENT_SENDER: OnceLock<Sender<DriverEvent>> = OnceLock::new();

/// Stats counters maintained by the reader thread. Read once per
/// second by the same thread for the UDP log line. Atomics
/// (`Relaxed`) so a future inspector (e.g. LCD overlay) can sample
/// them lock-free.
/// What the USB side is doing, for the screen.
///
/// A receiver whose only feedback is a log line over WiFi is not a
/// receiver you can use: plugging the radio in has to show something,
/// now, on the device itself. This is the state the display renders,
/// updated at every transition and once a second while streaming.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UacState {
    /// Host stack not installed (wrong boot mode, or install failed).
    Off,
    /// Installed, nothing attached — the state a bare board sits in.
    Waiting,
    /// A device enumerated and its audio interface was opened.
    Streaming,
    /// Attached but the stream could not be started, or the reader
    /// died. Distinct from `Waiting` because the fix is different.
    Error,
}

static UAC_STATE: AtomicU32 = AtomicU32::new(0);
/// Post-resample 12 kHz samples in the last tick — the rate check.
static UAC_SA_PER_S: AtomicU32 = AtomicU32::new(0);
/// Signal level in the last tick, as −dBFS × 10 (so 324 = −32.4 dBFS).
/// `u32::MAX` means "no samples yet".
static UAC_RMS_MDB: AtomicU32 = AtomicU32::new(u32::MAX);

pub(crate) fn set_state(st: UacState) {
    UAC_STATE.store(st as u32, Ordering::Release);
}

/// The USB side's current state and its last-second signal figures.
pub fn status() -> (UacState, u32, Option<f32>) {
    let st = match UAC_STATE.load(Ordering::Acquire) {
        1 => UacState::Waiting,
        2 => UacState::Streaming,
        3 => UacState::Error,
        _ => UacState::Off,
    };
    let rms = UAC_RMS_MDB.load(Ordering::Acquire);
    let rms = (rms != u32::MAX).then(|| -(rms as f32) / 10.0);
    (st, UAC_SA_PER_S.load(Ordering::Acquire), rms)
}

static RX_BYTES: AtomicU32 = AtomicU32::new(0);
static RX_PACKETS: AtomicU32 = AtomicU32::new(0);
static RX_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Gate against spawning multiple readers when the IDF driver fires
/// `RxConnected` more than once for the same physical attach (e.g.
/// IC-705 advertises both an RX and TX interface and the driver may
/// reissue events on alt-setting changes). Set by `app_task` via
/// `compare_exchange` before spawning the reader; released either
/// (a) explicitly on `handle_rx_connected` failure or
/// (b) automatically via [`ReaderActiveGuard`] when the reader thread
/// exits (normal exit, error exit, or panic).
static READER_ACTIVE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// How long the reader tolerates a silent device before declaring the
/// session dead.
///
/// `uac_host_device_read` returning `ESP_ERR_TIMEOUT` is routine — the
/// driver's ring buffer is momentarily empty. What is not routine is
/// every read timing out forever, which is what a
/// `USB_TRANSFER_STATUS_OVERFLOW` leaves behind: the isochronous
/// stream stops, the device stays enumerated, and no error is ever
/// returned to us. Measured 2026-08-23 — 100 s of clean capture, one
/// overflow, then silence until the cable was pulled two minutes
/// later. Three seconds is ~150 missed frames; nothing healthy is
/// quiet that long at 48 kHz.
const STALL_TIMEOUT_MS: u64 = 3_000;

/// Pause between a failed session and the re-open attempt.
const REOPEN_DELAY_MS: u64 = 500;

/// Re-opens allowed without a second of successful streaming in
/// between. Generous, because the case worth surviving is a device
/// that works for minutes and hiccups; the cap only exists to stop a
/// wedged device from spinning open/fail forever.
const REOPEN_MAX_ATTEMPTS: u32 = 30;

/// Consecutive re-opens since audio last flowed. Reset by the 1 Hz
/// tick whenever bytes actually moved.
static REOPEN_ATTEMPTS: AtomicU32 = AtomicU32::new(0);

/// RAII guard that releases [`READER_ACTIVE`] on drop. Held by
/// `reader_thread` for its entire lifetime so the gate gets reset
/// even on panic — without the guard a panic in the read /
/// resample / push chain would leave the gate stuck `true` and
/// every subsequent `RxConnected` would be ignored until reboot
/// (Gemini PR #98 r4 review).
struct ReaderActiveGuard {
    /// `Some((addr, iface))` when the session ended in a way a fresh
    /// one might survive — a stall or a terminal read error, as
    /// opposed to the device being unplugged. `None` on a disconnect:
    /// the driver fires `RxConnected` by itself when it comes back.
    reopen: Option<(u8, u8)>,
}

impl Drop for ReaderActiveGuard {
    fn drop(&mut self) {
        // Release the gate *first*. `app_task` dedups `RxConnected`
        // against it, so a re-open posted while it is still held gets
        // dropped as a duplicate and the radio never comes back.
        READER_ACTIVE.store(false, std::sync::atomic::Ordering::Release);

        let Some((addr, iface_num)) = self.reopen else {
            return;
        };

        let attempt = REOPEN_ATTEMPTS.fetch_add(1, Ordering::AcqRel) + 1;
        if attempt > REOPEN_MAX_ATTEMPTS {
            log::error!(
                "uac: {attempt} re-opens with no audio in between — giving up until the device \
                 is re-attached"
            );
            set_state(UacState::Error);
            return;
        }

        // Let the driver settle before asking for the interface again;
        // the stop/close in the cleanup above has to land first.
        std::thread::sleep(std::time::Duration::from_millis(REOPEN_DELAY_MS));

        match EVENT_SENDER.get() {
            Some(tx) => {
                log::warn!(
                    "uac: re-opening addr={addr} iface={iface_num} (attempt {attempt}/{REOPEN_MAX_ATTEMPTS})"
                );
                if tx
                    .send(DriverEvent::RxConnected { addr, iface_num })
                    .is_err()
                {
                    log::error!("uac: re-open send failed — app_task is gone");
                }
            }
            None => log::error!("uac: re-open impossible — EVENT_SENDER not initialised"),
        }
    }
}

/// Set by [`device_event_cb`] when the IDF driver fires
/// `DRIVER_EVENT_DISCONNECTED` (USB cable unplug, IC-705 power off,
/// VBUS sag). The reader thread polls this at the top of every loop
/// iteration and exits cleanly — disconnect latency = at most one
/// `READER_READ_TIMEOUT_MS` instead of waiting for the next
/// `device_read` to fail. Also lets the reader's cleanup path skip
/// `device_stop` / `device_close` (the IDF driver already
/// invalidated the handle when DISCONNECTED fired) so the post-
/// disconnect cleanup doesn't log spurious `INVALID_ARG` errors.
///
/// Reset by [`handle_rx_connected`] at the start of a new session,
/// before `uac_host_device_open` registers the device callback, so
/// a stale `true` from a previous attach can't kill the freshly-
/// spawned reader on its first iteration and a fresh DISCONNECT
/// firing during open isn't silently dropped by the reset.
static READER_STOP_REQUESTED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Receives freshly-resampled 12 kHz mono audio from `reader_thread`,
/// one `uac_host_device_read` batch's worth at a time (length varies —
/// not chunked to any fixed size; implementations that need fixed-size
/// chunks buffer internally, same as the pre-abstraction reader body
/// did inline). `uac`'s own host/driver/hot-plug machinery is
/// otherwise consumer-agnostic; this is the one point where FT8's
/// chunk-queue pipeline ([`Ft8ChunkSink`]) and WSPR's DDC push
/// (`wspr_app`'s own sink, in `src/bin/wspr_app.rs`) diverge.
///
/// Added 2026-08-15 wiring real UAC audio into `wspr_app` — before
/// this, `reader_thread` pushed straight into an FT8-specific
/// `QueueHandle_t` (`CHUNK_Q_ADDR`/`set_chunk_q`), which is exactly
/// the coupling this trait removes.
pub trait AudioSink: Send + 'static {
    fn push_samples(&mut self, samples_12k_mono: &[i16]);
}

/// Registered sink slot. `None` until [`set_audio_sink`] runs — the
/// reader thread just drops samples until then, same "wired late,
/// drop until ready" contract the old `CHUNK_Q_ADDR` had.
static AUDIO_SINK: Mutex<Option<Box<dyn AudioSink>>> = Mutex::new(None);

// ── Cold-acquisition capture ring (#356b) ────────────────────────────
//
// `decode_pipeline` arms this when the FT8 grid is lost past what the
// ±1 s coarse search can recover (no clock, no decodes, no
// `bootstrap_dt_med` for a run of slots). While armed, `Ft8ChunkSink`
// appends raw 12 kHz audio here; once it holds
// `ft8::acquire::REQUIRED_SAMPLES`, `decode_pipeline` runs the tiled
// acquisition on it and disarms.

/// `ft8::acquire::REQUIRED_SAMPLES` (25 s) plus one chunk of slack, so
/// the last `extend_from_slice` never undershoots. ~600 KB on PSRAM
/// while armed, freed the moment `decode_pipeline` takes it.
/// Audio a cold acquisition captures, in samples at 12 kHz.
///
/// **`REQUIRED_SAMPLES`, i.e. 25 s, and it cannot simply be grown.**
/// The trials that follow `acquire_slot_phases` cut a whole slot
/// starting at the candidate phase, so an offset past 10 s runs off
/// the end — a third of the phase space has no slot behind it
/// (2026-09-20). Two slots would remove the limit and was tried: the
/// ring goes to 720 KB, `take_acquisition_audio` hands that Vec out
/// while `arm_acquisition` reserves another, and the board died of
/// `rust_oom` in `stage1_inc` on the next slot's spectrogram.
///
/// The fix came from the trial's own window instead, which is where
/// the room was: those trials now cut at the nearest offset a slot
/// does fit behind, never more than 2.5 s away and so always inside
/// `decode_block_tuned`'s own search. See the clamp in
/// `decode_pipeline`'s trial loop. **This constant is load-bearing
/// for that argument** — the clamp's 2.5 s bound is
/// `(SLOT − (CAPTURE − SLOT)) / 2`, so shortening the capture widens
/// it past the search and the unreachable phases go back to
/// undecodable.
pub const ACQUIRE_CAPTURE_SAMPLES: usize = mfsk_core::ft8::acquire::REQUIRED_SAMPLES;
const ACQUIRE_RING_CAP: usize = ACQUIRE_CAPTURE_SAMPLES + CHUNK_LEN;


static ACQUIRE_ARMED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static ACQUIRE_RING: Mutex<Vec<i16>> = Mutex::new(Vec::new());

/// Samples into its slot the capture's first sample was, or
/// `usize::MAX` before a capture has started. Diagnostic (2026-09-18):
/// the ring starts filling at whatever point `arm_acquisition` is
/// called, and the phase acquisition returns is measured from the
/// capture's start, not from a slot boundary. Recorded as an atomic in
/// the audio path — no log call there, whose stack has overflowed
/// before — and printed by `decode_pipeline` beside the phase.
static ACQUIRE_START_IN_SLOT: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(usize::MAX);

/// See [`ACQUIRE_START_IN_SLOT`]. `None` until a capture has started.
pub fn acquisition_start_in_slot() -> Option<usize> {
    match ACQUIRE_START_IN_SLOT.load(Ordering::Acquire) {
        usize::MAX => None,
        n => Some(n),
    }
}

/// Start filling the acquisition ring from scratch.
pub fn arm_acquisition() {
    // **Already armed is left alone.** The capture is 25 s of audio; a
    // second arm that clears the ring throws away what has been
    // gathered and starts the wait again. That matters now that the
    // pipeline arms at boot — the `Acquire` action a slot later must
    // not undo it.
    if ACQUIRE_ARMED.load(Ordering::Acquire) {
        return;
    }
    if let Ok(mut r) = ACQUIRE_RING.lock() {
        r.clear();
        r.reserve(ACQUIRE_RING_CAP);
    }
    ACQUIRE_ARMED.store(true, Ordering::Release);
}

/// Whether a persisted grid fix was handed over by
/// [`seed_grid_fix_us`] — i.e. whether this boot has a phase from
/// anywhere other than the air.
pub fn grid_fix_seeded() -> bool {
    PENDING_GRID_FIX_US.load(Ordering::Acquire) != i32::MIN
}

/// Stop filling and drop the buffer.
pub fn disarm_acquisition() {
    ACQUIRE_ARMED.store(false, Ordering::Release);
    if let Ok(mut r) = ACQUIRE_RING.lock() {
        *r = Vec::new();
    }
}

/// Take the captured audio once the ring holds at least `min_samples`,
/// leaving the ring empty; `None` while it is still filling. Also
/// disarms — one acquisition per arm.
/// How much of the acquisition capture is in hand, while one is armed:
/// `(have, want)` in 12 kHz samples. `None` when nothing is armed.
pub fn acquisition_fill(want: usize) -> Option<(usize, usize)> {
    if !ACQUIRE_ARMED.load(Ordering::Acquire) {
        return None;
    }
    ACQUIRE_RING.lock().ok().map(|r| (r.len().min(want), want))
}

pub fn take_acquisition_audio(min_samples: usize) -> Option<Vec<i16>> {
    let mut r = ACQUIRE_RING.lock().ok()?;
    if r.len() < min_samples {
        return None;
    }
    ACQUIRE_ARMED.store(false, Ordering::Release);
    Some(core::mem::take(&mut r))
}

// ── `MFSK_CORES3_SIM` — a radio, faked ─────────────────────────────
//
// Feeds a baked FT8 slot through the *real* `Ft8ChunkSink` at
// real-time pace, so every alignment state machine (UTC anchor,
// air-sync, cold acquisition, the capture ring, the NVS grid fix)
// runs exactly as it would with an IC-705 — but on a board that,
// flashed over USB, stays a peripheral and keeps its console. The
// only things this cannot show are real band density and a real
// off-air clock spread.

/// Feed `wav` (a 44-byte-header 12 kHz mono PCM slot) into the
/// registered [`AudioSink`] forever, preceded by `lead_silence`
/// samples so the sink's slot grid starts `lead_silence / 12` ms
/// mis-aligned from the signal — the condition the grid code exists
/// to recover from.
pub fn spawn_sim_feed(wav: &'static [u8], lead_silence: usize) {
    struct Cfg {
        wav: &'static [u8],
        lead: usize,
    }
    let cfg = Box::into_raw(Box::new(Cfg { wav, lead: lead_silence })) as *mut core::ffi::c_void;

    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `spawn_sim_feed` leaked exactly this box. Drop it once
        // its two fields are copied into locals — the task never
        // returns, so nothing else needs it.
        let (wav, lead) = {
            let cfg = unsafe { Box::from_raw(arg as *mut Cfg) };
            (cfg.wav, cfg.lead)
        };
        let pcm: Vec<i16> = wav[44..]
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect();
        // Loop a *whole number of slots*, not the raw file length.
        // `qso3_busy.wav` is 180 101 samples — 101 past one 15 s slot —
        // so wrapping at `pcm.len()` slid the content 101 samples
        // (8.4 ms) under the slot grid every loop, and spliced a
        // discontinuity into whichever frame straddled the seam. That
        // is a harness artifact, and it is not a small one: with the
        // grid held perfectly still (zero shifts applied) the measured
        // median DT still crept +8.2 ms/slot, against 8.42 ms/slot
        // predicted from the 101 samples — the receiver was being
        // blamed for the test rig's own drift, and `dec` wobbled 4-7 as
        // the window crossed the seam. Truncating to whole slots makes
        // the loop phase-continuous, so identical audio really does
        // reach the decoder identically every slot.
        let loop_len = if pcm.len() >= SLOT_SAMPLES_12K {
            pcm.len() / SLOT_SAMPLES_12K * SLOT_SAMPLES_12K
        } else {
            pcm.len()
        };
        log::warn!(
            "uac SIM: feeding {loop_len} of {} baked samples on loop ({} slot(s), {} trimmed for phase continuity), {} ms lead silence — no radio",
            pcm.len(),
            loop_len / SLOT_SAMPLES_12K.max(1),
            pcm.len() - loop_len,
            lead / 12
        );
        const BLK: usize = 256;
        let t0 = unsafe { sys::esp_timer_get_time() };
        let mut fed: u64 = 0;
        let mut src = 0usize; // index into pcm, after the lead is done
        let mut lead_left = lead;
        let silence = [0i16; BLK];
        loop {
            let block: &[i16] = if lead_left >= BLK {
                lead_left -= BLK;
                &silence
            } else if lead_left > 0 {
                let n = lead_left;
                lead_left = 0;
                &silence[..n]
            } else {
                let end = (src + BLK).min(loop_len);
                let s = &pcm[src..end];
                src = if end == loop_len { 0 } else { end };
                s
            };
            if let Ok(mut g) = AUDIO_SINK.lock() {
                if let Some(sink) = g.as_mut() {
                    sink.push_samples(block);
                }
            }
            fed += block.len() as u64;
            let due = (fed * 1_000_000 / 12_000) as i64;
            let now = unsafe { sys::esp_timer_get_time() } - t0;
            if due > now {
                unsafe {
                    sys::vTaskDelay(
                        (((due - now) / 1_000).max(1) as u32)
                            / (1_000 / sys::configTICK_RATE_HZ).max(1),
                    )
                };
            }
        }
    }

    let r = unsafe {
        sys::xTaskCreatePinnedToCore(
            Some(entry),
            c"uac_sim".as_ptr(),
            4096,
            cfg,
            AUDIO_TASK_PRIORITY as u32,
            core::ptr::null_mut(),
            0,
        )
    };
    if r != 1 {
        log::error!("uac SIM: feed task spawn failed");
    }
}

/// NVS partition handle for persisting the cold-acquisition grid fix
/// across the reboot into FT4 mode (#356b). Set from `main` before the
/// decode pipeline spawns.
static GRID_FIX_NVS: Mutex<Option<esp_idf_svc::nvs::EspDefaultNvsPartition>> = Mutex::new(None);

/// Hand the decode pipeline a way to persist the acquired grid phase.
pub fn set_grid_fix_nvs(part: esp_idf_svc::nvs::EspDefaultNvsPartition) {
    if let Ok(mut slot) = GRID_FIX_NVS.lock() {
        *slot = Some(part);
    }
}

/// Persist a cold-acquisition grid fix, from a short-lived
/// internal-stack task — an NVS write disables the flash cache, which
/// a PSRAM stack (or the decode task mid-flight) must not be holding.
/// One-shot, fire and forget.
pub fn persist_grid_fix(fix: mfsk_app_shared::grid_fix::GridFix) {
    let Some(part) = GRID_FIX_NVS.lock().ok().and_then(|g| g.clone()) else {
        log::warn!("uac: grid-fix NVS not wired — acquired phase will not survive a reboot");
        return;
    };

    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `persist_grid_fix` leaked exactly this box.
        let boxed = unsafe {
            Box::from_raw(
                arg as *mut (
                    esp_idf_svc::nvs::EspDefaultNvsPartition,
                    mfsk_app_shared::grid_fix::GridFix,
                ),
            )
        };
        let (part, fix) = *boxed;
        match mfsk_app_shared::boot_mode::open_nvs(part) {
            Ok(mut nvs) => match mfsk_app_shared::grid_fix::save(&mut nvs, &fix) {
                Ok(()) => log::warn!(
                    "grid-fix persisted: {:+} us (R {:.2}) — will seed FT4's grid on next boot",
                    fix.offset_us,
                    fix.confidence
                ),
                Err(e) => log::error!("grid-fix save failed: {e}"),
            },
            Err(e) => log::error!("grid-fix NVS open failed: {e}"),
        }
        unsafe { sys::vTaskDelete(core::ptr::null_mut()) };
    }

    let ptr = Box::into_raw(Box::new((part, fix))) as *mut core::ffi::c_void;
    let created = unsafe {
        sys::xTaskCreatePinnedToCore(
            Some(entry),
            c"grid_fix_save".as_ptr(),
            4096,
            ptr,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("uac: could not spawn grid-fix save task");
        // Reclaim the leaked box so it does not just leak.
        drop(unsafe {
            Box::from_raw(
                ptr as *mut (
                    esp_idf_svc::nvs::EspDefaultNvsPartition,
                    mfsk_app_shared::grid_fix::GridFix,
                ),
            )
        });
    }
}

/// Register the audio sink. Call before [`start_host`] so the sink is
/// live before the class driver can enumerate a device and spawn
/// `reader_thread` — same ordering `main.rs` already relies on for
/// [`set_chunk_q`] (spawn the pipeline / register the sink first,
/// install the UAC driver second).
pub fn set_audio_sink<S: AudioSink>(sink: S) {
    match AUDIO_SINK.lock() {
        Ok(mut slot) => {
            *slot = Some(Box::new(sink));
            log::info!("uac: audio sink registered");
        }
        Err(e) => log::error!("uac: AUDIO_SINK mutex poisoned, sink not registered: {e}"),
    }
}

/// FT8 chunk-queue sink — the pre-abstraction `reader_thread` behavior
/// verbatim: chunks resampled 12 kHz mono audio to [`CHUNK_LEN`]
/// (100 ms) blocks and emits `ChunkMsg::SlotEnd` every
/// [`SLOT_SAMPLES_12K`] (15 s, one FT8 slot), publishing the new slot
/// index via `time_sync::publish_capture_slot`.
struct Ft8ChunkSink {
    chunk_q: sys::QueueHandle_t,
    chunk: Vec<i16>,
    slot_samples: usize,
    /// Samples this slot runs before `SlotEnd` fires. Normally
    /// [`SLOT_SAMPLES_12K`]; the air-sync shift (#356) moves it by up to
    /// one slot's worth while the clock is not NTP-disciplined.
    slot_target: usize,
    wav_idx: usize,
    /// The one-time rough anchor from the system clock (RTC *or* NTP)
    /// has run. It gets the grid inside FT8's ±2.5 s and, more to the
    /// point, inside the ±1 s coarse search — so the air-sync
    /// refinement below has candidates to measure a DT from. Distinct
    /// from "the clock is trusted": a plausible RTC value anchors here,
    /// only NTP hands the phase over to the UTC drift check.
    coarse_anchored: bool,
    /// One line has been logged saying NTP disciplined the clock and
    /// air-sync stood down — not one per slot.
    utc_owns_phase_logged: bool,
}
// SAFETY: `QueueHandle_t` is a raw pointer into IDF-owned state; the
// IDF queue API is thread-safe by design (that's the whole point of a
// FreeRTOS queue), and `Ft8ChunkSink` never dereferences the pointer
/// **How long the reader spent blocked handing chunks on.** Max and
/// total per 1 Hz tick, in µs, reset by the tick that prints them.
///
/// The question these answer cannot be answered from the outside: a
/// second with fewer bytes in it says audio was lost, not where. The
/// reader drains a 16 KB ring (85 ms, [`STREAM_BUFFER_BYTES`]) and
/// blocks on a four-chunk queue (400 ms), so "the reader was held
/// longer than the ring" and "the driver was not scheduled" produce
/// the same missing bytes and want opposite fixes.
///
/// Two relaxed atomics per chunk, no allocation, no lock, nothing on
/// the stack — the constraints `embedded-poc/CLAUDE.md` sets for a
/// probe on this board.
static SINK_BLOCK_MAX_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static SINK_BLOCK_SUM_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// **Longest gap between two reads, per tick, in µs.** The question
/// [`SINK_BLOCK_MAX_US`] cannot answer.
///
/// A deficit second is never followed by a surplus one — measured on a
/// radio 2026-09-19, 34-38 reads of 4 096 B where every other second
/// does exactly 47 — so the samples are discarded rather than queued,
/// and the ring is only [`STREAM_BUFFER_BYTES`] = 85 ms deep. Anything
/// in the path that stops for longer than that loses audio outright,
/// and "the reader was not scheduled" and "the class driver did not
/// resubmit transfers" look identical from the byte count while
/// wanting opposite fixes. A read normally completes every 21.3 ms
/// (4 096 B at 192 000 B/s), so this separates them on its own: a
/// figure near 21 ms exonerates this thread.
static READ_GAP_MAX_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Reads that came back `ESP_ERR_TIMEOUT` — the ring stayed empty for
/// the whole [`READER_READ_TIMEOUT_MS`]. Counted because the loop
/// otherwise `continue`s past it in silence, and it is the driver side
/// of the same question.
static READ_TIMEOUTS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Hand a chunk on, timing how long that took.
fn send_chunk_timed(q: sys::QueueHandle_t, msg: Box<ChunkMsg>) {
    let t0 = unsafe { sys::esp_timer_get_time() };
    send_box(q, msg);
    let dt = (unsafe { sys::esp_timer_get_time() } - t0).clamp(0, u32::MAX as i64) as u32;
    SINK_BLOCK_SUM_US.fetch_add(dt, Ordering::Relaxed);
    SINK_BLOCK_MAX_US.fetch_max(dt, Ordering::Relaxed);
}

// itself — every use goes through `pipeline::send_box`, which wraps
// the IDF `xQueueGenericSend`. Matches `DeviceHandle`'s own identical
// `Send` rationale above.
unsafe impl Send for Ft8ChunkSink {}

impl Ft8ChunkSink {
    fn new(chunk_q: sys::QueueHandle_t) -> Self {
        Self {
            chunk_q,
            chunk: Vec::with_capacity(CHUNK_LEN),
            slot_samples: 0,
            slot_target: SLOT_SAMPLES_12K,
            coarse_anchored: false,
            utc_owns_phase_logged: false,
            wav_idx: 0,
        }
    }
}

impl AudioSink for Ft8ChunkSink {
    fn push_samples(&mut self, samples: &[i16]) {
        // Feed the cold-acquisition ring while `decode_pipeline` has it
        // armed (#356b). Bounded — stop appending once it is full and
        // let the pipeline pick it up.
        if ACQUIRE_ARMED.load(Ordering::Acquire) {
            if let Ok(mut r) = ACQUIRE_RING.lock() {
                if r.is_empty() {
                    ACQUIRE_START_IN_SLOT
                        .store(self.slot_samples + self.chunk.len(), Ordering::Release);
                }
                if r.len() < ACQUIRE_RING_CAP {
                    let room = ACQUIRE_RING_CAP - r.len();
                    r.extend_from_slice(&samples[..samples.len().min(room)]);
                }
            }
        }

        // One-time rough anchor from the system clock, RTC or NTP.
        //
        // Without any anchor the boundary is whatever 15 s window the
        // reader started in, and FT8's coarse sync only searches ±2.5 s
        // of it — so a real signal is outside the window about 2/3 of
        // the time and the receiver looks broken for a reason that has
        // nothing to do with the audio (#313/#163). Costs no samples:
        // the *current* partial slot is simply given the right length,
        // so the next boundary lands on the grid.
        //
        // A merely-plausible RTC value is enough here — the point is to
        // get inside the ±1 s coarse search so the air-sync refinement
        // (#356) has a DT to work with. Only NTP hands the phase to the
        // UTC drift check below.
        // **`AIR DT` takes the anchor too.** It used not to, on the
        // reading that an RTC-guessed phase only got in the air's way:
        // "measured 0.68 s and 1.65 s out on 2026-09-19, each time
        // costing minutes before the capture put it right".
        //
        // Those measurements were taken while the audio path was losing
        // 6.5 % of its samples (the isochronous URB budget, aa10bf2e).
        // The RTC was not 1.65 s wrong; the grid was *drifting* that far
        // after the anchor placed it correctly, at +1035 ms a slot. With
        // the loss gone the same anchor holds to −5.8 ms a slot, and
        // skipping it costs exactly what it was supposed to save:
        // measured on a radio 2026-09-19, an `AIR DT` start with a
        // good RTC spent **2 min 9 s** and two 25 s captures — the
        // first of which decoded nothing at all — rediscovering a phase
        // the clock already had.
        //
        // The air still owns the phase: this is a one-shot placement,
        // `clock_is_disciplined()` stays false without NTP, and the
        // acquisition path is untouched. If the RTC really is seconds
        // out — the hilltop case this mode exists for — the anchor
        // places the grid wrongly, nothing decodes, and the under-par
        // run reaches acquisition exactly as before. There is no case
        // where starting from the clock is worse than starting from
        // nothing.
        if !self.coarse_anchored {
            if let Some(remain) = mfsk_app_shared::time_sync::samples_to_next_slot_12k(SLOT_SECS) {
                // The clock puts the boundary within a second; a
                // persisted air fix puts it within milliseconds. Same
                // fold and the same sign as `apps/ft4.rs` does for its
                // own grid — µs to 12 kHz samples, modulo the period.
                let fix_us = PENDING_GRID_FIX_US.load(Ordering::Acquire);
                let remain = if fix_us == i32::MIN {
                    remain
                } else {
                    let shifted = remain as i64 + (fix_us as i64 * 12 / 1000);
                    let r = shifted.rem_euclid(SLOT_SAMPLES_12K as i64) as usize;
                    log::info!(
                        "uac: folding a persisted air fix into the anchor — {:+} ms                          ({} ms to the boundary, was {})",
                        fix_us / 1000,
                        r / 12,
                        remain / 12,
                    );
                    r
                };
                // **Shorten this slot; do not pre-load its counter.**
                //
                // Both ways move the boundary to `remain` samples from
                // now. Only one of them tells the truth downstream:
                // `slot_samples` is "how much of this slot has been
                // pushed", and stage1_inc fills its buffer from what it
                // actually receives, so a pre-loaded counter makes
                // `SlotEnd` report a full slot while the buffer holds
                // `remain` samples. `finalize_slot` then hands the
                // decoder a truncated slot labelled as a whole one —
                // the last second missing, every station at positive DT
                // losing its tail. Seen on the radio as
                // `audio_fill=168000 != reported total 180756` and, in
                // NTP mode where the drift re-anchor fires often, as
                // every other slot decoding nothing (2026-09-19).
                self.slot_target = remain;
                self.coarse_anchored = true;
                // Grid lock state (#356b): a plausible clock, disciplined
                // or not. `decode_pipeline`'s air-sync raises this to
                // `Air` once it locks; only NTP raises it to `Ntp`.
                mfsk_app_shared::time_sync::note_grid_lock(
                    if mfsk_app_shared::time_sync::clock_is_disciplined() {
                        mfsk_app_shared::time_sync::GridLock::Ntp
                    } else {
                        mfsk_app_shared::time_sync::GridLock::Rtc
                    },
                );
                log::info!(
                    "uac: slot grid coarse-anchored to the system clock — {} ms to the next \
                     boundary (clock sub-second {} ms, slot_samples now {})",
                    remain / 12,
                    mfsk_app_shared::time_sync::utc_now_ms().map_or(-1, |ms| (ms % 1000) as i64),
                    self.slot_samples,
                );
            }
        }
        for &s in samples {
            self.chunk.push(s);
            if self.chunk.len() >= CHUNK_LEN {
                let to_send = core::mem::replace(&mut self.chunk, Vec::with_capacity(CHUNK_LEN));
                send_chunk_timed(self.chunk_q, Box::new(ChunkMsg::Samples(to_send)));
                self.slot_samples += CHUNK_LEN;
                if self.slot_samples >= self.slot_target {
                    send_chunk_timed(
                        self.chunk_q,
                        Box::new(ChunkMsg::SlotEnd {
                            wav_idx: self.wav_idx,
                            total_samples: self.slot_samples,
                        }),
                    );
                    self.wav_idx = self.wav_idx.wrapping_add(1);
                    self.slot_samples = 0;
                    // Next slot is the nominal length unless the air-sync
                    // shift below moves it.
                    self.slot_target = SLOT_SAMPLES_12K;
                    // Issue #110: publish capture-slot boundary for
                    // any TX scheduler running in this BootMode.
                    // BootMode::Uac is currently RX-only on the S3
                    // board, but published unconditionally to keep
                    // the slot index live in case downstream
                    // logging / TX wiring is added.
                    let now_us = unsafe { sys::esp_timer_get_time() };
                    mfsk_app_shared::time_sync::publish_capture_slot(self.wav_idx as u32, now_us);

                    // **Where this boundary fell against UTC**, in ms
                    // into the 15 s grid, signed to the nearer tick.
                    //
                    // The one measurement that separates the two
                    // explanations for a slot grid that reads ~0.75 s
                    // out after the anchor and 0.04 s out once the
                    // per-slot tracker has had a turn (measured on a
                    // radio 2026-09-19, the same 0.7 s in both NTP and
                    // `AIR DT` on the first full slot):
                    //
                    //   ≈0    — the boundary is declared at the right
                    //           *time*, so the offset is in the audio
                    //           this sink is holding, not in the
                    //           arithmetic;
                    //   ≈±750 — the declaration itself is late or
                    //           early, and the bug is in `slot_target`
                    //           / `slot_samples` or in the anchor.
                    //
                    // Guessing between them was tried and the sign did
                    // not work out either way, which is why this is a
                    // measurement rather than a fix.
                    if let Some(ms) = mfsk_app_shared::time_sync::utc_now_ms() {
                        let into = (ms % (SLOT_SECS as u64 * 1000)) as i64;
                        let err = if into > SLOT_SECS as i64 * 500 {
                            into - SLOT_SECS as i64 * 1000
                        } else {
                            into
                        };
                        log::info!(
                            "uac: slot {} boundary {err:+} ms off the UTC grid (target {} samples)",
                            self.wav_idx,
                            self.slot_target,
                        );
                    }

                    // One phase authority per boundary, never both.
                    // Once NTP has disciplined the clock it is trusted
                    // absolutely; before then — or forever, off-grid —
                    // the grid rides coarse sync's own DT through
                    // `decode_pipeline`'s air-sync (#356).
                    if mfsk_app_shared::time_sync::clock_is_disciplined() {
                        mfsk_app_shared::time_sync::note_grid_lock(
                            mfsk_app_shared::time_sync::GridLock::Ntp,
                        );
                        if !self.utc_owns_phase_logged {
                            self.utc_owns_phase_logged = true;
                            log::info!(
                                "uac: NTP-disciplined — UTC owns the slot phase, air-sync stood down"
                            );
                        }
                        // Drain the air-sync hint so a later loss of NTP
                        // does not re-apply something stale.
                        let _ = mfsk_app_shared::time_sync::take_bootstrap_slot_shift_12k();
                        // Re-anchor only when the phase error is a
                        // fraction of what FT8 can search — corrects the
                        // NTP step and long-term drift, not jitter.
                        if let Some(remain) =
                            mfsk_app_shared::time_sync::samples_to_next_slot_12k(SLOT_SECS)
                        {
                            let err_ms = if remain > SLOT_SAMPLES_12K / 2 {
                                -(((SLOT_SAMPLES_12K - remain) / 12) as i32)
                            } else {
                                (remain / 12) as i32
                            };
                            // **Correct late by shortening, leave
                            // small early alone, and take one short
                            // slot for a large early.**
                            //
                            // This used to fire only past
                            // `SLOT_DRIFT_REANCHOR_MS`, and then took
                            // the whole error out of one slot. That is
                            // the worse half of the trade twice over:
                            // the audio arrives at 12 032 sa/s against
                            // a nominal 12 000 (measured on an IC-705,
                            // 2026-09-19), so the error refills and the
                            // correction is not rare; and a slot cut
                            // more than 12 000 samples short never
                            // reaches `stage1_inc::SPEC_EMIT_PAIR` at
                            // 168 000, so it emits a partial SpecBundle
                            // and the decoder gets no tail window at
                            // all. Measured: alternating slots at
                            // `pair_done=85/92`, `tail_win=0`, `dec` 0-2
                            // against 7 on the slots between them.
                            //
                            // Following it every slot keeps the phase
                            // inside one chunk of UTC, and the clamp
                            // spreads a genuine step (an NTP jump) over
                            // a few slots that all still decode instead
                            // of one that cannot.
                            let target = if remain <= DEAD_ZONE_SAMPLES
                                || remain >= SLOT_SAMPLES_12K - DEAD_ZONE_SAMPLES
                            {
                                // Already on the grid, either side of
                                // it. Leave the slot alone.
                                SLOT_SAMPLES_12K
                            } else {
                                // End this slot on the next UTC
                                // boundary. Exact, and done in one
                                // slot whichever way the error points.
                                remain
                            };
                            if err_ms.unsigned_abs() > SLOT_DRIFT_REANCHOR_MS {
                                log::warn!(
                                    "uac: slot phase {err_ms:+} ms off UTC — next slot {target} \
                                     samples"
                                );
                            }
                            self.slot_target = target;
                        }
                    } else {
                        let acq = mfsk_app_shared::time_sync::take_acquisition_shift_12k();
                        if acq != 0 {
                            // Cold-acquisition one-shot (#356b): the grid
                            // was lost past ±1 s and `decode_pipeline`
                            // recovered the phase from a 25 s FT8
                            // capture, or the DT trim wants a smaller
                            // move through the same channel.
                            //
                            // **A slot can only be shortened.** This
                            // used to be `(SLOT + acq).clamp(30_000,
                            // 300_000)`, and `acq` reaches +90 000 (dt
                            // is normalised to ±7.5 s), so a positive
                            // shift asked for a slot of up to 270 000
                            // samples. `stage1_inc::NMAX` is 180 000 and
                            // is the spectrogram's geometry, not a
                            // buffer — the excess is dropped with a
                            // warning, and the sink then reports a total
                            // the builder never held. That is the same
                            // `audio_fill != reported total` split the
                            // grid anchor had this morning.
                            //
                            // A shift of `+acq` and one of `acq - SLOT`
                            // land the boundary in the same place, so
                            // the positive case is taken by *shortening*
                            // to `acq`. Never fired on hardware — both
                            // acquisitions measured 2026-09-19 were
                            // negative — which is the whole reason to
                            // fix it now rather than after it does.
                            let acq_abs = acq.unsigned_abs() as usize;
                            self.slot_target = if acq_abs <= DEAD_ZONE_SAMPLES {
                                SLOT_SAMPLES_12K
                            } else if acq > 0 {
                                acq as usize
                            } else {
                                (SLOT_SAMPLES_12K as i32 + acq).max(MIN_ACQ_SLOT_SAMPLES) as usize
                            };
                            self.coarse_anchored = true;
                            mfsk_app_shared::time_sync::note_grid_lock(
                                mfsk_app_shared::time_sync::GridLock::Air,
                            );
                            log::info!(
                                "uac: cold-acquisition slot shift {acq:+} — next slot {} samples",
                                self.slot_target,
                            );
                        } else {
                            // Air-sync shift from `decode_pipeline`
                            // (#356): the DT of coarse sync's own
                            // candidates. Sign is WSJT-X's — DT > 0 means
                            // the slot opened early, lengthen the next.
                            // Capped ±2400 by the producer, so the slot
                            // stays in [177_600, 182_400] and stage1_inc
                            // still completes.
                            let air_shift =
                                mfsk_app_shared::time_sync::take_bootstrap_slot_shift_12k();
                            if air_shift != 0 {
                                self.slot_target = (SLOT_SAMPLES_12K as i32 + air_shift)
                                    .clamp(60_000, 200_000)
                                    as usize;
                                log::info!(
                                    "uac: air-sync slot shift {air_shift:+} — next slot {} samples",
                                    self.slot_target,
                                );
                            }
                        }
                    }

                    // **How much of this slot is still to come**, from
                    // the boundary published just above — not the
                    // slot's nominal length.
                    //
                    // The two branches above can both move the grid:
                    // the UTC coarse anchor and the drift re-anchor set
                    // `slot_samples` (this slot starts part-way in), and
                    // acquisition / air-sync set `slot_target` (it runs
                    // long or short). Publishing `slot_target` alone was
                    // right only when neither had fired, and wrong by
                    // exactly the jump when one had — which is the
                    // moment the decode task most needs the number,
                    // since the floor is about to judge the next bundle
                    // by it. Seen on a radio as `hint_err` near −1 s
                    // right after each re-anchor (2026-09-19).
                    let remaining = self.slot_target.saturating_sub(self.slot_samples);
                    mfsk_app_shared::time_sync::publish_capture_slot_len_us(
                        remaining as i64 * (SLOT_SECS as i64 * 1_000_000)
                            / SLOT_SAMPLES_12K as i64,
                    );
                }
            }
        }
    }
}

/// Wire the FT8 decode pipeline's chunk queue as the audio sink.
/// Called once from `decode_pipeline::run_with_source`'s source-spawn
/// closure in the pipeline thread, before the decode loop blocks on
/// `recv_box`. Thin wrapper around [`set_audio_sink`] kept under its
/// original name so `main.rs`/`decode_pipeline.rs` need no changes.
pub fn set_chunk_q(q: sys::QueueHandle_t) {
    set_audio_sink(Ft8ChunkSink::new(q));
    log::info!("uac: chunk_q wired (addr={:#x})", q as usize);
}

/// A persisted grid-phase fix waiting to be folded into the first
/// coarse anchor, in microseconds — `i32::MIN` when there is none.
///
/// The RTC gives the boundary to within a second; this gives the rest.
/// It is the same record `apps/ft4.rs` reads on the FT8 → FT4 reboot,
/// used here for the case it was always for and never wired to: a
/// station that acquired from the air yesterday and is switched on
/// again today, with no network to ask.
static PENDING_GRID_FIX_US: core::sync::atomic::AtomicI32 =
    core::sync::atomic::AtomicI32::new(i32::MIN);

/// Hand the sink a persisted fix, before the first audio arrives.
pub fn seed_grid_fix_us(offset_us: i32) {
    PENDING_GRID_FIX_US.store(offset_us, Ordering::Release);
}

/// `SlotEnd` cadence in 12 kHz mono samples. Same as `wav_sim`'s
/// `SLOT_SAMPLES` — 180_000 = 15 s @ 12 kHz, one FT8 slot. UAC streams
/// continuously, so the reader synthesises the boundary from the
/// post-resample sample count.
///
/// **Anchored to UTC since 2026-08-22** (#313 open item 1). The count
/// still sets the slot's *length*; `time_sync::samples_to_next_slot_12k`
/// sets its *phase*, once at first sight of a real clock and again
/// whenever the two drift more than [`SLOT_DRIFT_REANCHOR_MS`] apart.
/// Without NTP there is no phase source and the boundary falls back to
/// stream-relative — the sink says which of the two it is doing rather
/// than leaving a reader to guess.
const SLOT_SAMPLES_12K: usize = 180_000;
/// The same slot, in seconds — what the UTC grid is computed from.
const SLOT_SECS: u64 = 15;
/// Phase error that is worth a log line. The correction itself runs
/// every slot (see the tracking block); this is only the threshold for
/// saying so, so it stays at a tenth of the ±2.5 s FT8 searches.
const SLOT_DRIFT_REANCHOR_MS: u32 = 250;

/// How far off the grid may sit before a correction is worth a slot.
///
/// **Corrections are exact and cost at most one slot; they are never
/// spread.** Ending the slot on the next UTC boundary puts the grid
/// right in one step whichever way the error points, and when the
/// error is large that slot is short enough to emit a partial
/// SpecBundle and decode nothing. That is the whole price, and it is
/// the right one: spreading a 3 s error over six 0.5 s steps gives six
/// slots at a phase the band cannot be found at, where one short slot
/// gives one.
///
/// A clamped, spread version shipped for one run and taught this the
/// expensive way. `stage1_inc::NMAX` is 180 000 and it is the
/// spectrogram's geometry (`N_TIME = NMAX / NSTEP - 3`), not a buffer
/// that can be grown — so a grid running *early* cannot be pulled back
/// by running one slot long, and clamping it to a 174 000 floor
/// shortens where lengthening was wanted. The error then grew by
/// exactly one clamp step per slot: on a radio it walked +503, +1027,
/// +1494 … +7002 ms, and every one of those slots decoded nothing —
/// 26 of 123 slots in a 30-minute run, 2026-09-19.
///
/// The dead zone is what keeps the exact correction from firing on
/// jitter: once the audio rate error is gone (measured −5.8 ms per
/// slot) the phase sits near zero and crosses it, and a correction
/// that costs a slot must not trigger on that. 200 ms, a fifth of the
/// ±1.0 s the coarse search covers.
const DEAD_ZONE_SAMPLES: usize = 2_400;

/// Floor for a cold-acquisition slot, in samples. `acq` is bounded by
/// ±90 000, so the shortening branch cannot go below 90 000 on its own;
/// this is the guard for a future wider bound rather than a live one.
const MIN_ACQ_SLOT_SAMPLES: i32 = 30_000;

/// Driver event callback. Invoked by the UAC class-driver background
/// task on every `RX_CONNECTED` / `TX_CONNECTED` notification (i.e.
/// every time an audio streaming interface enumerates).
///
/// Runs in the class-driver task context — must not block / allocate
/// significantly. We forward to the app task via the mpsc channel
/// (lock-free for the single-producer case) and return.
extern "C" fn driver_event_cb(
    addr: u8,
    iface_num: u8,
    event: sys::uac::uac_host_driver_event_t,
    _arg: *mut core::ffi::c_void,
) {
    let driver_event = match event {
        sys::uac::uac_host_driver_event_t_UAC_HOST_DRIVER_EVENT_RX_CONNECTED => {
            DriverEvent::RxConnected { addr, iface_num }
        }
        sys::uac::uac_host_driver_event_t_UAC_HOST_DRIVER_EVENT_TX_CONNECTED => {
            DriverEvent::TxConnected { addr, iface_num }
        }
        other => {
            log::warn!("uac: unknown driver event addr={addr} iface={iface_num} raw={other}");
            return;
        }
    };
    DRIVER_EVENTS.fetch_add(1, Ordering::Relaxed);
    // **Latch it, because the log line may never arrive.**
    //
    // Enumeration happens inside `start_host`, which runs as soon as
    // the UDP sink object exists — and on 2026-09-20 that was 33 s
    // before WiFi finished associating, so every line between the two
    // was written and dropped. The resulting log showed no
    // `TxConnected` and no `RxConnected` while the receiver was plainly
    // decoding, which says nothing about the radio and everything
    // about the path the evidence took.
    //
    // A counter survives that. `usb_host_lib_info`'s periodic line
    // reports it, so "did the OUT interface ever appear" is answerable
    // from any later point in the log.
    match driver_event {
        DriverEvent::RxConnected { iface_num, .. } => {
            RX_IFACE_SEEN.store(iface_num as i32, Ordering::Relaxed);
        }
        DriverEvent::TxConnected { iface_num, .. } => {
            TX_IFACE_SEEN.store(iface_num as i32, Ordering::Relaxed);
        }
    }
    log::info!("uac: driver event {driver_event:?}");
    if let Some(sender) = EVENT_SENDER.get() {
        if let Err(e) = sender.send(driver_event) {
            log::error!("uac: app channel send failed (app_task gone): {e}");
        }
    } else {
        log::error!("uac: driver event before EVENT_SENDER init — dropped {driver_event:?}");
    }
}

/// Device-level event callback. Set in `uac_host_device_config_t` at
/// `uac_host_device_open` time, fires on `RX_DONE` / `TX_DONE` /
/// `TRANSFER_ERROR` / `DRIVER_EVENT_DISCONNECTED`. The reader thread
/// polls `uac_host_device_read` rather than waiting on the callback,
/// so the first three are logged and nothing else; `DISCONNECTED`
/// sets [`READER_STOP_REQUESTED`], which is how the reader learns to
/// stop and to skip the cleanup the IDF driver has already done.
extern "C" fn device_event_cb(
    _handle: sys::uac::uac_host_device_handle_t,
    event: sys::uac::uac_host_device_event_t,
    _arg: *mut core::ffi::c_void,
) {
    let kind = match event {
        sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_RX_DONE => "RX_DONE",
        sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_TX_DONE => "TX_DONE",
        sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR => "TRANSFER_ERROR",
        sys::uac::uac_host_device_event_t_UAC_HOST_DRIVER_EVENT_DISCONNECTED => "DISCONNECTED",
        other => {
            log::warn!("uac: unknown device event raw={other}");
            return;
        }
    };
    // RX_DONE is the high-frequency one (every ~10 ms once streaming);
    // logging it would saturate UDP. Suppress, log only the other
    // three which are exceptional.
    if event != sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_RX_DONE {
        log::info!("uac: device event {kind}");
    }
    // Disconnect signal: the IDF driver invalidates the handle after
    // this callback returns, so any pending `device_read` will fail.
    // Setting the flag lets the reader thread exit on its next loop
    // iteration (≤ 100 ms latency) instead of waiting for the failing
    // read to surface — and lets the reader skip the
    // `device_stop` / `device_close` cleanup since the IDF already
    // released the underlying state.
    if event == sys::uac::uac_host_device_event_t_UAC_HOST_DRIVER_EVENT_DISCONNECTED {
        READER_STOP_REQUESTED.store(true, Ordering::Release);
    }
}

/// USB host event-pump body. Blocks indefinitely on
/// `usb_host_lib_handle_events`. Returned `event_flags` are
/// intentionally ignored (see PR #92 review history for why).
fn usb_events_task() {
    const FOREVER: sys::TickType_t = sys::TickType_t::MAX;
    loop {
        let mut event_flags: u32 = 0;
        let err = unsafe { sys::usb_host_lib_handle_events(FOREVER, &mut event_flags as *mut u32) };
        if err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: usb_host_lib_handle_events err={err:#x}");
            if err == sys::ESP_ERR_INVALID_STATE as sys::esp_err_t {
                break;
            }
            esp_idf_svc::hal::delay::FreeRtos::delay_ms(50);
            continue;
        }
        let _ = event_flags;
    }
    log::error!("uac: usb_events_task exiting — host stack gone");
}

/// Convert a `(addr, iface_num)` `RxConnected` into an open + started
/// UAC device + reader thread. Returns the device handle on success
/// The handle moves into the reader thread; a disconnect reaches it
/// through [`READER_STOP_REQUESTED`], not through the handle.
fn handle_rx_connected(addr: u8, iface_num: u8) -> Result<()> {
    log::info!("uac: opening device addr={addr} iface={iface_num}");
    // Clear any stale stop-request BEFORE `uac_host_device_open` —
    // that call registers `device_event_cb` for the new handle, after
    // which a fast DISCONNECT (e.g. cable yanked mid-bringup) would
    // race against the reset and have its `true` silently dropped.
    // Resetting now is safe because (1) `app_task` processes
    // `DriverEvent`s sequentially on the mpsc channel, so the previous
    // session's DISCONNECTED callback finished firing before this
    // RxConnected was dispatched, and (2) the `READER_ACTIVE` gate
    // guarantees the previous `reader_thread` has fully exited before
    // we re-enter `handle_rx_connected` (Gemini PR #107 review —
    // the older "callback unregistered at device_close" rationale was
    // incorrect since the disconnect cleanup path explicitly skips
    // `device_close`).
    READER_STOP_REQUESTED.store(false, Ordering::Release);
    let dev_config = sys::uac::uac_host_device_config_t {
        addr,
        iface_num,
        buffer_size: STREAM_BUFFER_BYTES,
        buffer_threshold: STREAM_BUFFER_THRESHOLD,
        callback: Some(device_event_cb),
        callback_arg: core::ptr::null_mut(),
    };
    let mut handle: sys::uac::uac_host_device_handle_t = core::ptr::null_mut();
    let err = unsafe {
        sys::uac::uac_host_device_open(
            &dev_config as *const _,
            &mut handle as *mut sys::uac::uac_host_device_handle_t,
        )
    };
    if err != sys::ESP_OK as sys::esp_err_t {
        return Err(anyhow!(
            "uac_host_device_open(addr={addr}, iface={iface_num}) failed err={err:#x}"
        ));
    }
    log::info!("uac: device opened, starting stream {STREAM_CHANNELS}ch / {STREAM_BIT_RESOLUTION}b / {STREAM_SAMPLE_FREQ_HZ}Hz");

    let stream_config = sys::uac::uac_host_stream_config_t {
        channels: STREAM_CHANNELS,
        bit_resolution: STREAM_BIT_RESOLUTION,
        sample_freq: STREAM_SAMPLE_FREQ_HZ,
        flags: 0,
    };
    let err = unsafe { sys::uac::uac_host_device_start(handle, &stream_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        // Best-effort close; if it fails we can't do much beyond logging.
        let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: device_close after device_start failure also failed err={close_err:#x}"
            );
        }
        return Err(anyhow!(
            "uac_host_device_start failed err={err:#x} (config 48k/stereo/16b — IC-705 should support this; check the device descriptor in UDP log)"
        ));
    }

    // Reader spawn failure rollback: device_open + device_start
    // succeeded, so the handle owns USB resources; bail without
    // releasing them would mean the IDF driver thinks the device is
    // streaming forever (next RxConnected would race against a stuck
    // alt-setting). Stop + close before bubbling the error.
    // Reset stats for the new session so the 1 Hz throughput log
    // reflects the current device, not accumulated bytes from a
    // previous attach (Gemini PR #98 r3 review). `Relaxed` since
    // no concurrent reader exists at this point — the new reader
    // is about to spawn below.
    RX_BYTES.store(0, Ordering::Relaxed);
    RX_PACKETS.store(0, Ordering::Relaxed);
    RX_ERRORS.store(0, Ordering::Relaxed);

    let handle_wrapped = DeviceHandle(handle);
    if let Err(e) = spawn_psram_thread(
        c"uac_reader",
        READER_TASK_STACK,
        Some(AUDIO_TASK_PRIORITY),
        // **PRO_CPU, with the rest of the capture path.**
        //
        // Left unpinned it shares whichever core has room, and at
        // priority 6 that means it can land on APP_CPU beside
        // `stage1_inc` — also 6, and the task whose lateness the whole
        // slot grid rides on. Equal priority there is round-robin, so
        // the reader's resampling takes half of stage1_inc's core
        // whenever they coincide.
        //
        // Measured on a radio (2026-09-19, `logs/udp_monitor`): with
        // the reader unpinned, `hint_err` ran −0.6..−1.1 s and every
        // other slot's SpecBundle arrived 69-90 ms before key-up, where
        // the 500 ms floor dropped it — a real band decoding 8-13
        // stations a slot, and half the slots never tried. The SIM
        // feeder is pinned to core 0, which is why nothing on the bench
        // showed it.
        Some(esp_idf_svc::hal::cpu::Core::Core0),
        move || reader_thread(handle_wrapped, addr, iface_num),
    ) {
        let stop_err = unsafe { sys::uac::uac_host_device_stop(handle) };
        let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
        if stop_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: device_stop after reader spawn failure also failed err={stop_err:#x}"
            );
        }
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: device_close after reader spawn failure also failed err={close_err:#x}"
            );
        } else {
            log::info!("uac: rolled back device_open + device_start after reader spawn failure");
        }
        return Err(anyhow!("uac_reader spawn failed: {e}"));
    }
    log::info!("uac: reader thread spawned");
    set_state(UacState::Streaming);
    Ok(())
}

/// Phase T0 (TX/QSO feasibility, not yet wired to anything that keys
/// PTT): compile-time opt-in probe that the IC-705's USB audio OUT
/// interface — `DriverEvent::TxConnected`, enumerated and logged since
/// #163 but never opened — actually accepts `uac_host_device_open` /
/// `_start` / `_write` the same way `handle_rx_connected` does for the
/// IN side. Off by default; every other `TxConnected` build keeps
/// today's behaviour (log and ignore).
///
/// **Writes digital silence only, never a tone.** The goal here is
/// proving the write path, not proving the radio can be driven — a
/// nonzero signal into the IC-705's USB MOD input could key TX by
/// itself if the radio's PTT source is set to VOX, and this file has
/// no way to know that setting from software. True zero samples carry
/// no audio level to VOX-detect, so they're the safe way to exercise
/// `uac_host_device_write` before anything here can assert PTT on
/// purpose.
///
/// Before running this against a real radio: confirm the IC-705's
/// `PTT SOURCE` menu is anything but `VOX`.
const TX_PROBE_ENABLED: bool = option_env!("MFSK_CORES3_TX_PROBE").is_some();

/// One silent write attempt, this many times, before closing again.
/// Enough to see whether the ring buffer keeps accepting writes past
/// the first one (an OUT endpoint that stalls after N packets would
/// look identical to success on a single write).
const TX_PROBE_WRITES: u32 = 20;
/// One video frame's worth at 48 kHz stereo 16-bit — matches
/// `STREAM_BUFFER_THRESHOLD`'s sizing logic, scaled down since this
/// only needs to exercise the path, not sustain real-time throughput.
const TX_PROBE_CHUNK_BYTES: usize = 1920; // 10 ms @ 48k stereo 16b
const TX_PROBE_WRITE_TIMEOUT_MS: u32 = 100;

/// **Amplitude for a real FT8 frame, and why it is not the default.**
///
/// `MFSK_CORES3_TX_AMPLITUDE=<0..32767>` at build time. **Unset means
/// zero, which means the whole transmit chain runs and the radio hears
/// nothing.** That is the point: `GfskStream`, the 12 k → 48 k
/// upsample, the chunked writes and their timing are all exercised
/// identically at amplitude 0, so the path can be proven before
/// anything can reach the air.
///
/// A nonzero value puts a real signal into the IC-705's USB MOD input,
/// and if the radio's `PTT SOURCE` is `VOX` **that keys the
/// transmitter with no further action from this board**. Software here
/// cannot read that menu. Before setting this:
///
/// - `PTT SOURCE` is not `VOX`, or PTT is under deliberate control
/// - the radio is on a dummy load, or on a band and power the operator
///   intends to transmit on
/// - the frequency is one this station is licensed and willing to
///   transmit on, at the moment the board decides to
///
/// 20 000 of 32 767 is what `m5stack-s3-app`'s `tx.rs` uses (≈ −4 dBFS,
/// headroom for the GFSK envelope while staying above the noise an ALC
/// needs).
const TX_AMPLITUDE: i16 = match option_env!("MFSK_CORES3_TX_AMPLITUDE") {
    Some(s) => crate::decode_pipeline::parse_u32(s) as i16,
    None => 0,
};

/// One 20 ms chunk at 12 kHz — what `GfskStream` is filled with and
/// what `m5stack-s3-app`'s `tx::play` sends, chosen there for DMA
/// underrun headroom against ringbuffer overhead.
const TX_CHUNK_12K: usize = 240;
/// The same chunk as the radio wants it: 4x zero-order hold to 48 kHz,
/// duplicated L = R, 16-bit. `240 * 4 * 2 * 2`.
const TX_CHUNK_BYTES: usize = TX_CHUNK_12K * 4 * 2 * 2;

/// Send one FT8 frame to the radio's USB audio input, synthesising it
/// a chunk at a time.
///
/// Returns `(chunks_written, worst_chunk_us, total_us)`.
///
/// **The waveform never exists as a buffer.** `GfskStream` produces
/// 20 ms at a time straight into the upsample scratch — measured on
/// this board at 439 us per chunk against the 20 ms the chunk
/// represents, a 2.2 % duty. The batch synthesiser it replaces cost
/// 472 ms up front and 1.2 MB of PSRAM temporaries, which put the
/// decoder's deadline before the end of the slot it was decoding
/// (`docs/notes/CORES3_FT8_SLOT_BUDGET.md` §10).
fn write_ft8_frame(
    handle: sys::uac::uac_host_device_handle_t,
    msg77: &[u8; 77],
    df_hz: f32,
    amplitude: i16,
) -> (u32, i64, i64) {
    let tones = mfsk_core::ft8::wave_gen::message_to_tones(msg77);
    let mut stream = mfsk_core::engine::dsp::gfsk::GfskStream::new(
        &tones,
        df_hz,
        &mfsk_core::ft8::wave_gen::FT8_GFSK,
    );
    let mut mono = [0i16; TX_CHUNK_12K];
    let mut out = [0u8; TX_CHUNK_BYTES];
    let now_us = || unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let (mut chunks, mut worst, t_start) = (0u32, 0i64, now_us());
    while stream.remaining() > 0 {
        let t0 = now_us();
        let n = stream.fill_i16(&mut mono, amplitude);
        // 4x zero-order hold, L = R. The receive side's
        // `LinearResamplerI16To12k` run backwards; ZOH rather than
        // interpolation because the radio's own input filter is what
        // shapes the image, and `m5stack-s3-app` has driven an IC-705
        // this way since Phase 1.6.
        let mut o = 0usize;
        for &sm in mono.iter().take(n) {
            let [lo, hi] = sm.to_le_bytes();
            for _ in 0..4 {
                out[o] = lo;
                out[o + 1] = hi;
                out[o + 2] = lo;
                out[o + 3] = hi;
                o += 4;
            }
        }
        let err = unsafe {
            sys::uac::uac_host_device_write(
                handle,
                out.as_mut_ptr(),
                o as u32,
                TX_PROBE_WRITE_TIMEOUT_MS,
            )
        };
        if err != sys::ESP_OK as sys::esp_err_t {
            log::warn!("uac: tx write failed at chunk {chunks} err={err:#x}");
            break;
        }
        chunks += 1;
        worst = worst.max(now_us() - t0);
    }
    (chunks, worst, now_us() - t_start)
}

fn handle_tx_connected(addr: u8, iface_num: u8) {
    log::info!("uac: TX_CONNECTED addr={addr} iface={iface_num} — probe start (silence only)");
    let dev_config = sys::uac::uac_host_device_config_t {
        addr,
        iface_num,
        buffer_size: STREAM_BUFFER_BYTES,
        buffer_threshold: STREAM_BUFFER_THRESHOLD,
        callback: Some(device_event_cb),
        callback_arg: core::ptr::null_mut(),
    };
    let mut handle: sys::uac::uac_host_device_handle_t = core::ptr::null_mut();
    let err = unsafe {
        sys::uac::uac_host_device_open(
            &dev_config as *const _,
            &mut handle as *mut sys::uac::uac_host_device_handle_t,
        )
    };
    if err != sys::ESP_OK as sys::esp_err_t {
        log::error!("uac: tx probe device_open failed err={err:#x}");
        return;
    }

    // Same fixed config the IN side uses. Unverified for the OUT
    // interface specifically — the IC-705's TX descriptor has never
    // been queried, so a `device_start` failure here is expected
    // information, not a bug: check the device descriptor dump in the
    // UDP log (same as the RX open-failure comment above) and correct
    // this constant from what the radio actually reports.
    let stream_config = sys::uac::uac_host_stream_config_t {
        channels: STREAM_CHANNELS,
        bit_resolution: STREAM_BIT_RESOLUTION,
        sample_freq: STREAM_SAMPLE_FREQ_HZ,
        flags: 0,
    };
    let err = unsafe { sys::uac::uac_host_device_start(handle, &stream_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        log::error!(
            "uac: tx probe device_start failed err={err:#x} (tried {STREAM_CHANNELS}ch / \
             {STREAM_BIT_RESOLUTION}b / {STREAM_SAMPLE_FREQ_HZ}Hz — this is a guess, not read \
             from the OUT interface's own descriptor)"
        );
        let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: tx probe device_close after start failure failed err={close_err:#x}");
        }
        return;
    }

    let silence = [0u8; TX_PROBE_CHUNK_BYTES];
    let mut wrote_ok = 0u32;
    for i in 0..TX_PROBE_WRITES {
        let mut buf = silence;
        let err = unsafe {
            sys::uac::uac_host_device_write(
                handle,
                buf.as_mut_ptr(),
                buf.len() as u32,
                TX_PROBE_WRITE_TIMEOUT_MS,
            )
        };
        if err == sys::ESP_OK as sys::esp_err_t {
            wrote_ok += 1;
        } else {
            log::warn!("uac: tx probe write {i}/{TX_PROBE_WRITES} failed err={err:#x}");
        }
        esp_idf_svc::hal::delay::FreeRtos::delay_ms(10);
    }
    log::info!("uac: tx probe wrote {wrote_ok}/{TX_PROBE_WRITES} silent chunks OK");

    // **The whole transmit chain, at whatever amplitude is configured
    // — which is zero unless someone set it.**
    //
    // Twenty 10 ms writes prove the endpoint accepts data. They do not
    // prove a transmission: a frame is 632 consecutive chunks over
    // 12.64 s, paced by the ring's own backpressure, with `GfskStream`
    // producing each one 439 us before it is needed. Whether that
    // holds for twelve seconds is a different question from whether
    // twenty writes succeed, and it is the question a QSO depends on.
    //
    // At `TX_AMPLITUDE = 0` every part of that runs and the radio
    // hears silence — see that constant for what a nonzero value
    // means and what has to be true first.
    if wrote_ok > 0 {
        let msg77 = match mfsk_core::msg::wsjt77::pack77(
            "CQ",
            crate::decode_pipeline::MY_CALL,
            crate::decode_pipeline::MY_GRID,
        ) {
            Some(m) => m,
            None => {
                log::warn!(
                    "uac: tx frame skipped — pack77 failed for CQ {} {}",
                    crate::decode_pipeline::MY_CALL,
                    crate::decode_pipeline::MY_GRID
                );
                [0u8; 77]
            }
        };
        let (chunks, worst_us, total_us) = write_ft8_frame(handle, &msg77, 1_500.0, TX_AMPLITUDE);
        log::warn!(
            "uac: tx frame {} — {chunks} chunks of 20 ms, worst {worst_us} us, total {} ms \
             (audio is 12 640 ms; a shortfall is the ring not pacing, an excess is this board \
             not keeping up)",
            if TX_AMPLITUDE == 0 {
                "SILENT (amplitude 0, nothing reaches the air)"
            } else {
                "AT FULL AMPLITUDE — the radio may be transmitting"
            },
            total_us / 1_000,
        );
    }

    let stop_err = unsafe { sys::uac::uac_host_device_stop(handle) };
    let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
    if stop_err != sys::ESP_OK as sys::esp_err_t {
        log::error!("uac: tx probe device_stop failed err={stop_err:#x}");
    }
    if close_err != sys::ESP_OK as sys::esp_err_t {
        log::error!("uac: tx probe device_close failed err={close_err:#x}");
    }
    log::info!("uac: tx probe done, interface closed");
}

/// Reader thread body. Polls `uac_host_device_read` for raw
/// 48 kHz/stereo/16-bit iso IN packets, extracts the left channel,
/// resamples to 12 kHz mono via `LinearResamplerI16To12k`, and pushes
/// `CHUNK_LEN`-sized chunks into the decode pipeline's chunk queue
/// (with `SlotEnd` every `SLOT_SAMPLES_12K` samples).
///
/// Counts iso IN throughput in `RX_BYTES` / `RX_PACKETS` / `RX_ERRORS`
/// and logs a 1 Hz status line.
fn reader_thread(handle: DeviceHandle, addr: u8, iface_num: u8) {
    // RAII: clears READER_ACTIVE on any exit path including panic,
    // and — when `reopen` is set below — asks for a fresh session.
    let mut gate = ReaderActiveGuard { reopen: None };
    let mut buf = [0u8; READER_BUFFER_BYTES];
    let mut resampler = LinearResamplerI16To12k::new(STREAM_SAMPLE_FREQ_HZ);
    // L-channel scratch (one device_read worth of mono samples). At
    // 4 KB raw / stereo i16, max = 1024 mono samples per read.
    let mut left_scratch = [0i16; READER_BUFFER_BYTES / 4];
    // Resampled output staging. Sized for at most one read's worth
    // of input → ~256 output samples at 48k→12k (4:1). Doubled for
    // headroom against the resampler's per-call rounding.
    let mut dst_scratch = [0i16; 512];
    let mut last_log = std::time::Instant::now();
    // When audio last actually arrived — the stall watchdog's clock.
    let mut last_data = std::time::Instant::now();
    // Completion time of the previous read, for `READ_GAP_MAX_US`.
    let mut last_read_done: i64 = 0;
    let mut last_bytes: u32 = 0;
    // Post-resample signal statistics for the 1 Hz tick — issue #163.
    //
    // The byte counter below says the *transport* is alive. It cannot
    // distinguish a radio streaming real audio from one streaming
    // 192 kB/s of digital silence, which is what a muted source, a
    // wrong input selection or an unconfigured codec all look like.
    // A live bring-up session is expensive enough that "some bytes
    // arrived" is not a result worth coming away with, so measure the
    // thing the decoder actually consumes: how many 12 kHz samples per
    // second (a rate check — it should be 12 000), and how loud they
    // are.
    let mut out_samples: u32 = 0;
    let mut peak: i32 = 0;
    let mut sum_sq: u64 = 0;
    let mut clipped: u32 = 0;
    // Track whether we exited via DISCONNECTED so the cleanup path
    // can skip the redundant `device_stop` / `device_close` calls
    // — the IDF driver already invalidated the handle at
    // callback time, so calling them just logs spurious INVALID_ARG.
    let mut disconnect_triggered = false;
    loop {
        // Top-of-loop disconnect check. `device_event_cb` sets
        // the flag immediately on DISCONNECTED; we exit on the next
        // iteration (≤ `READER_READ_TIMEOUT_MS` latency) rather than
        // waiting for the failing read to surface.
        if READER_STOP_REQUESTED.load(Ordering::Acquire) {
            log::info!("uac: reader exiting — DISCONNECTED signaled by device_event_cb");
            // Hot-unplug: back to waiting, and the screen says so.
            set_state(UacState::Waiting);
            disconnect_triggered = true;
            break;
        }
        // 1 Hz throughput log. `Instant::now()` is the FreeRTOS tick
        // count under the hood — sub-microsecond cost.
        let now = std::time::Instant::now();
        if now.duration_since(last_log).as_secs() >= 1 {
            let bytes = RX_BYTES.load(Ordering::Relaxed);
            let packets = RX_PACKETS.load(Ordering::Relaxed);
            let errors = RX_ERRORS.load(Ordering::Relaxed);
            let bps = bytes.wrapping_sub(last_bytes);
            // A second of real audio means the last re-open worked, so
            // the budget resets. A device that hiccups every few
            // minutes then gets retried forever, which is the point;
            // the cap is only there for one that never streams at all.
            if bps > 0 {
                REOPEN_ATTEMPTS.store(0, Ordering::Relaxed);
            }
            // 48 k × stereo × 2 B = 192_000 B/s expected for a fully-
            // streaming IC-705. The throughput delta is the diagnostic
            // we care about here (anything well below ~190 kB/s
            // suggests packet drops or wrong stream config).
            let rms = if out_samples > 0 {
                ((sum_sq / out_samples as u64) as f64).sqrt()
            } else {
                0.0
            };
            // dBFS against full scale, so "is there signal" is one
            // glance rather than an i16 magnitude to interpret.
            let dbfs = if rms > 0.0 {
                20.0 * (rms / 32_768.0).log10()
            } else {
                -99.0
            };
            // Internal DRAM goes in the per-second line, not just the
            // 6 s alive tick. The board reboots about one second after
            // the stream starts, with nothing from the Rust panic hook
            // — which is what an allocation failure or a hardware
            // exception looks like, both handled in C below the log
            // path. Three attached devices now hold a 4 KB control
            // buffer each, and the decode pipeline allocates on top of
            // that, so "how much was left when it died" is the first
            // thing worth knowing. Refs #163.
            let free_internal = unsafe {
                sys::heap_caps_get_free_size(sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT)
            };
            // `bps` is bytes in this interval, and the interval is
            // only *at least* a second — so it is printed, or a slow
            // tick reads as lost audio.
            let int_ms = now.duration_since(last_log).as_millis();
            let blk_max = SINK_BLOCK_MAX_US.swap(0, Ordering::Relaxed);
            let blk_sum = SINK_BLOCK_SUM_US.swap(0, Ordering::Relaxed);
            let gap_max = READ_GAP_MAX_US.swap(0, Ordering::Relaxed);
            let to = READ_TIMEOUTS.swap(0, Ordering::Relaxed);
            log::info!(
                "uac: rx tick: {bps} B/{int_ms}ms ({packets} pkt / {errors} err) \
                 | audio {out_samples} sa/s, rms {dbfs:.1} dBFS, peak {peak}, clip {clipped} \
                 | blk {blk_max}/{blk_sum}us gap {gap_max}us to {to} | int={free_internal}",
            );
            let _ = bytes;
            UAC_SA_PER_S.store(out_samples, Ordering::Release);
            UAC_RMS_MDB.store(
                if out_samples > 0 {
                    ((-dbfs) * 10.0).clamp(0.0, (u32::MAX - 1) as f64) as u32
                } else {
                    u32::MAX
                },
                Ordering::Release,
            );
            last_log = now;
            last_bytes = bytes;
            out_samples = 0;
            peak = 0;
            sum_sq = 0;
            clipped = 0;
        }

        // Stall watchdog.
        //
        // The timeout path below used to `continue`, which skipped the
        // log above as well — so a reader whose device had gone quiet
        // spun at 10 Hz emitting nothing at all, indistinguishable
        // from a dead thread. Both halves are fixed here: the tick
        // runs before the read so "0 B/s" is visible, and going quiet
        // for long enough now ends the session instead of hanging on
        // to it. Refs #163.
        if now.duration_since(last_data).as_millis() as u64 >= STALL_TIMEOUT_MS {
            log::error!(
                "uac: no audio for {} ms — ending session (addr={addr} iface={iface_num})",
                now.duration_since(last_data).as_millis()
            );
            set_state(UacState::Error);
            gate.reopen = Some((addr, iface_num));
            break;
        }

        let mut bytes_read: u32 = 0;
        let t_read_start = unsafe { sys::esp_timer_get_time() };
        if last_read_done > 0 {
            let gap = (t_read_start - last_read_done).clamp(0, u32::MAX as i64) as u32;
            READ_GAP_MAX_US.fetch_max(gap, Ordering::Relaxed);
        }
        let err = unsafe {
            sys::uac::uac_host_device_read(
                handle.0,
                buf.as_mut_ptr(),
                buf.len() as u32,
                &mut bytes_read as *mut u32,
                READER_READ_TIMEOUT_MS,
            )
        };
        if err != sys::ESP_OK as sys::esp_err_t {
            // ESP_ERR_TIMEOUT is a routine ringbuf-empty signal (the
            // 100 ms timeout fires when the IDF driver hasn't yet
            // received a fresh iso IN frame). NOT a reason to exit
            // (Gemini PR #98 review). Just continue the loop.
            if err == sys::ESP_ERR_TIMEOUT as sys::esp_err_t {
                READ_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
                last_read_done = unsafe { sys::esp_timer_get_time() };
                continue;
            }
            RX_ERRORS.fetch_add(1, Ordering::Relaxed);
            // Other errors (INVALID_STATE on disconnect, INVALID_ARG,
            // ringbuf failures) are terminal — end the session and
            // hand the interface to the re-open gate below, which
            // retries it up to `REOPEN_MAX_ATTEMPTS` times.
            // `BootMode::Uac` is sticky-until-reboot, so a reader that
            // dies past that is at least observable in the next UDP
            // log tick (rx=0B/s).
            log::error!("uac: device_read err={err:#x}, ending session");
            set_state(UacState::Error);
            gate.reopen = Some((addr, iface_num));
            break;
        }
        last_read_done = unsafe { sys::esp_timer_get_time() };
        if bytes_read > 0 {
            last_data = std::time::Instant::now();
        }
        if bytes_read == 0 {
            // 0-byte read = timeout-with-no-data (rare); skip the
            // resample / push pipeline and continue.
            continue;
        }
        // RX_BYTES uses `fetch_add` which wraps on AtomicU32 overflow
        // (~4 GB ≈ 6 h of streaming). xtensa-esp32s3 has no 64-bit
        // atomic intrinsics. The wrap is harmless for the 1 Hz delta
        // computation below — `wrapping_sub` on the u32 values gives
        // the correct per-second window even across the boundary.
        RX_BYTES.fetch_add(bytes_read, Ordering::Relaxed);
        RX_PACKETS.fetch_add(1, Ordering::Relaxed);

        // Decode interleaved stereo i16 → take left channel only.
        // bytes_read is always a multiple of 4 (stereo i16) per the
        // IDF driver's frame alignment. left_scratch is sized for
        // the max bytes_read / 4 case so the slice can't overflow.
        let stereo_samples = (bytes_read as usize) / 4;
        debug_assert!(stereo_samples <= left_scratch.len());
        for i in 0..stereo_samples {
            let off = i * 4;
            // i16 LE, little-endian (USB Audio Class default).
            left_scratch[i] = i16::from_le_bytes([buf[off], buf[off + 1]]);
        }

        // Route through the registered `AudioSink` (FT8's chunk
        // queue, WSPR's DDC push, ...). While unregistered we just
        // drop the current read buffer (lossy by design — the race
        // window is bounded by how fast the consumer thread can spawn
        // + register, ~200 ms). Gemini PR #99 review fixed the
        // earlier "accumulates samples" wording which contradicted
        // the actual `continue`.
        let mut sink_guard = match AUDIO_SINK.lock() {
            Ok(g) => g,
            Err(e) => {
                log::error!("uac: AUDIO_SINK mutex poisoned: {e}");
                continue;
            }
        };
        let Some(sink) = sink_guard.as_mut() else {
            continue;
        };

        // Feed the resampler in a loop until the input is drained.
        // process() returns (consumed, produced); if consumed < input
        // we loop back with the unconsumed tail.
        let mut src_offset = 0usize;
        while src_offset < stereo_samples {
            let (consumed, produced) =
                resampler.process(&left_scratch[src_offset..stereo_samples], &mut dst_scratch);
            if produced > 0 {
                for &v in &dst_scratch[..produced] {
                    let a = (v as i32).abs();
                    if a > peak {
                        peak = a;
                    }
                    if a >= 32_000 {
                        clipped += 1;
                    }
                    sum_sq += (a as u64) * (a as u64);
                }
                out_samples += produced as u32;
                sink.push_samples(&dst_scratch[..produced]);
            }
            // Defensive: if process() makes zero progress (shouldn't,
            // given the input is non-empty), break to avoid an
            // infinite loop.
            if consumed == 0 && produced == 0 {
                break;
            }
            src_offset += consumed;
        }
        drop(sink_guard);
    }
    // Cleanup paths differ by exit reason:
    //
    // - Disconnect-triggered exit: the IDF driver already invalidated
    //   the handle when DISCONNECTED fired in `device_event_cb`.
    //   Calling `device_stop` / `device_close` on the invalidated
    //   handle returns INVALID_ARG/STATE — harmless but it would log
    //   confusing errors. Skip.
    // - Read-error exit (terminal non-TIMEOUT error from device_read):
    //   the handle is still nominally valid; explicit stop + close
    //   releases the IDF state so a re-enumeration takes a clean path.
    //
    // Re-consult the atomic at cleanup time so a DISCONNECT that
    // fired between the top-of-loop check and a later break (read
    // error, chunk-push failure, slot-push failure) still routes
    // to the skip path. Without this, DISCONNECT racing a chunk
    // push would still emit the spurious INVALID_ARG logs.
    //
    // Either way `_gate: ReaderActiveGuard` drops below to release
    // `READER_ACTIVE` so the next RxConnected can re-take it.
    let disconnect_triggered =
        disconnect_triggered || READER_STOP_REQUESTED.load(Ordering::Acquire);
    if disconnect_triggered {
        log::info!(
            "uac: reader_thread cleanup (disconnect path — skipping device_stop/close, IDF already invalidated handle)"
        );
    } else {
        let stop_err = unsafe { sys::uac::uac_host_device_stop(handle.0) };
        if stop_err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: device_stop on reader exit failed err={stop_err:#x}");
        }
        let close_err = unsafe { sys::uac::uac_host_device_close(handle.0) };
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: device_close on reader exit failed err={close_err:#x}");
        } else {
            log::info!("uac: reader_thread cleanup complete (device stopped + closed)");
        }
    }
    // `_gate: ReaderActiveGuard` is dropped here, clearing
    // READER_ACTIVE so the next RxConnected can re-take the gate
    // (Gemini PR #98 r3 + r4 review — RAII so panic also clears).
}

/// App task body. Consumes driver events from the channel; on first
/// `RxConnected` opens the device + starts the stream + spawns the
/// reader. Subsequent `RxConnected` events (e.g. a hub adds another
/// audio device, or IC-705 re-enumerates after a USB reset) are
/// re-handled — telling "same device returned" from "new device" and
/// closing the previous handle is still open, with no issue of its
/// own.
fn app_task(rx: std::sync::mpsc::Receiver<DriverEvent>) {
    while let Ok(event) = rx.recv() {
        match event {
            DriverEvent::RxConnected { addr, iface_num } => {
                // Dedup: the IDF driver re-fires `RxConnected` on alt-
                // setting transitions and (per Gemini PR #98 review)
                // on multi-interface devices, which would spawn
                // additional reader threads racing the first on
                // device_start. compare_exchange wins atomically;
                // losers just log and drop the event.
                if READER_ACTIVE
                    .compare_exchange(
                        false,
                        true,
                        std::sync::atomic::Ordering::AcqRel,
                        std::sync::atomic::Ordering::Acquire,
                    )
                    .is_err()
                {
                    log::info!(
                        "uac: ignoring duplicate RxConnected addr={addr} iface={iface_num} (reader already active)"
                    );
                    continue;
                }
                if let Err(e) = handle_rx_connected(addr, iface_num) {
                    log::error!("uac: RxConnected handler failed: {e:#}");
                    // Release the gate so a future re-attach can retry
                    // (e.g. IC-705 power cycle during bring-up).
                    READER_ACTIVE.store(false, std::sync::atomic::Ordering::Release);
                }
            }
            DriverEvent::TxConnected { addr, iface_num } => {
                if TX_PROBE_ENABLED {
                    handle_tx_connected(addr, iface_num);
                } else {
                    log::info!(
                        "uac: TX_CONNECTED addr={addr} iface={iface_num} — ignored (RX only for FT8)"
                    );
                }
            }
        }
    }
    log::error!("uac: app_task exiting — driver event channel closed");
}

/// Install the USB host stack + the UAC class driver. Returns once
/// both are running in their respective background tasks; the caller
/// (main.rs UAC dispatch arm) falls through to the display loop.
/// Whether [`start_host`] installed the host driver on this boot.
///
/// `UacState::Off` cannot answer this: it means both "the host is
/// installed and idle" and "there is no host, because a PC is powering
/// the port". Those are the two states the link bar has to tell apart,
/// and confusing them is what made a charging board look like a broken
/// UAC stack.
static HOST_INSTALLED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

pub fn host_installed() -> bool {
    HOST_INSTALLED.load(core::sync::atomic::Ordering::Acquire)
}

/// What `start_host` concluded, kept for re-emission once a log sink
/// exists.
///
/// The result is printed the moment it happens, which on battery is
/// before WiFi has associated — so it goes into the fanout's staging
/// ring, and the ring is small enough that the alive tick pushes it out
/// before the UDP sink can replay it. The single most important line of
/// the boot was therefore never seen on the only channel that works in
/// host mode. Allocation-free slot, same one the diagnostic probes use.
pub static HOST_RESULT: crate::log_slot::LogSlot = crate::log_slot::LogSlot::new();

pub fn start_host() -> Result<()> {
    // Set up the driver→app channel BEFORE installing the class
    // driver — `driver_event_cb` may fire as soon as `uac_host_install`
    // returns (an IC-705 already plugged in would enumerate immediately).
    let (tx, rx) = channel::<DriverEvent>();
    EVENT_SENDER
        .set(tx)
        .map_err(|_| anyhow!("uac: EVENT_SENDER double init — start_host called twice?"))?;
    spawn_psram_thread(c"uac_app", APP_TASK_STACK, None, None, move || app_task(rx))
        .map_err(|e| anyhow!("uac_app spawn failed: {e}"))?;
    log::info!("uac: app_task spawned (stack={APP_TASK_STACK} B)");

    log::info!("uac: installing USB host stack");
    let host_config = sys::usb_host_config_t {
        skip_phy_setup: false,
        root_port_unpowered: false,
        intr_flags: sys::ESP_INTR_FLAG_LEVEL1 as i32,
        enum_filter_cb: None,
        peripheral_map: 0,
        fifo_settings_custom: sys::usb_host_config_t__bindgen_ty_1 {
            nptx_fifo_lines: 0,
            ptx_fifo_lines: 0,
            rx_fifo_lines: 0,
        },
    };
    let err = unsafe { sys::usb_host_install(&host_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        return Err(anyhow!("usb_host_install failed (err={err:#x})"));
    }

    if let Err(e) =
        spawn_psram_thread(
            c"usb_events",
            USB_EVENTS_TASK_STACK,
            // Part of the audio path: it is what dispatches the host
            // library's events, so it answers to
            // `UAC_DRIVER_TASK_PRIORITY`'s argument, not to a default.
            Some(AUDIO_TASK_PRIORITY),
            None,
            usb_events_task,
        )
    {
        let uninstall_err = unsafe { sys::usb_host_uninstall() };
        if uninstall_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: usb_host_uninstall after spawn failure also failed (err={uninstall_err:#x}); host stack left in inconsistent state"
            );
        } else {
            log::info!("uac: rolled back usb_host_install after spawn failure");
        }
        return Err(anyhow!("usb_events_task spawn failed: {e}"));
    }
    log::info!("uac: usb_events_task spawned (stack={USB_EVENTS_TASK_STACK} B)");

    log::info!("uac: installing UAC class driver");
    let uac_config = sys::uac::uac_host_driver_config_t {
        create_background_task: true,
        task_priority: UAC_DRIVER_TASK_PRIORITY,
        stack_size: UAC_DRIVER_TASK_STACK,
        core_id: UAC_DRIVER_TASK_CORE,
        callback: Some(driver_event_cb),
        callback_arg: core::ptr::null_mut(),
    };
    let err = unsafe { sys::uac::uac_host_install(&uac_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        // Rollback: unblock events pump, wait for settle, uninstall host stack.
        let unblock_err = unsafe { sys::usb_host_lib_unblock() };
        if unblock_err != sys::ESP_OK as sys::esp_err_t {
            log::warn!("uac: usb_host_lib_unblock before rollback returned err={unblock_err:#x}");
        }
        esp_idf_svc::hal::delay::FreeRtos::delay_ms(20);
        let uninstall_err = unsafe { sys::usb_host_uninstall() };
        if uninstall_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: usb_host_uninstall after uac_host_install failure also failed (err={uninstall_err:#x}); host stack left in inconsistent state"
            );
        } else {
            log::info!("uac: rolled back usb_host_install after uac_host_install failure");
        }
        return Err(anyhow!("uac_host_install failed (err={err:#x})"));
    }

    log::info!(
        "uac: host + class driver up — waiting for IC-705 enumeration (driver task core={UAC_DRIVER_TASK_CORE}, prio={UAC_DRIVER_TASK_PRIORITY})"
    );
    set_state(UacState::Waiting);
    HOST_INSTALLED.store(true, core::sync::atomic::Ordering::Release);
    HOST_RESULT.store("host+class driver installed OK");
    spawn_device_count_probe();
    Ok(())
}

/// 何秒かおきに USB ホストライブラリが把握しているデバイス数を吐く。
///
/// 「列挙されない」には段階があり、どこで止まっているかで打ち手が
/// まったく変わる — VBUS が出ていないのか、ハブは見えているが下流が
/// 歩けていないのか、下流まで見えていて UAC ドライバが掴めていないのか。
/// `usb_host_lib_info()` の `num_devices` は列挙まで終わった台数なので、
/// IC-705 のように内蔵ハブを持つ機器なら、ハブ + CDC + オーディオで
/// 複数台に見えるのが正常。0 のまま動かないなら、そもそもポートに
/// 何も見えていないということで、UAC ドライバより手前の問題になる。
///
/// Refs #163.
/// No-op client callback. The dump opens devices to read descriptors
/// and never submits a transfer, so it has no events to handle — but
/// `usb_host_client_register` requires a function pointer.
extern "C" fn dump_client_cb(
    _event: *const sys::usb_host_client_event_msg_t,
    _arg: *mut core::ffi::c_void,
) {
}

/// Print every enumerated device and every interface it offers.
///
/// **Written because the comments around `uac_host_device_start`
/// already told the reader to "check the device descriptor dump in the
/// UDP log", and there was no such dump.** The OUT interface's
/// channels/bits/rate have been a guess since the probe was written,
/// and whether the radio offers an audio OUT interface at all is
/// unestablished — both are answerable straight from the descriptors.
///
/// An IC-705 on USB should show a CDC/serial function for CI-V and an
/// audio function with a streaming interface each way, which is what
/// `num_devices=3` has been hinting at without saying.
///
/// Registers its own client rather than borrowing the UAC driver's:
/// the host stack allows several, and a reader that owns nothing
/// cannot disturb the capture that is running.
fn dump_enumeration() {
    let cfg = sys::usb_host_client_config_t {
        is_synchronous: false,
        max_num_event_msg: 5,
        __bindgen_anon_1: sys::usb_host_client_config_t__bindgen_ty_1 {
            async_: sys::usb_host_client_config_t__bindgen_ty_1__bindgen_ty_1 {
                client_event_callback: Some(dump_client_cb),
                callback_arg: core::ptr::null_mut(),
            },
        },
    };
    let mut client: sys::usb_host_client_handle_t = core::ptr::null_mut();
    // SAFETY: called after the host library is installed; `cfg` lives
    // for the duration of the call.
    let err = unsafe { sys::usb_host_client_register(&cfg, &mut client) };
    if err != sys::ESP_OK {
        log::warn!("uac: enumeration dump — client_register failed err={err:#x}");
        return;
    }

    let mut addrs = [0u8; 8];
    let mut n: core::ffi::c_int = 0;
    // SAFETY: `addrs` is `len` bytes and `n` is written by the callee.
    let err = unsafe {
        sys::usb_host_device_addr_list_fill(addrs.len() as i32, addrs.as_mut_ptr(), &mut n)
    };
    if err != sys::ESP_OK {
        log::warn!("uac: enumeration dump — addr_list_fill failed err={err:#x}");
        unsafe { sys::usb_host_client_deregister(client) };
        return;
    }
    log::warn!("uac: enumeration dump — {n} device(s)");

    for &addr in addrs.iter().take(n.max(0) as usize) {
        let mut dev: sys::usb_device_handle_t = core::ptr::null_mut();
        // SAFETY: `client` is registered; `dev` is written on success.
        if unsafe { sys::usb_host_device_open(client, addr, &mut dev) } != sys::ESP_OK {
            log::warn!("uac:   addr {addr}: open failed");
            continue;
        }
        let mut dd: *const sys::usb_device_desc_t = core::ptr::null();
        // SAFETY: `dev` is open; the pointer returned is owned by the
        // stack and valid until the device is closed.
        if unsafe { sys::usb_host_get_device_descriptor(dev, &mut dd) } == sys::ESP_OK
            && !dd.is_null()
        {
            // The bindings wrap the packed body in an anonymous
            // union, and it is `#[repr(packed)]`, so every field is
            // copied out rather than referenced — a reference into a
            // packed struct is UB even unread.
            //
            // SAFETY: the union has a single variant in these
            // bindings and the stack filled the descriptor.
            let (vid, pid, cls, sub, proto, ncfg) = unsafe {
                let d = core::ptr::addr_of!((*dd).__bindgen_anon_1);
                (
                    core::ptr::addr_of!((*d).idVendor).read_unaligned(),
                    core::ptr::addr_of!((*d).idProduct).read_unaligned(),
                    core::ptr::addr_of!((*d).bDeviceClass).read_unaligned(),
                    core::ptr::addr_of!((*d).bDeviceSubClass).read_unaligned(),
                    core::ptr::addr_of!((*d).bDeviceProtocol).read_unaligned(),
                    core::ptr::addr_of!((*d).bNumConfigurations).read_unaligned(),
                )
            };
            log::warn!(
                "uac:   addr {addr}: VID {vid:04x} PID {pid:04x} class {cls}/{sub}/{proto} \
                 configs {ncfg}"
            );
        }
        let mut cd: *const sys::usb_config_desc_t = core::ptr::null();
        // SAFETY: as above.
        if unsafe { sys::usb_host_get_active_config_descriptor(dev, &mut cd) } == sys::ESP_OK
            && !cd.is_null()
        {
            // Same anonymous-union wrapping as the device descriptor.
            // SAFETY: as above.
            let total = unsafe {
                core::ptr::addr_of!((*cd).__bindgen_anon_1.wTotalLength).read_unaligned()
            } as usize;
            // SAFETY: the stack guarantees `wTotalLength` bytes behind
            // the descriptor; walked read-only.
            let bytes = unsafe { core::slice::from_raw_parts(cd as *const u8, total) };
            walk_config(bytes);
        }
        // SAFETY: opened above by this client.
        unsafe { sys::usb_host_device_close(client, dev) };
    }
    // SAFETY: registered above, and every device it opened is closed.
    unsafe { sys::usb_host_client_deregister(client) };
}

/// Walk a configuration descriptor's TLV chain, printing interfaces
/// and endpoints.
///
/// Manual rather than through a helper because what matters is the
/// *audio streaming* interfaces and their directions, and a generic
/// pretty-printer buries those in the class-specific records the UAC
/// spec puts between them.
fn walk_config(bytes: &[u8]) {
    const T_INTERFACE: u8 = 0x04;
    const T_ENDPOINT: u8 = 0x05;
    let mut i = 0usize;
    while i + 2 <= bytes.len() {
        let len = bytes[i] as usize;
        if len < 2 || i + len > bytes.len() {
            break;
        }
        match bytes[i + 1] {
            T_INTERFACE if len >= 9 => {
                let (num, alt, neps) = (bytes[i + 2], bytes[i + 3], bytes[i + 4]);
                let (cls, sub, proto) = (bytes[i + 5], bytes[i + 6], bytes[i + 7]);
                let name = match (cls, sub) {
                    (0x01, 0x01) => " (audio control)",
                    (0x01, 0x02) => " (AUDIO STREAMING)",
                    (0x02, _) => " (CDC control — CI-V)",
                    (0x0a, _) => " (CDC data — CI-V)",
                    _ => "",
                };
                log::warn!(
                    "uac:     iface {num} alt {alt}: class {cls:#04x}/{sub:#04x}/{proto:#04x} \
                     {neps} endpoint(s){name}"
                );
            }
            T_ENDPOINT if len >= 7 => {
                let addr = bytes[i + 2];
                let attr = bytes[i + 3];
                let mps = u16::from_le_bytes([bytes[i + 4], bytes[i + 5]]);
                log::warn!(
                    "uac:       ep {:#04x} {} {} maxpkt {mps}",
                    addr,
                    if addr & 0x80 != 0 { "IN " } else { "OUT" },
                    match attr & 0x03 {
                        0 => "control",
                        1 => "isochronous",
                        2 => "bulk",
                        _ => "interrupt",
                    },
                );
            }
            _ => {}
        }
        i += len;
    }
}

fn spawn_device_count_probe() {
    let _ = crate::board::spawn_named(c"uac_probe", 3072, || {
        let mut last: i32 = -1;
        let mut since_log = u32::MAX;
        loop {
            let mut info = sys::usb_host_lib_info_t::default();
            // SAFETY: ホストライブラリ導入後にのみ呼ばれる。
            let err = unsafe { sys::usb_host_lib_info(&mut info) };
            if err == sys::ESP_OK {
                DEVICE_COUNT.store(info.num_devices, Ordering::Relaxed);
                CLIENT_COUNT.store(info.num_clients, Ordering::Relaxed);
                // 変化時は即、そうでなくても 10 秒おきに。LCD の
                // ログパネルは数行しか出ないので、変化時だけだと
                // 起動直後の 1 行が流れて消えて読めない。
                if info.num_devices != last || since_log >= 5 {
                    // The latched interface numbers ride along: they
                    // answer "did an audio IN / OUT interface ever
                    // appear" from any point in the log, which the
                    // one-shot enumeration lines cannot when the
                    // network that carries them is not up yet.
                    let (rx, tx) = (
                        RX_IFACE_SEEN.load(Ordering::Relaxed),
                        TX_IFACE_SEEN.load(Ordering::Relaxed),
                    );
                    let fmt = |v: i32| {
                        if v < 0 {
                            String::from("never")
                        } else {
                            format!("iface {v}")
                        }
                    };
                    // **Dump the descriptors the first time the device
                    // count settles**, not at install: enumeration
                    // finishes after `start_host` returns, and on
                    // 2026-09-20 the network that carries the log was
                    // 33 s behind it. Printed from this loop the dump
                    // lands wherever the log is actually reaching.
                    if info.num_devices > 0 && !DUMPED.swap(true, Ordering::Relaxed) {
                        dump_enumeration();
                    }
                    log::info!(
                        "uac: usb_host_lib_info — num_devices={} num_clients={} | audio IN {} \
                         | audio OUT {}",
                        info.num_devices,
                        info.num_clients,
                        fmt(rx),
                        fmt(tx),
                    );
                    last = info.num_devices;
                    since_log = 0;
                } else {
                    since_log += 1;
                }
            } else {
                log::warn!("uac: usb_host_lib_info failed (err={err:#x})");
            }
            std::thread::sleep(std::time::Duration::from_secs(2));
        }
    });
}

/// ホストライブラリが把握している列挙済みデバイス数。probe 未起動なら `-1`。
static DEVICE_COUNT: AtomicI32 = AtomicI32::new(-1);
/// 登録済みクライアント数 (UAC ドライバが 1 つ登録する)。
static CLIENT_COUNT: AtomicI32 = AtomicI32::new(-1);
/// クラスドライバのイベント通知回数。0 のままなら UAC ドライバは
/// 一度も呼ばれていない。
/// Interface number of the last `RxConnected` / `TxConnected` the
/// driver raised, or `-1` if it never has. See the latch in
/// `driver_event_cb`: the enumeration log can be written before the
/// network that carries it exists, and a missing line then reads as a
/// missing interface.
/// One-shot guard for [`dump_enumeration`].
static DUMPED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

static RX_IFACE_SEEN: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(-1);
/// See [`RX_IFACE_SEEN`]. **`-1` here is the answer to "does the
/// IC-705 offer a USB audio OUT interface at all"**, which nothing has
/// established yet.
static TX_IFACE_SEEN: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(-1);

static DRIVER_EVENTS: AtomicU32 = AtomicU32::new(0);
/// 直近のエラーコード (0 = なし)。
static LAST_ERR: AtomicU32 = AtomicU32::new(0);

/// 画面の USB パネルに出す一式: (デバイス数, クライアント数,
/// ドライバイベント数, 直近エラー)。
pub fn usb_counters() -> (i32, i32, u32, u32) {
    (
        DEVICE_COUNT.load(Ordering::Relaxed),
        CLIENT_COUNT.load(Ordering::Relaxed),
        DRIVER_EVENTS.load(Ordering::Relaxed),
        LAST_ERR.load(Ordering::Relaxed),
    )
}

/// Record the last USB host error for the link bar.
///
/// **Nothing calls this**, which means `LAST_ERR` is always 0 and the
/// link bar's error field has never shown anything. Surfaced by the
/// 2026-08-30 lib split — in a bin crate a `pub(crate)` item that is
/// never used is not flagged. Kept rather than deleted because the
/// counter it feeds *is* displayed, so the gap is a missing call site
/// somewhere in the host-event path, not a redundant setter.
#[allow(dead_code)]
pub(crate) fn note_err(err: u32) {
    LAST_ERR.store(err, Ordering::Relaxed);
}

/// The state of the USB and network links, as the shared
/// [`link_bar`](mfsk_app_shared::ui::link_bar) draws it.
///
/// Built here rather than in each receiver's render loop: all three
/// need it, the mapping from `UacState` to what an operator should see
/// is a judgement (`Off` means two different things depending on
/// whether a host was ever installed), and three copies of a judgement
/// drift.
pub fn link_info() -> mfsk_app_shared::ui::link_bar::LinkInfo {
    use mfsk_app_shared::ui::link_bar::{LinkInfo, UsbLink};

    let (state, _sa, _rms) = status();
    let (devices, _clients, _events, _err) = usb_counters();
    let (tried_vbus, ..) = crate::pmic::power_state();
    // The role comes from what the firmware *decided*, which is exactly
    // "did it enable VBUS", not from whether the driver came up. A
    // board that chose host mode and has no host is a fault, and has to
    // read as one.
    let usb = if !host_installed() {
        if tried_vbus {
            UsbLink::NoHost
        } else {
            UsbLink::Peripheral
        }
    } else {
        match state {
            UacState::Streaming => UsbLink::Streaming,
            UacState::Error => UsbLink::Error,
            UacState::Off | UacState::Waiting => UsbLink::Waiting,
        }
    };
    let (tried, p0, p1, _st1) = crate::pmic::power_state();
    LinkInfo {
        usb,
        devices: devices.clamp(0, 9) as u8,
        wifi_rssi: mfsk_app_shared::wifi::rssi_cached(),
        expander_ok: crate::pmic::expander_ok(),
        battery_mv: crate::pmic::battery_mv_cached(),
        vbus_mv: crate::pmic::vbus_mv_cached(),
        clock_set: mfsk_app_shared::time_sync::utc_now_ms().is_some(),
        grid: mfsk_app_shared::time_sync::grid_lock(),
        grid_src: crate::grid_source(),
        vbus: tried.then(|| {
            (
                p1 & crate::board::AW9523_P1_BOOST_EN != 0,
                p0 & crate::board::AW9523_P0_USB_OTG_EN != 0,
                p0 & crate::board::AW9523_P0_BUS_OUT_EN != 0,
            )
        }),
    }
}

/// Bring the USB host up, once the things that make its failure
/// legible are in place.
///
/// **One sequence, for all three receivers.** `start_host` was always
/// shared; the steps around it were not, and the difference was not
/// deliberate — the FT8 controller waited for a log sink and installed
/// `esp_log_bridge` first, WSPR and FST4 called `start_host` straight
/// after enabling VBUS. So the two receivers that most needed the
/// enumeration trace could not produce one: `ENUM` is a C-side tag, and
/// without the bridge it goes to a console that host mode is about to
/// take away. A board that would not enumerate had, in those modes, no
/// way to say why.
///
/// **What is not here any more: a serviceability delay.** The FT8 path
/// slept `USB_HOST_DELAY_MS` (6 s) before installing, on the reasoning
/// that this was the last window in which a flasher could reach the
/// port. That was true when the firmware could take host mode with a
/// PC attached. Since #163 it decides from VBUS and only becomes a host
/// when nothing is powering the port — which means there is no PC, and
/// no flasher to wait for. Six seconds of nothing, on every boot,
/// guarding against a case that can no longer happen.
///
/// The wait that remains is for the UDP log sink, and it is bounded:
/// a receiver with no WiFi still has to start.
/// How long to wait for the UDP log sink before installing the host
/// anyway. A receiver with no WiFi still has to start.
///
/// **8 s, not 45.** The radio is not recognised until the host stack is
/// installed, so this wait is time the operator spends looking at a
/// board that has not noticed the IC-705 — reported from the bench as
/// exactly that. Measured on this network (2026-09-19): the sink comes
/// up 4.75-5.25 s after boot normally, and 24.5 s when the association
/// is cold, so 8 s keeps the usual case unchanged (WiFi still wins the
/// race, and its DMA buffers are still allocated before the host
/// stack's) while a slow association no longer costs the radio twenty
/// seconds.
///
/// What the wait was protecting is smaller than it looks: the one line
/// that matters, what `start_host` concluded, is kept in
/// [`HOST_RESULT`] and re-emitted by the panel once a sink exists,
/// whenever that is. The rest is the IDF's own enumeration chatter at
/// debug level.
const LOG_SINK_WAIT_MS: u32 = 8_000;

pub fn start_host_when_ready() {
    log::info!("uac: installing USB host — the serial console goes away when it returns");

    // Hold until the log sink exists, if it is coming.
    //
    // Everything interesting about enumeration is logged in the
    // few hundred milliseconds after `start_host()`, and on
    // battery there is no serial console to catch it — the VBUS
    // gate means a board in host mode is a board with no cable to
    // a PC. WiFi takes ~30 s, so without this wait those lines land
    // in the staging ring and are gone by the time anything can
    // read them. Bounded, because a receiver with no WiFi still has
    // to work.
    // Only worth waiting if a sink is actually coming. UAC mode
    // now leaves WiFi off by default, and waiting 45 s for a sink
    // that will never exist is just a receiver that takes 45 s
    // longer to start. Refs #163.
    let sink_expected = crate::wifi_enabled_for_this_boot();
    let mut waited_ms = 0u32;
    while sink_expected && waited_ms < LOG_SINK_WAIT_MS {
        if crate::FANOUT
            .udp
            .try_lock()
            .map(|g| g.is_some())
            .unwrap_or(false)
        {
            log::info!("uac: log sink up after {waited_ms} ms — installing USB host now");
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(250));
        waited_ms += 250;
    }
    if sink_expected && waited_ms >= LOG_SINK_WAIT_MS {
        log::warn!(
            "uac: no log sink after {waited_ms} ms — installing USB host anyway; \
             enumeration will only be visible on screen"
        );
    }

    // Turn up ESP-IDF's own USB enumeration logging before the
    // stack starts.
    //
    // The UAC class driver only reports what it recognises, so a
    // device that never finishes enumeration — or a hub whose
    // downstream ports are never walked — is indistinguishable
    // from nothing being plugged in. These tags are the ones the
    // host library uses on that path, and they are quiet outside
    // attach/detach, so leaving them up costs nothing while the
    // board waits. Issue #163.
    // `ENUM` alone at DEBUG. It carries the per-stage verdicts —
    // GET_FULL_DEV_DESC, CHECK_SHORT_CONFIG_DESC, and which one
    // FAILED — which is the whole diagnostic.
    //
    // The rest stay at INFO deliberately. At DEBUG, `EXT_PORT` /
    // `USBH` / `EXT_HUB` emit a "Processing actions" line per state
    // transition, so one attach is several hundred lines inside a
    // few milliseconds. Every one of those becomes a UDP datagram
    // sent from the USB task, and the board reliably fell off the
    // network right after enumeration whenever they were on — the
    // log volume was costing us the log. Refs #163.
    unsafe {
        esp_idf_svc::sys::esp_log_level_set(
            c"ENUM".as_ptr(),
            esp_idf_svc::sys::esp_log_level_t_ESP_LOG_DEBUG,
        );
    }
    for tag in [
        c"USB HOST".as_ptr(),
        c"USBH".as_ptr(),
        c"HUB".as_ptr(),
        c"EXT_HUB".as_ptr(),
        c"EXT_PORT".as_ptr(),
    ] {
        unsafe {
            esp_idf_svc::sys::esp_log_level_set(
                tag,
                esp_idf_svc::sys::esp_log_level_t_ESP_LOG_INFO,
            );
        }
    }

    crate::log_free_internal("pre-uac-host-install");
    // Serial is about to go away with the PHY; move the C-side log
    // output somewhere that survives (see `esp_log_bridge`).
    crate::esp_log_bridge::install();

    if let Err(e) = start_host() {
        log::error!("UAC host start failed: {e:#}");
        let mut msg: heapless::String<96> = heapless::String::new();
        {
            use core::fmt::Write as _;
            let _ = write!(&mut msg, "start_host FAILED: {e:#}");
        }
        HOST_RESULT.store(msg.as_str());
    }
    crate::log_free_internal("post-uac-host-install");
}
