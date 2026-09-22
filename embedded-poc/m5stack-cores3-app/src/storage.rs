//! The files this board keeps: `qso.adi` (the activator's contact log)
//! and `all.txt` (every decode and transmission, WSJT-X's `ALL.TXT`
//! layout), on the `littlefs` partition, and the one task that touches
//! them.
//!
//! **Why a task of its own.** A flash write or read disables the flash
//! cache, which also unmaps PSRAM, so ESP-IDF asserts that the caller's
//! stack is in internal DRAM (`esp_task_stack_is_sane_cache_disabled`,
//! `cache_utils.c:127`) — and aborts the board if not. The HTTP server
//! and the panel tasks run on PSRAM stacks on purpose, to leave internal
//! DRAM to the decoder; `boot_mode::commit_and_restart` is where this
//! was learned. Every read and write therefore goes over a channel to
//! [`TASK_NAME`], whose stack is internal DRAM by explicit caps.
//! Callers never block on the flash: appends are
//! fire-and-forget, and a full queue drops the batch and counts it
//! ([`dropped`]) rather than stall a decoder at its reply deadline.
//!
//! LittleFS's own buffers are put in PSRAM
//! (`CONFIG_LITTLEFS_MALLOC_STRATEGY_SPIRAM`); `esp_flash` copies
//! through an internal bounce buffer when the source or destination is
//! external, so that costs speed, not correctness.
//!
//! **Not yet measured**: what a sector erase (~45 ms typical on this
//! class of flash, one per 4 KB appended) does to the UAC capture, whose
//! isochronous URBs hold 48 ms of audio (`embedded-poc/CLAUDE.md`, "The
//! isochronous URB budget"). The cache is off for the erase on both
//! cores, and the URB resubmit runs from flash. Check the delivered
//! sample rate with logging on before trusting a long session.

use std::io::{Read, Seek, SeekFrom, Write};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::sync::Mutex;
use std::sync::OnceLock;
use std::time::Duration;

use mfsk_app_shared::boot_mode::BootMode;
use mfsk_app_shared::ui::state::{SlotDecode, UiState};

const MOUNT: &str = "/littlefs";
const PARTITION: &core::ffi::CStr = c"littlefs";
const TASK_NAME: &core::ffi::CStr = c"storage";
/// Measured high-water mark wanted from `board::log_task_stacks` before
/// this is trusted; the frames are LittleFS's path walk plus newlib's
/// VFS, neither of them deep.
const TASK_STACK: usize = 5120;
/// Below the decoder and the audio path: the flash op stalls both cores
/// whoever issues it, so the only thing priority buys is *when*.
const TASK_PRIO: u8 = 2;
/// Queued batches. A slot is one batch, so this is several slots of
/// slack for a stuck flash before anything is dropped.
const QUEUE: usize = 8;

/// `all.txt` rotates to `all.1.txt` past this; `qso.adi` never rotates.
/// Two of them is 4 MiB of the 6.125 MiB partition — roughly a day and
/// a half of a busy band at ~120 KB/h.
const ALL_MAX_BYTES: u64 = 2 * 1024 * 1024;

/// A file this module serves. The names are the HTTP paths too.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum File {
    Qso,
    All,
    AllOld,
}

impl File {
    pub const ALL_FILES: [File; 3] = [File::Qso, File::All, File::AllOld];

    pub fn name(self) -> &'static str {
        match self {
            File::Qso => "qso.adi",
            File::All => "all.txt",
            File::AllOld => "all.1.txt",
        }
    }

    pub fn from_name(name: &str) -> Option<File> {
        Self::ALL_FILES.into_iter().find(|f| f.name() == name)
    }

    fn path(self) -> String {
        format!("{MOUNT}/{}", self.name())
    }
}

enum Req {
    /// Lines for `all.txt`, and the slot period they came from — the
    /// write waits for the middle of the current slot (see [`serve`]).
    AppendAll { text: String, period_ms: u64 },
    /// A record, and the header to write first if the file is new.
    AppendQso { record: String, header: String },
    Read {
        file: File,
        offset: u64,
        len: usize,
        reply: SyncSender<Option<Vec<u8>>>,
    },
}

static TX: OnceLock<SyncSender<Req>> = OnceLock::new();
/// The receiving end, from the first request until
/// [`start_if_requested`] hands it to the task.
static PENDING_RX: Mutex<Option<Receiver<Req>>> = Mutex::new(None);
static MOUNTED: AtomicBool = AtomicBool::new(false);
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Batches dropped because the queue was full or the filesystem is not
/// mounted.
pub fn dropped() -> u32 {
    DROPPED.load(Ordering::Relaxed)
}

pub fn mounted() -> bool {
    MOUNTED.load(Ordering::Acquire)
}

static ENABLED: AtomicBool = AtomicBool::new(false);

/// Allow the storage task to exist; it is spawned after the first
/// request, by [`start_if_requested`].
///
/// **Why lazily.** Spawned from `boot::run`, its 5 KB internal stack
/// was carved out of the block the decoder allocates from next: the
/// largest free internal block before the decode loop fell from
/// 40 960 B to 32 768 B, and with it FT8 went from 0 of 18 SIM slots
/// past key-up to 9-14 of 18 (2026-09-22, `logs/storage_off_ft8sim_*`
/// against `logs/storage_midslot_ft8sim_*`). The first request comes
/// from the first published slot, by which time the decoder has what
/// it needs.
///
/// The channel is made here, on the booting task, and not by the first
/// request: building an `mpsc` channel assembles its state on the
/// caller's stack before boxing it, and the first caller is a decoder.
/// Made by the first request, it left `ft4_slot` (8 KB) 444-464 B free in
/// FT4 SIM runs against 732 B with storage off (2026-09-22,
/// `logs/ft4sim_storage_*_2026-09-22.log`).
pub fn enable() {
    ENABLED.store(true, Ordering::Release);
    let _ = ensure_channel();
}

/// The channel: made once, by [`enable`].
fn ensure_channel() -> Option<&'static SyncSender<Req>> {
    if !ENABLED.load(Ordering::Acquire) {
        return None;
    }
    Some(TX.get_or_init(|| {
        let (tx, rx) = sync_channel::<Req>(QUEUE);
        if let Ok(mut p) = PENDING_RX.lock() {
            *p = Some(rx);
        }
        tx
    }))
}

/// Spawn the storage task once something has asked for it. Called by
/// the panel loop every frame; a no-op after the first spawn.
///
/// **Not from the requester**, which is a decoder: a thread spawn is
/// not a shallow call, and a decoder's stack is sized to its decode.
/// The panel's `main` task has ~10 KB free. Requests made before the
/// spawn wait in the channel, [`QUEUE`] deep.
pub fn start_if_requested() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    if ONCE.is_completed() || TX.get().is_none() {
        return;
    }
    ONCE.call_once(|| {
        let rx = PENDING_RX.lock().ok().and_then(|mut p| p.take());
        if let Some(rx) = rx {
            spawn(rx);
        }
    });
}

/// Spawn the storage task; it mounts the partition (formatting it the
/// first time) before taking requests.
///
/// A `std::thread` with its stack caps set to internal DRAM explicitly
/// — `uac::spawn_psram_thread`'s mechanism with the opposite caps — so
/// the channel below parks a pthread, as `std` expects.
fn spawn(rx: Receiver<Req>) {
    use esp_idf_svc::hal::cpu::Core;
    use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};

    let d = ThreadSpawnConfiguration::default();
    let cfg = ThreadSpawnConfiguration {
        name: Some(TASK_NAME),
        stack_size: TASK_STACK,
        stack_alloc_caps: MallocCap::Internal | MallocCap::Cap8bit,
        priority: TASK_PRIO,
        pin_to_core: Some(Core::Core1),
        ..d
    };
    if let Err(e) = cfg.set() {
        log::error!("storage: spawn config failed ({e:?}) — no logs this boot");
        return;
    }
    let spawned = std::thread::Builder::new()
        .stack_size(TASK_STACK)
        .spawn(move || {
            if mount() {
                MOUNTED.store(true, Ordering::Release);
            }
            let mut all = AllTxt::default();
            let ok = mounted();
            for req in rx.iter() {
                if ok {
                    serve(req, &mut all);
                } else {
                    DROPPED.fetch_add(1, Ordering::Relaxed);
                }
            }
        });
    let _ = ThreadSpawnConfiguration::default().set();
    if let Err(e) = spawned {
        log::error!("storage: could not spawn the task ({e}) — no logs this boot");
    }
}

fn mount() -> bool {
    use esp_idf_svc::sys;
    let mut conf = sys::esp_vfs_littlefs_conf_t {
        base_path: c"/littlefs".as_ptr(),
        partition_label: PARTITION.as_ptr(),
        ..Default::default()
    };
    conf.set_format_if_mount_failed(1);
    // SAFETY: `conf` and the strings it points at outlive the call; the
    // component copies what it keeps.
    let r = unsafe { sys::esp_vfs_littlefs_register(&conf) };
    if r != sys::ESP_OK {
        log::error!("storage: mount {MOUNT} failed ({r}) — no logs this boot");
        return false;
    }
    let (mut total, mut used) = (0usize, 0usize);
    // SAFETY: two valid out-pointers.
    unsafe { sys::esp_littlefs_info(PARTITION.as_ptr(), &mut total, &mut used) };
    log::info!(
        "storage: {MOUNT} mounted, {} of {} KB used",
        used / 1024,
        total / 1024
    );
    true
}

fn serve(req: Req, all: &mut AllTxt) {
    match req {
        Req::AppendAll { text, period_ms } => {
            // Only a write that will reach the flash needs a quiet moment.
            if all.pending.len() + text.len() >= ALL_FLUSH_BYTES {
                wait_for_mid_slot(period_ms);
            }
            let t0 = std::time::Instant::now();
            match all.append(&text) {
                Ok((0, _)) => {}
                Ok((written, size)) => {
                    // What an append costs, erase included: the number
                    // that decides whether the flash stall is safe for
                    // the UAC capture's 48 ms of queued audio.
                    let ms = t0.elapsed().as_millis() as u32;
                    let (w, s) = all.last_split_us;
                    // Where in the UTC slot the flash was busy: a write
                    // stops the cache, and with it the USB host's
                    // resubmissions (`uac::RX_DONE_GAP_MAX_US`).
                    let at = mfsk_app_shared::time_sync::utc_now_ms()
                        .map_or(-1, |t| (t % (period_ms.max(1))) as i64 - ms as i64);
                    log::info!(
                        "storage: all.txt +{} B in {ms} ms (write {} ms, sync {} ms) from slot +{at} ms, now {} KB",
                        written,
                        w / 1000,
                        s / 1000,
                        size / 1024
                    );
                }
                Err(e) => log::warn!("storage: all.txt append failed: {e}"),
            }
        }
        Req::AppendQso { record, header } => match write_qso(&record, &header) {
            Ok(()) => log::info!("storage: qso.adi += {}", record.trim_end()),
            // Loud: this is the contact log, not a convenience.
            Err(e) => log::error!(
                "storage: qso.adi append FAILED: {e} — {}",
                record.trim_end()
            ),
        },
        Req::Read {
            file,
            offset,
            len,
            reply,
        } => {
            if matches!(file, File::All) {
                if let Err(e) = all.flush() {
                    log::warn!("storage: all.txt flush before read failed: {e}");
                }
            }
            let _ = reply.send(read(file, offset, len).ok());
        }
    }
}

/// Sleep until the middle of the current slot.
///
/// **Why.** A flash write stalls both cores while the cache is off.
/// Issued the moment a slot's decodes were published — ~0.4 s after the
/// slot ended — it lands in the stretch where the next slot's first
/// pass is racing key-up, and the A/B on 2026-09-22 (FT8 SIM, 18 slots
/// each, `logs/storage_off_ft8sim_2026-09-22.log` against
/// `logs/storage_ft8sim_http_2026-09-22.log`) put a number on it: with
/// the writes, 9-12 of 18 slots finished 21-117 ms past key-up and
/// `post_slotend` rose ~100 ms; without them, none did. Mid-slot is
/// the point furthest from every deadline — capture is running, and
/// capture is buffered.
fn wait_for_mid_slot(period_ms: u64) {
    let Some(now) = mfsk_app_shared::time_sync::utc_now_ms() else {
        return;
    };
    if period_ms == 0 {
        return;
    }
    let target = period_ms / 2;
    let phase = now % period_ms;
    let wait = (target + period_ms - phase) % period_ms;
    std::thread::sleep(Duration::from_millis(wait));
}

/// Bytes of `all.txt` lines held in memory before they are written.
///
/// **Why batch.** LittleFS cannot program more into a block after a
/// `sync` has committed it, so every synced append copies the file's
/// tail block to a freshly erased one — the bytes already in that block
/// are written again each time. Traced on the CoreS3 (2026-09-22,
/// `MFSK_STORAGE_TRACE`, `logs/storage_trace_ft8sim_2026-09-22.log`),
/// a 448 B slot append cost one 4 KB erase plus 512 B to 4 096 B of
/// programming, growing with the block (7-37 ms), and roughly one
/// `sync` in ten also compacted the metadata log: 538 reads, 69 KB,
/// one erase, **147 ms**. One block's worth per write bounds the copy
/// at one block, and one `sync` per ~9 FT8 slots makes the compaction
/// that much rarer. The price is what a power cut takes: the unwritten
/// ~2 minutes of `all.txt`. `qso.adi` is still synced per record.
const ALL_FLUSH_BYTES: usize = 4096;

/// `all.txt`: lines buffered until [`ALL_FLUSH_BYTES`], then appended
/// through a handle held open between writes.
#[derive(Default)]
struct AllTxt {
    open: Option<(std::fs::File, u64)>,
    pending: String,
    /// `(write, sync)` of the last flush, µs — where the time goes.
    last_split_us: (u32, u32),
}

impl AllTxt {
    /// Buffer `text`; write if a block's worth is waiting. Returns the
    /// bytes written now (0 if only buffered) and the file size after.
    fn append(&mut self, text: &str) -> std::io::Result<(usize, u64)> {
        self.pending.push_str(text);
        if self.pending.len() < ALL_FLUSH_BYTES {
            return Ok((0, self.open.as_ref().map_or(0, |(_, n)| *n)));
        }
        self.flush()
    }

    /// Write whatever is buffered — before a read, so a download is
    /// current.
    fn flush(&mut self) -> std::io::Result<(usize, u64)> {
        if self.pending.is_empty() {
            return Ok((0, self.open.as_ref().map_or(0, |(_, n)| *n)));
        }
        let path = File::All.path();
        if self.open.is_none() {
            let size = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
            let f = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&path)?;
            self.open = Some((f, size));
        }
        let size = self.open.as_ref().map_or(0, |(_, n)| *n);
        if size + self.pending.len() as u64 > ALL_MAX_BYTES {
            self.open = None; // close before the rename
            let old = File::AllOld.path();
            let _ = std::fs::remove_file(&old);
            std::fs::rename(&path, &old)?;
            log::info!("storage: all.txt rotated at {size} B");
            return self.flush();
        }
        let text = core::mem::take(&mut self.pending);
        let (f, n) = self.open.as_mut().expect("opened above");
        let t0 = std::time::Instant::now();
        #[cfg(storage_trace)]
        let s0 = trace::snapshot();
        let r = f.write_all(text.as_bytes()).and_then(|()| {
            let t1 = std::time::Instant::now();
            #[cfg(storage_trace)]
            let s1 = trace::snapshot();
            let r = f.sync_all();
            #[cfg(storage_trace)]
            log::info!(
                "storage: trace write: {}| sync: {}",
                trace::delta(s0, s1),
                trace::delta(s1, trace::snapshot())
            );
            self.last_split_us = (
                (t1 - t0).as_micros() as u32,
                t1.elapsed().as_micros() as u32,
            );
            r
        });
        if let Err(e) = r {
            // Reopen next time rather than keep writing through a handle
            // in an unknown state; the lines are lost, and said so.
            self.open = None;
            return Err(e);
        }
        *n += text.len() as u64;
        Ok((text.len(), *n))
    }
}

fn write_qso(record: &str, header: &str) -> std::io::Result<()> {
    let path = File::Qso.path();
    let new = std::fs::metadata(&path).map_or(true, |m| m.len() == 0);
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)?;
    if new {
        f.write_all(header.as_bytes())?;
    }
    f.write_all(record.as_bytes())?;
    // The contact is on flash before the next transmission, which is the
    // guarantee a battery-powered log has to make.
    f.sync_all()
}

fn read(file: File, offset: u64, len: usize) -> std::io::Result<Vec<u8>> {
    let mut f = std::fs::File::open(file.path())?;
    f.seek(SeekFrom::Start(offset))?;
    let mut buf = vec![0u8; len];
    let mut n = 0;
    while n < len {
        match f.read(&mut buf[n..])? {
            0 => break,
            k => n += k,
        }
    }
    buf.truncate(n);
    Ok(buf)
}

fn send(req: Req) -> bool {
    // Queued even before the task exists: it mounts first and then
    // drains, and drops (and counts) what it cannot write.
    let ok = ensure_channel().is_some_and(|tx| tx.try_send(req).is_ok());
    if !ok {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
    ok
}

/// Append lines (each ending in `\n`) to `all.txt`.
pub fn append_all_txt(text: String, period_ms: u64) {
    if !text.is_empty() {
        send(Req::AppendAll { text, period_ms });
    }
}

/// Append one ADIF record to `qso.adi`, writing `header` first if the
/// file is new. Returns `false` if it could not even be queued — the
/// caller should say so on the panel, since a contact log that silently
/// misses a contact is worse than none.
pub fn append_qso(record: String, header: String) -> bool {
    send(Req::AppendQso { record, header })
}

/// Up to `len` bytes of `file` from `offset`; `Some(empty)` at the end,
/// `None` if the file is absent or the task did not answer. For the
/// HTTP download handlers, which run on a PSRAM stack and so must not
/// read the flash themselves.
pub fn read_chunk(file: File, offset: u64, len: usize) -> Option<Vec<u8>> {
    let tx = ensure_channel()?;
    let (reply, rx) = sync_channel(1);
    tx.send(Req::Read {
        file,
        offset,
        len,
        reply,
    })
    .ok()?;
    rx.recv_timeout(Duration::from_secs(5)).ok().flatten()
}

/// The files, as the HTTP server serves them.
pub const HTTP_FILES: mfsk_app_shared::http_config::FileSource =
    mfsk_app_shared::http_config::FileSource {
        names: &["qso.adi", "all.txt", "all.1.txt"],
        read: |name, offset, len| read_chunk(File::from_name(name)?, offset, len),
    };

/// The UTC second the slot just decoded began, for a decode published
/// after its slot ended and before the next one did — true of every
/// receiver here at its publish point. `None` while the clock is unset:
/// a line stamped 1970 is worse than no line.
pub fn decoded_slot_unix(period_ms: u64) -> Option<i64> {
    let now = mfsk_app_shared::time_sync::utc_now_ms()?;
    let start_ms = (now / period_ms).checked_sub(1)? * period_ms;
    Some((start_ms / 1000) as i64)
}

/// Publish one slot — **the one call every receiver makes**: the rows
/// go to the station list and, from the same values, to `ALL.TXT`.
///
/// FT8, FT4, WSPR and FST4 each used to call `UiState::publish_slot`
/// and then build their own `ALL.TXT` tuples beside it, four times
/// over. The mode name and slot period come from the boot mode, and
/// the dial frequency from the status bar's `rig_freq_hz`, so what the
/// panel shows and what the log records cannot disagree — and a CAT
/// link that sets the status field puts the frequency into every
/// mode's log at once.
///
/// Takes the locked [`UiState`] because FT8 publishes from inside the
/// lock it decodes under. The `ALL.TXT` half only formats and queues;
/// the flash write happens on the storage task.
///
/// `slot_unix` is the UTC second the slot began, `None` while the
/// clock is unset (the log is skipped: a line stamped 1970 is worse
/// than no line). [`decoded_slot_unix`] gives it for receivers that
/// publish in the slot after the one decoded.
pub fn publish_slot(
    ui: &mut UiState,
    mode: BootMode,
    slot_unix: Option<i64>,
    decodes: &[SlotDecode<'_>],
) {
    ui.publish_slot(decodes.iter().copied());
    let Some(t) = slot_unix else { return };
    let name = mfsk_app_shared::ui::mode_picker::mode_name(mode).unwrap_or("?");
    let dial_hz = ui.status.rig_freq_hz.map(u64::from);
    let mut text = String::new();
    for d in decodes {
        let snr = if d.snr_db.is_finite() {
            d.snr_db.round() as i32
        } else {
            0
        };
        text.push_str(&mfsk_app_shared::all_txt::rx_line(
            t,
            dial_hz,
            name,
            snr,
            d.dt_sec,
            d.freq_hz.round() as i32,
            d.text,
        ));
    }
    append_all_txt(text, mode.slot_period_ms() as u64);
}

/// Counting wrappers around the flash calls LittleFS makes
/// (`littlefs_esp_part.c`: read, write, erase_range), linked in with
/// `-Wl,--wrap` when built with `MFSK_STORAGE_TRACE=1`. Everything
/// that touches a partition goes through them — NVS too — but in steady
/// state the storage task is the only writer, and [`Snapshot`] deltas
/// are taken around its own calls.
#[cfg(storage_trace)]
mod trace {
    use core::ffi::c_void;
    use core::sync::atomic::{AtomicU32, Ordering};
    use esp_idf_svc::sys::{esp_err_t, esp_partition_t, esp_timer_get_time};

    static N: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];
    static US: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];
    static BYTES: [AtomicU32; 3] = [AtomicU32::new(0), AtomicU32::new(0), AtomicU32::new(0)];

    fn record(i: usize, t0: i64, size: usize) {
        // SAFETY: no arguments.
        let dt = (unsafe { esp_timer_get_time() } - t0) as u32;
        N[i].fetch_add(1, Ordering::Relaxed);
        US[i].fetch_add(dt, Ordering::Relaxed);
        BYTES[i].fetch_add(size as u32, Ordering::Relaxed);
    }

    extern "C" {
        fn __real_esp_partition_read(
            p: *const esp_partition_t,
            off: usize,
            dst: *mut c_void,
            size: usize,
        ) -> esp_err_t;
        fn __real_esp_partition_write(
            p: *const esp_partition_t,
            off: usize,
            src: *const c_void,
            size: usize,
        ) -> esp_err_t;
        fn __real_esp_partition_erase_range(
            p: *const esp_partition_t,
            off: usize,
            size: usize,
        ) -> esp_err_t;
    }

    #[no_mangle]
    unsafe extern "C" fn __wrap_esp_partition_read(
        p: *const esp_partition_t,
        off: usize,
        dst: *mut c_void,
        size: usize,
    ) -> esp_err_t {
        let t0 = esp_timer_get_time();
        let r = __real_esp_partition_read(p, off, dst, size);
        record(0, t0, size);
        r
    }

    #[no_mangle]
    unsafe extern "C" fn __wrap_esp_partition_write(
        p: *const esp_partition_t,
        off: usize,
        src: *const c_void,
        size: usize,
    ) -> esp_err_t {
        let t0 = esp_timer_get_time();
        let r = __real_esp_partition_write(p, off, src, size);
        record(1, t0, size);
        r
    }

    #[no_mangle]
    unsafe extern "C" fn __wrap_esp_partition_erase_range(
        p: *const esp_partition_t,
        off: usize,
        size: usize,
    ) -> esp_err_t {
        let t0 = esp_timer_get_time();
        let r = __real_esp_partition_erase_range(p, off, size);
        record(2, t0, size);
        r
    }

    /// Counts, µs and bytes for read / write / erase.
    #[derive(Clone, Copy)]
    pub struct Snapshot([(u32, u32, u32); 3]);

    pub fn snapshot() -> Snapshot {
        let mut s = [(0, 0, 0); 3];
        for (i, e) in s.iter_mut().enumerate() {
            *e = (
                N[i].load(Ordering::Relaxed),
                US[i].load(Ordering::Relaxed),
                BYTES[i].load(Ordering::Relaxed),
            );
        }
        Snapshot(s)
    }

    /// `read 12x/1536B/3ms prog 4x/640B/2ms erase 1x/4096B/41ms`.
    pub fn delta(a: Snapshot, b: Snapshot) -> String {
        let name = ["read", "prog", "erase"];
        let mut out = String::new();
        for i in 0..3 {
            let (n, us, by) = (
                b.0[i].0.wrapping_sub(a.0[i].0),
                b.0[i].1.wrapping_sub(a.0[i].1),
                b.0[i].2.wrapping_sub(a.0[i].2),
            );
            out.push_str(&format!(
                "{} {n}x/{by}B/{:.1}ms ",
                name[i],
                us as f32 / 1000.0
            ));
        }
        out
    }
}
