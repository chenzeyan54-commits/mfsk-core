// SPDX-License-Identifier: GPL-3.0-or-later
//! The CoreS3 FT4 per-slot decode, reproduced call for call on the host.
//!
//! `ft4_ddc_equivalence` compares *front ends* — one candidate list, two
//! ways of building `cd0` — over the whole 90 000-sample slot. That is
//! not the receiver. The receiver
//! (`embedded-poc/embedded-shared/src/apps/ft4_rx.rs`) closes its
//! capture window early at `CAPTURE_CLOSE_SAMPLES = 81_300`, builds its
//! periodogram incrementally as audio arrives, decimates by two in the
//! same pass, searches a narrowed `WSJTX_WINDOW`, and shares one set of
//! coarse phasor tables across every candidate. Each of those changes
//! which candidates exist and where they land, so a number measured on
//! the other shape does not transfer.
//!
//! This file follows `decode_slot` instead, with the FreeRTOS plumbing
//! folded away:
//!
//! ```text
//! per 256-sample block   Ft4SavgBuilder::push_with_rows   (137 rows / slot)
//!                        SlotDecimator::push_i16          -> half, 6 kHz
//! at CAPTURE_CLOSE       savg.finish()
//!                        ft4_coarse_sync_from_savg(FREQ_MIN..MAX, SYNC_MIN, MAX_CAND)
//!                        Ft4CoarsePhasors::new::<Ft4>()   (once per slot)
//! per candidate          candidate_baseband_half -> rms_normalise
//!                        ft4_sync_search_window_with::<Ft4>(WSJTX_WINDOW, refs)
//!                        process_candidate_precomputed::<Ft4>(EMBEDDED, SYNC_Q_MIN)
//!                        unpack77
//! ```
//!
//! **What it does not reproduce**, deliberately, and what the board is
//! therefore still the only instrument for:
//!
//! - the deadline and its cut (`TX_TURNAROUND_BUDGET_MS`); every
//!   candidate runs here;
//! - the two-core split, so nothing here sees the IDF heap lock two
//!   cores contend for;
//! - PSRAM, the internal-DRAM threshold and the silent fallback between
//!   them, and the PIE 16-byte alignment that
//!   `docs/notes/FT4_BENCHMARK.md` §32.1 measured at 1 049 -> 1 789 ms;
//! - the slot grid, which on the board decides the phase this file
//!   takes as given.
//!
//! What it *is* the instrument for is everything computational: which
//! messages come out, and — through the counting allocator below — how
//! many allocations and bytes a candidate costs. Those questions do not
//! need a board, and answering them on one costs a build, a capture
//! window and a grid that has to re-anchor.
//!
//! ```sh
//! cargo test -p mfsk-core --release --features full,internal-testing \
//!     --test ft4_embedded_pipeline_mirror -- --nocapture
//! ```
#![cfg(all(
    feature = "ft4",
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use num_complex::Complex;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::{Ft4SavgBuilder, ft4_coarse_sync_from_savg};
use mfsk_core::engine::pipeline::{
    DecodeDepth, DecodeResult, DecodeStrictness, process_candidate_precomputed,
};
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::engine::sync2d::{Ft4CoarsePhasors, ft4_sync_search_window_with};
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::ddc::{SlotDecimator, candidate_baseband_half};
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

// ── the counting allocator ──────────────────────────────────────────
//
// An integration test is its own binary, so this instruments this file
// and nothing else. It is what turns "the candidate loop allocates ~50
// times" from a code reading into a number a change has to move.

static ALLOCS: AtomicUsize = AtomicUsize::new(0);
static ALLOC_BYTES: AtomicUsize = AtomicUsize::new(0);

struct Counting;

// SAFETY: every method forwards to `System` unchanged; the counters are
// `Relaxed` adds that cannot affect the pointer returned.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        ALLOC_BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        // A realloc is an allocation as far as the board's heap lock is
        // concerned, so count it as one.
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        ALLOC_BYTES.fetch_add(new_size, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// `(allocations, bytes)` since boot. Both monotonic; a caller reads the
/// pair around a region and reports the difference.
fn alloc_census() -> (usize, usize) {
    (
        ALLOCS.load(Ordering::Relaxed),
        ALLOC_BYTES.load(Ordering::Relaxed),
    )
}

// ── the receiver's own constants ────────────────────────────────────
//
// Every one of these is a literal in `ft4_rx.rs`. They are repeated
// rather than imported because `embedded-shared` is outside this
// workspace and needs the `+esp` toolchain; `mirror_constants_match_the_receiver`
// below is what keeps the copy honest.

/// `ft4_rx.rs`'s `SLOT_SAMPLES`.
const SLOT_SAMPLES: usize = 90_000;
/// `ft4_rx.rs`'s `CAPTURE_CLOSE_SAMPLES` — 6.775 s, where the receiver
/// stops taking audio because no candidate can reach past it.
const CAPTURE_CLOSE_SAMPLES: usize = 81_300;
const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 2_700.0;
const SYNC_MIN: f32 = 1.2;
const MAX_CAND: usize = 100;
/// `ft4::decode`'s own private `SYNC_Q_MIN`.
const SYNC_Q_MIN: u32 = 8;
/// `ft4_rx.rs`'s `WSJTX_WINDOW`, in `cd0` samples at 666.667 Hz.
const WSJTX_WINDOW: (i32, i32) = (-344, 1012);
/// The block size a UAC read produces, which is the cadence the board
/// feeds `SlotAccum` at. Block-independence is pinned elsewhere
/// (`slot_decimator_is_block_independent`); using the real one here
/// keeps this mirror a mirror.
const BLOCK: usize = 256;

fn slot_audio() -> Option<Vec<i16>> {
    let path = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    )?;
    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

/// The RMS normalisation `process_candidate_basic_impl` applies to a
/// `cd0` it builds itself (WSJT-X `ft4_decode.f90:231-232`).
/// `candidate_baseband_half` deliberately does not, so the receiver
/// does it — `ft4_rx.rs`'s `rms_normalise` — and so does this.
fn rms_normalise(cd0: &mut [Complex<f32>]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}

/// What one slot produced, and what it cost to produce.
struct SlotRun {
    messages: Vec<String>,
    cands: usize,
    /// Allocations and bytes inside the candidate loop only — the
    /// figure L1 exists to drive to zero.
    cand_allocs: usize,
    cand_bytes: usize,
    /// The same, for the capture side: everything before the window
    /// closes.
    capture_allocs: usize,
    capture_bytes: usize,
}

/// `SlotAccum::push_with_rows` + `decode_slot`, in one function.
fn run_slot(audio: &[i16]) -> SlotRun {
    // ── capture side ────────────────────────────────────────────────
    let c0 = alloc_census();
    let mut savg = Ft4SavgBuilder::new(CAPTURE_CLOSE_SAMPLES);
    let mut decim = SlotDecimator::new();
    let mut half: Vec<f32> = Vec::with_capacity(CAPTURE_CLOSE_SAMPLES / 2 + 64);
    let mut fed = 0usize;
    while fed < CAPTURE_CLOSE_SAMPLES {
        let take = BLOCK.min(CAPTURE_CLOSE_SAMPLES - fed);
        let block = &audio[fed..fed + take];
        savg.push_with_rows(block, &mut |_row| {});
        decim.push_i16(block, &mut half);
        fed += take;
    }
    let savg = savg.finish();
    let c1 = alloc_census();

    // ── post-close, candidate-independent ───────────────────────────
    let cands =
        ft4_coarse_sync_from_savg(&savg, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
    let refs = Ft4CoarsePhasors::new::<Ft4>();

    // ── the candidate loop ──────────────────────────────────────────
    let c2 = alloc_census();
    let mut messages: Vec<String> = Vec::new();
    for cand in &cands {
        if let Some(text) = decode_candidate(&half, cand, &refs)
            && !messages.contains(&text)
        {
            messages.push(text);
        }
    }
    let c3 = alloc_census();

    SlotRun {
        messages,
        cands: cands.len(),
        cand_allocs: c3.0 - c2.0,
        cand_bytes: c3.1 - c2.1,
        capture_allocs: c1.0 - c0.0,
        capture_bytes: c1.1 - c0.1,
    }
}

/// `ft4_rx::decode_candidate`, call for call.
fn decode_candidate(half: &[f32], cand: &SyncCandidate, refs: &Ft4CoarsePhasors) -> Option<String> {
    let mut cd0 = candidate_baseband_half(half, cand.freq_hz);
    rms_normalise(&mut cd0);
    let s2 = ft4_sync_search_window_with::<Ft4>(&cd0, cand, WSJTX_WINDOW.0, WSJTX_WINDOW.1, refs);
    let r: DecodeResult = process_candidate_precomputed::<Ft4>(
        cand,
        // FT4's `snr_db` reads the coarse candidate score, not a
        // wide-band cache, so the board passes an empty slice here and
        // so does this.
        &[],
        &FT4_DOWNSAMPLE,
        DecodeDepth::EMBEDDED,
        DecodeStrictness::Normal,
        &[],
        EqMode::Off,
        SYNC_Q_MIN,
        (cd0, s2.freq_hz, s2.i0, s2.score),
        false,
        false,
    )?;
    let m77: [u8; 77] = r.message77().try_into().ok()?;
    unpack77(&m77)
}

fn require_corpus() -> bool {
    std::env::var("MFSK_REQUIRE_CORPUS").is_ok()
}

/// The mirror's own gate: the receiver's arrangement decodes the golden,
/// and the candidate loop's allocation cost is on the record.
///
/// The decode count is an equality, not a floor. A front-end or
/// scheduling change that moves it is exactly what this file exists to
/// catch, and "more" is as much a change as "fewer" — a phantom is a
/// decode too.
#[test]
fn mirror_decodes_the_golden_and_reports_what_a_candidate_allocates() {
    let Some(audio) = slot_audio() else {
        assert!(
            !require_corpus(),
            "MFSK_REQUIRE_CORPUS=1 but the FT4 golden recording is missing"
        );
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };

    let run = run_slot(&audio);

    let per_cand_allocs = run.cand_allocs as f64 / run.cands.max(1) as f64;
    let per_cand_bytes = run.cand_bytes as f64 / run.cands.max(1) as f64;
    eprintln!(
        "ft4 mirror: {} candidates, {} decodes\n  \
         capture side : {} allocations, {} B\n  \
         candidate loop: {} allocations, {} B  ({:.1}/cand, {:.0} B/cand)",
        run.cands,
        run.messages.len(),
        run.capture_allocs,
        run.capture_bytes,
        run.cand_allocs,
        run.cand_bytes,
        per_cand_allocs,
        per_cand_bytes,
    );
    let mut sorted = run.messages.clone();
    sorted.sort();
    for m in &sorted {
        eprintln!("    {m}");
    }

    assert_eq!(
        run.messages.len(),
        11,
        "the receiver's arrangement decodes 11 on this recording; got {:?}",
        sorted
    );
}

/// The constants above are a copy of `ft4_rx.rs`'s, and a copy rots.
///
/// This cannot import them — `embedded-shared` is outside this
/// workspace and builds only under the `+esp` toolchain — so it checks
/// them against the crate-side facts they are derived from instead.
#[test]
fn mirror_constants_match_the_receiver() {
    use mfsk_core::engine::{FrameLayout, ModulationParams};

    // 7.5 s at 12 kHz.
    assert_eq!(SLOT_SAMPLES, 90_000);
    // The window has to cover the frame's own length plus the +1.0 s of
    // DT `WSJTX_WINDOW` allows plus the DDC's group delay, and stop
    // before the slot ends — that is the whole reason it is 81 300 and
    // not 90 000.
    const { assert!(CAPTURE_CLOSE_SAMPLES < SLOT_SAMPLES) };
    let frame_samples = (Ft4::N_SYMBOLS as usize) * (Ft4::NSPS as usize);
    assert!(
        CAPTURE_CLOSE_SAMPLES > frame_samples + 12_000,
        "the capture window must still hold a frame at DT = +1.0 s"
    );
    // `WSJTX_WINDOW` is in `cd0` samples; upstream's own bounds are
    // `ibmin = -344`, `ibmax = 1012` (`lib/ft4_decode.f90:241-242`).
    assert_eq!(WSJTX_WINDOW, (-344, 1012));
    // The search must stay inside the buffer the DDC produces.
    let cd0_len = mfsk_core::ft4::ddc::CD0_LEN as i32;
    let frame_cd0 = (Ft4::N_SYMBOLS * Ft4::NSPS / Ft4::NDOWN) as i32;
    assert!(WSJTX_WINDOW.1 + frame_cd0 <= cd0_len);
}
