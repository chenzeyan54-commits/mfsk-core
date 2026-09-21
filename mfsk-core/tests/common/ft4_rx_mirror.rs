// SPDX-License-Identifier: GPL-3.0-or-later
//! The CoreS3 FT4 receiver's per-slot decode, reproduced call for call.
//!
//! `embedded-poc/embedded-shared/src/apps/ft4_rx.rs` is outside this
//! workspace and builds only under the `+esp` toolchain, so this is the
//! host stand-in: the same public `mfsk-core` entry points, in the same
//! order, with the same constants. Every consumer that needs to ask
//! "what would the board decode?" goes through here rather than
//! rebuilding the sequence, because the sequence is the thing under
//! test — `ft4_coarse_sync` over the whole 90 000-sample slot, which is
//! what the other FT4 tests use, produces a different candidate list
//! from the receiver's early-closed, incrementally-accumulated one.
//!
//! What it deliberately does **not** reproduce, and what the board is
//! therefore still the only instrument for: the deadline and its cut
//! (`TX_TURNAROUND_BUDGET_MS`), the two-core split, PSRAM and the
//! internal-DRAM threshold, and the slot grid.

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

// ── the receiver's own constants ────────────────────────────────────
//
// Every one of these is a literal in `ft4_rx.rs`, repeated here because
// that crate cannot be linked from the host workspace.

/// `ft4_rx.rs`'s `SLOT_SAMPLES` — 7.5 s at 12 kHz.
pub const SLOT_SAMPLES: usize = 90_000;
/// `ft4_rx.rs`'s `CAPTURE_CLOSE_SAMPLES` — 6.775 s, where the receiver
/// stops taking audio because no candidate can reach past it.
pub const CAPTURE_CLOSE_SAMPLES: usize = 81_300;
pub const FREQ_MIN_HZ: f32 = 100.0;
pub const FREQ_MAX_HZ: f32 = 2_700.0;
pub const SYNC_MIN: f32 = 1.2;
pub const MAX_CAND: usize = 100;
/// `ft4::decode`'s own private `SYNC_Q_MIN`.
pub const SYNC_Q_MIN: u32 = 8;
/// `ft4_rx.rs`'s `WSJTX_WINDOW`, in `cd0` samples at 666.667 Hz.
pub const WSJTX_WINDOW: (i32, i32) = (-344, 1012);
/// The block size a UAC read produces, which is the cadence the board
/// feeds `SlotAccum` at. Block-independence is pinned elsewhere
/// (`slot_decimator_is_block_independent`); using the real one keeps
/// this a mirror.
pub const BLOCK: usize = 256;

/// The capture side: the periodogram and the shared half-rate stream,
/// advanced together a block at a time exactly as `SlotAccum` does.
pub fn capture(audio: &[i16]) -> (Vec<f32>, Vec<f32>) {
    let mut savg = Ft4SavgBuilder::new(CAPTURE_CLOSE_SAMPLES);
    let mut decim = SlotDecimator::new();
    let mut half: Vec<f32> = Vec::with_capacity(CAPTURE_CLOSE_SAMPLES / 2 + 64);
    let mut fed = 0usize;
    while fed < CAPTURE_CLOSE_SAMPLES.min(audio.len()) {
        let take = BLOCK.min(CAPTURE_CLOSE_SAMPLES.min(audio.len()) - fed);
        let block = &audio[fed..fed + take];
        savg.push_with_rows(block, &mut |_row| {});
        decim.push_i16(block, &mut half);
        fed += take;
    }
    (savg.finish(), half)
}

/// The coarse stage, with the receiver's own band and caps.
pub fn coarse(savg: &[f32]) -> Vec<SyncCandidate> {
    ft4_coarse_sync_from_savg(savg, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND)
}

/// The RMS normalisation `process_candidate_basic_impl` applies to a
/// `cd0` it builds itself (WSJT-X `ft4_decode.f90:231-232`).
/// `candidate_baseband_half` deliberately does not, so the receiver
/// does it — `ft4_rx.rs`'s `rms_normalise` — and so does this.
pub fn rms_normalise(cd0: &mut [Complex<f32>]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}

/// `ft4_rx::decode_candidate`, call for call.
pub fn decode_candidate(
    half: &[f32],
    cand: &SyncCandidate,
    refs: &Ft4CoarsePhasors,
) -> Option<String> {
    let mut cd0 = candidate_baseband_half(half, cand.freq_hz);
    rms_normalise(&mut cd0);
    let s2 = ft4_sync_search_window_with::<Ft4>(&cd0, cand, WSJTX_WINDOW.0, WSJTX_WINDOW.1, refs);
    let r: Option<DecodeResult> = process_candidate_precomputed::<Ft4>(
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
    );
    let m77: [u8; 77] = r?.message77().try_into().ok()?;
    unpack77(&m77)
}

/// One whole slot in, its distinct messages out, in candidate order —
/// which is descending coarse score, the order the receiver dedups in.
pub fn run_slot(audio: &[i16]) -> Vec<String> {
    let (savg, half) = capture(audio);
    let cands = coarse(&savg);
    let refs = Ft4CoarsePhasors::new::<Ft4>();
    let mut out: Vec<String> = Vec::new();
    for cand in &cands {
        if let Some(text) = decode_candidate(&half, cand, &refs)
            && !out.contains(&text)
        {
            out.push(text);
        }
    }
    out
}
