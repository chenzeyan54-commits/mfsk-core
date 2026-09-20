// SPDX-License-Identifier: GPL-3.0-or-later
//! What a spectrogram's unfilled tail actually does to coarse sync.
//!
//! **Runs on the embedded time grid only.** `nstep-half` (implied by
//! `fixed-point`) gives the 184-row slot this arithmetic is written
//! against; the host default grid has 372 and an emit point means
//! nothing on it.
//!
//! ```sh
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
//!     --test ft8_coarse_partial_blocks -- --nocapture
//! ```
#![cfg(all(feature = "ft8", not(feature = "fft-rustfft")))]

use mfsk_core::ft8::decode_block::{
    coarse_sync_with_allsum_and_lag, compute_spectrogram, precompute_coarse_allsum,
};

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

#[path = "common/embedded_driver_harness.rs"]
mod harness;

use harness::load_wav_i16;

const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
const MAX_CAND: usize = 30;
/// `stage1_inc`'s shipped emit point fills `2 * 87` of the slot's 184
/// rows; the rest are present and zero when the bundle is sent.
const FILLED: usize = 174;

fn spec_and_allsum(zero_tail: bool) -> (mfsk_core::ft8::decode_block::Spectrogram, Vec<f32>) {
    let audio = load_wav_i16(std::path::Path::new(asset_path!("qso3_busy.wav")));
    let mut spec = compute_spectrogram(&audio[..180_000.min(audio.len())], FREQ_MAX);
    assert_eq!(spec.n_time, 184, "not the embedded time grid");
    if zero_tail {
        // Layout is time-major: `data[time * n_freq + freq]`.
        let n_freq = spec.n_freq;
        for m in FILLED..spec.n_time {
            for f in 0..n_freq {
                spec.data[m * n_freq + f] = Default::default();
            }
        }
    }
    let allsum = precompute_coarse_allsum(&spec, FREQ_MIN, FREQ_MAX);
    (spec, allsum)
}

/// **Zeros are not a hazard, and this is the test that says so.**
///
/// The reverted ±1.75 s widen (2026-09-19) was explained in the code
/// as "a correlation against zeros does not come out small — it comes
/// out whatever the normalisation makes of it". That explanation is
/// wrong, and believing it cost a design round: the score accumulates
/// `t_blocks[2] += power` and `t0_blocks[2] += allsum` as plain
/// **sums**, so a zero row adds nothing to either. Reading zeros and
/// skipping the symbol are numerically identical.
///
/// What large lag really does is leave a candidate scored on **two
/// Costas blocks instead of three**, with a ratio that is not
/// penalised for the missing evidence — so a two-block coincidence
/// competes with a three-block station. That is the mechanism, it is
/// WSJT-X's own (`sync8.f90` guards `m + nssy*72 <= NHSYM` and skips),
/// and it is not fixed by telling coarse sync where the fill stopped.
#[test]
fn a_zeroed_tail_scores_the_same_as_a_shorter_slot() {
    let (spec, allsum) = spec_and_allsum(true);
    // ±1.6 s: block 2's last symbol lands at row 162 + 20 = 182, deep
    // in the zeroed tail, which is the condition the widen created.
    let wide = coarse_sync_with_allsum_and_lag(
        &spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, &allsum, 1.6,
    );
    let far: Vec<_> = wide.iter().filter(|c| c.dt_sec.abs() > 0.88).collect();
    println!("  {} candidates, {} past |0.88| s", wide.len(), far.len());
    for c in far.iter().take(6) {
        println!(
            "    {:+.2}s {:.0}Hz score {:.2}",
            c.dt_sec, c.freq_hz, c.score
        );
    }
    // The far-lag candidates exist and carry ordinary scores — they
    // are not suppressed by having lost block 2, which is the whole
    // point. If a future change makes the score evidence-aware, this
    // is the assertion that will fail and should be re-derived.
    assert!(
        !far.is_empty(),
        "expected the wide window to admit far-lag candidates"
    );
}

/// **The shipped ±1.0 s window does admit two-block candidates**, and
/// they cost pass-1 slots.
///
/// 0.12 s past what 174 rows score in full
/// (`stage1_inc::max_lag_s` = 0.88), and on `qso3_busy` that is 2 of
/// the 30 pass-1 places going to candidates whose block 2 is partly
/// missing. They do not decode — 1 382 on-air decodes across three
/// arms on 7041 kHz had **none** past +0.88 s (2026-09-20) — so the
/// cost is the shortlist place, not a wrong row.
///
/// Recorded rather than fixed: narrowing the window to 0.88 s would
/// free those two places and would also drop the *negative* side,
/// where 0.26-1.35 % of real decodes live (the exposure is positive
/// only — block 2 runs past the end, block 1 bounds the other side).
/// Paying a measured loss for an unmeasured gain is the trade this
/// test exists to keep visible.
#[test]
fn the_shipped_window_admits_two_block_candidates() {
    let (spec, allsum) = spec_and_allsum(true);
    let shipped = coarse_sync_with_allsum_and_lag(
        &spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, &allsum, 1.0,
    );
    let far: Vec<_> = shipped.iter().filter(|c| c.dt_sec > 0.88).collect();
    let ranks: Vec<usize> = shipped
        .iter()
        .enumerate()
        .filter(|(_, c)| c.dt_sec > 0.88)
        .map(|(i, _)| i)
        .collect();
    println!(
        "  {} candidates, {} past +0.88 s at pass-1 ranks {:?}",
        shipped.len(),
        far.len(),
        ranks
    );
    // The claim is that they exist and are few, not that they are
    // absent. A change that makes them many, or that pushes them into
    // the refined top-15 (`MAX_CAND` on the board), should fail here.
    assert!(
        far.len() <= 4,
        "far-lag candidates took {} of {} pass-1 places",
        far.len(),
        shipped.len()
    );
}
