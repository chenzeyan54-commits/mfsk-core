//! What `decode_block` — the board's driver, `#[cfg(not(feature =
//! "fft-rustfft"))]` — actually recovers of the fixed WSJT-X/JTDX
//! golden for `qso3_busy.wav`, instead of against a same-build
//! `DecodeRequest` reference.
//!
//! ## Why not `ft8_qso3_apoff_recall.rs`
//!
//! That file already runs this exact ship config
//! (`decode_block(..., DecodeDepth::EMBEDDED, 15)`, `sync_min=1.3`)
//! against the fixed 20-entry `QSO3_KNOWN_REAL_SIGNALS` golden — but
//! it is `#![cfg(feature = "fft-rustfft")]`. `decode_block`'s own
//! `decode_block_multipass` has two bodies split on that exact flag:
//! three passes with subtraction between them under `fft-rustfft`,
//! one pass with none without it. So that test's 14/12 floors, and
//! every other host recall-floor test's numbers, describe the
//! **3-pass driver — not the one any embedded build ships.**
//!
//! ## Why not a same-build `DecodeRequest` "truth" either
//!
//! `ft8_fixed_point_reference_drift.rs` assumed `DecodeRequest`'s
//! output was immune to this because its own code has no
//! `fft-rustfft` cfg branch. It compiles a completely different
//! internal path when `fft-rustfft` is off regardless — measured
//! directly: `DecodeRequest::new(...).decode()` (`max_cand=200`) on
//! `qso3_busy.wav` returns 15 messages under `full`, **8** under
//! `alloc,ft8,fft-extern` — because its default single-pass strategy
//! (`Ft8::__single_pass` -> `decode_frame_inner`) itself gates
//! `apply_wsjtx_xsnr2` and reaches into `fill_symbol_spectra.rs` /
//! `osd_strategy.rs`, both independently `fft-rustfft`-split. A
//! same-build reference is not a fixed reference; this crate's own
//! `#359` thread already said the floors want one, and this is why.
//!
//! So this file measures the ship config against the **external,
//! feature-independent** golden — WSJT-X's own AP-off decode plus
//! JTDX's, reconciled in `common::ft8_qso3::QSO3_KNOWN_REAL_SIGNALS` —
//! reusing the same `#359`/`ft8_decode_block_streaming.rs` pattern of
//! providing `fft-extern`'s contract directly rather than pulling in
//! `mod common` (which needs `fft-rustfft`/`uvpacket` to compile at
//! all). `common::golden`/`common::ft8_qso3` have no such dependency,
//! so they are mounted directly by path instead of duplicated.
//!
//! Run against the board's actual matrix entry:
//! ```sh
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
//!     --test ft8_embedded_driver_recall -- --nocapture
//! ```
//! and its f32 twin (drop `fixed-point`) for the pre-quantisation number.
//!
//! Refs #359, #357.
#![cfg(not(feature = "fft-rustfft"))]

use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[path = "common/ft8_qso3.rs"]
mod ft8_qso3;
#[path = "common/golden.rs"]
mod golden;

use ft8_qso3::{DF_TOL_HZ, QSO3_KNOWN_REAL_SIGNALS, SNR_TOL_DB};

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

// The `fft-extern` contract and the WAV loader, shared with
// `ft8_embedded_grid_phase.rs` — see that module's own doc for why a
// host test provides the contract itself.
#[path = "common/embedded_driver_harness.rs"]
mod harness;

use harness::load_wav_i16;

/// Hard-assertion floor for the driver every embedded build actually
/// ships (`decode_block_multipass`'s `not(fft-rustfft)` body) against
/// the fixed, feature-independent golden. Measured 2026-09-06:
///
/// | numeric | recall | phantoms |
/// |---|---:|---:|
/// | f32 | 4/20 | 0 |
/// | fixed-point (what the board runs) | 7/20 | 0 |
///
/// `assert_golden` isn't reused — it also checks per-entry SNR, and no
/// independent SNR reference exists for this driver/golden pairing yet
/// (unlike `ft8_qso3_apoff_recall.rs`'s tolerance-checked SNRs). This
/// stays a plain recall/phantom check until one does.
#[cfg(not(feature = "fixed-point"))]
const MIN_HITS: usize = 4;
#[cfg(feature = "fixed-point")]
const MIN_HITS: usize = 7;
const MAX_EXTRA: usize = 0;

#[test]
fn true_ship_driver_meets_fixed_golden_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    let decoded = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15);
    let msgs: Vec<String> = decoded
        .iter()
        .filter_map(|d| unpack77(d.message77()))
        .collect();

    println!(
        "\ndriver: single-pass, no subtraction (cfg(not(fft-rustfft)))  numeric: {}\n",
        if cfg!(feature = "fixed-point") {
            "fixed-point"
        } else {
            "f32"
        }
    );

    let hits: Vec<&str> = QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .filter(|g| msgs.iter().any(|m| m == g.msg))
        .map(|g| g.msg)
        .collect();
    let missing: Vec<&str> = QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .map(|g| g.msg)
        .filter(|m| !hits.contains(m))
        .collect();
    let phantoms: Vec<&str> = msgs
        .iter()
        .filter(|m| !QSO3_KNOWN_REAL_SIGNALS.iter().any(|g| g.msg == **m))
        .map(|m| m.as_str())
        .collect();

    println!(
        "recall {}/{} (floor {MIN_HITS})  phantoms {} (ceiling {MAX_EXTRA})",
        hits.len(),
        QSO3_KNOWN_REAL_SIGNALS.len(),
        phantoms.len()
    );
    println!("hit:      {hits:?}");
    println!("missing:  {missing:?}");
    println!("phantoms: {phantoms:?}");

    assert!(
        hits.len() >= MIN_HITS,
        "recall regressed: {}/{} decoded, floor is {MIN_HITS}. Missing: {missing:?}",
        hits.len(),
        QSO3_KNOWN_REAL_SIGNALS.len()
    );
    assert!(
        phantoms.len() <= MAX_EXTRA,
        "emitted {} decode(s) outside the golden set, budget is {MAX_EXTRA}. Phantoms: {phantoms:?}",
        phantoms.len()
    );
    // Reserved for when this test also checks per-entry SNR.
    let _ = (DF_TOL_HZ, SNR_TOL_DB);
}
