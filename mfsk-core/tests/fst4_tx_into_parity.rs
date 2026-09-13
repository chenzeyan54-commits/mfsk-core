//! FST4's zero-allocation synthesis must produce exactly what the
//! allocating path produces, for every sub-mode.
//!
//! FT8 and FT4 have had `tones_to_{f32,i16}_into` since they were
//! written; FST4 had only the allocating variants, for no recorded
//! reason. Adding them is what lets the C ABI offer one
//! caller-allocates TX shape across all three protocols instead of
//! FT8's three-stage zero-copy pipeline and FT4/FST4's heap-returning
//! one.
//!
//! Two things are asserted, and the second is the one that would
//! actually catch a mistake:
//!
//! 1. `*_into` equals the allocating variant bit for bit.
//! 2. **The size query is right for every sub-mode.** `synth_sample_count`
//!    is the only thing standing between a C caller and a buffer
//!    overrun, and FST4's five geometries differ by a factor of 30
//!    (720 → 21 504 samples per symbol). A constant baked for FST4-60A
//!    would be silently wrong for the other four — which is precisely
//!    the shape of the existing `tones_to_f32` wrapper's latent trap.
#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use mfsk_core::engine::dsp::gfsk::GfskCfg;
use mfsk_core::fst4::encode::{
    FST4_15_GFSK, FST4_30_GFSK, FST4_60A_GFSK, FST4_120_GFSK, FST4_300_GFSK, message_to_tones,
    synth_sample_count, tones_to_f32, tones_to_f32_into, tones_to_f32_with_gfsk, tones_to_i16,
    tones_to_i16_into, tones_to_i16_with_gfsk,
};
use mfsk_core::msg::wsjt77::pack77;

const SUBMODES: [(&str, &GfskCfg, usize); 5] = [
    ("FST4-15", &FST4_15_GFSK, 720),
    ("FST4-30", &FST4_30_GFSK, 1_680),
    ("FST4-60A", &FST4_60A_GFSK, 3_888),
    ("FST4-120", &FST4_120_GFSK, 8_200),
    ("FST4-300", &FST4_300_GFSK, 21_504),
];

fn tones() -> Vec<u8> {
    let msg = pack77("JA1ABC", "VK3NV", "PM95").expect("pack77");
    message_to_tones(&msg)
}

#[test]
fn synth_sample_count_matches_every_submode_geometry() {
    let itone = tones();
    assert_eq!(itone.len(), 160, "every FST4 sub-mode has 160 symbols");

    for (name, cfg, nsps) in SUBMODES {
        assert_eq!(
            synth_sample_count(cfg),
            160 * nsps,
            "{name}: size query disagrees with N_SYMBOLS x NSPS"
        );
        // And the allocating path agrees with the query — the two are
        // independent routes to the same number, so a mistake in either
        // shows up here rather than as a C-side overrun.
        assert_eq!(
            tones_to_f32_with_gfsk(&itone, 1500.0, 0.5, cfg).len(),
            synth_sample_count(cfg),
            "{name}: allocating synthesis length disagrees with the query"
        );
    }
}

#[test]
fn into_variants_are_bit_identical_to_the_allocating_ones() {
    let itone = tones();

    for (name, cfg, _) in SUBMODES {
        let want_f32 = tones_to_f32_with_gfsk(&itone, 1500.0, 0.5, cfg);
        let mut got_f32 = vec![0.0f32; synth_sample_count(cfg)];
        tones_to_f32_into(&mut got_f32, &itone, 1500.0, 0.5, cfg);
        assert_eq!(got_f32, want_f32, "{name}: f32 into-variant diverged");

        let want_i16 = tones_to_i16_with_gfsk(&itone, 1500.0, 16_000, cfg);
        let mut got_i16 = vec![0i16; synth_sample_count(cfg)];
        tones_to_i16_into(&mut got_i16, &itone, 1500.0, 16_000, cfg);
        assert_eq!(got_i16, want_i16, "{name}: i16 into-variant diverged");
    }
}

/// The no-suffix wrappers silently mean FST4-60A. That is documented,
/// and worth pinning: if one ever changed sub-mode, every caller that
/// took the short name would move with it and nothing else would say so.
#[test]
fn bare_wrappers_still_mean_fst4_60a() {
    let itone = tones();
    assert_eq!(
        tones_to_f32(&itone, 1500.0, 0.5),
        tones_to_f32_with_gfsk(&itone, 1500.0, 0.5, &FST4_60A_GFSK)
    );
    assert_eq!(
        tones_to_i16(&itone, 1500.0, 16_000),
        tones_to_i16_with_gfsk(&itone, 1500.0, 16_000, &FST4_60A_GFSK)
    );
}
