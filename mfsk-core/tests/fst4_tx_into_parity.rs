//! FST4's zero-allocation synthesis must produce exactly what the
//! allocating path produces, for every sub-mode.
//!
//! This was written against FST4's own `tones_to_*_with_gfsk` /
//! `tones_to_*_into` / `synth_sample_count`, which took a `GfskCfg`
//! beside the tones. Since #391 all of that is
//! `engine::tx::{synthesize, synthesize_into, synth_len}::<P>`, with the
//! configuration a property of `P`. The two assertions still stand:
//!
//! 1. `*_into` equals the allocating variant bit for bit.
//! 2. **The size query is right for every sub-mode.** `synth_len` is
//!    the only thing standing between a C caller and a buffer overrun,
//!    and FST4's five geometries differ by a factor of 30 (720 → 21 504
//!    samples per symbol).
//!
//! A third test used to pin that the un-suffixed `tones_to_f32` wrapper
//! meant FST4-60A. That wrapper is gone: `synthesize::<P>` cannot be
//! called without naming a sub-mode, so there is no default to pin.
#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use mfsk_core::engine::tx::{
    FskWaveform, message_to_tones, synth_len, synthesize, synthesize_i16, synthesize_i16_into,
    synthesize_into,
};
use mfsk_core::fst4::{Fst4s15, Fst4s30, Fst4s60, Fst4s120, Fst4s300};
use mfsk_core::msg::wsjt77::pack77;

fn tones() -> Vec<u8> {
    let msg = pack77("JA1ABC", "VK3NV", "PM95").expect("pack77");
    message_to_tones::<Fst4s60>(&msg)
}

fn check<P: FskWaveform>(name: &str, nsps: usize) {
    let itone = tones();
    assert_eq!(itone.len(), 160, "every FST4 sub-mode has 160 symbols");

    assert_eq!(
        synth_len::<P>(12_000),
        160 * nsps,
        "{name}: size query disagrees with N_SYMBOLS x NSPS"
    );
    // The allocating path agrees with the query: two independent routes
    // to the same number, so a mistake in either shows up here rather
    // than as a C-side overrun.
    let want_f32 = synthesize::<P>(&itone, 12_000, 1500.0, 0.5);
    assert_eq!(
        want_f32.len(),
        synth_len::<P>(12_000),
        "{name}: allocating synthesis length disagrees with the query"
    );

    let mut got_f32 = vec![0.0f32; synth_len::<P>(12_000)];
    synthesize_into::<P>(&mut got_f32, &itone, 12_000, 1500.0, 0.5);
    assert_eq!(got_f32, want_f32, "{name}: f32 into-variant diverged");

    let want_i16 = synthesize_i16::<P>(&itone, 12_000, 1500.0, 16_000);
    let mut got_i16 = vec![0i16; synth_len::<P>(12_000)];
    synthesize_i16_into::<P>(&mut got_i16, &itone, 12_000, 1500.0, 16_000);
    assert_eq!(got_i16, want_i16, "{name}: i16 into-variant diverged");
}

#[test]
fn into_variants_and_size_query_hold_for_every_submode() {
    check::<Fst4s15>("FST4-15", 720);
    check::<Fst4s30>("FST4-30", 1_680);
    check::<Fst4s60>("FST4-60A", 3_888);
    check::<Fst4s120>("FST4-120", 8_200);
    check::<Fst4s300>("FST4-300", 21_504);
}
