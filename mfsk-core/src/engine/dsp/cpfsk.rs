//! Plain continuous-phase FSK synthesis: the transmit waveform of WSPR,
//! JT9, JT65 and Q65.
//!
//! This is WSJT-X's positive-`toneSpacing` path, generated sample by
//! sample in `Modulator::modulate` (`m_phi += m_dphi; sample =
//! m_amp·sin(m_phi)`). It applies no symbol shaping; see
//! [`super::envelope`] for why these four modes get none and why they
//! still get an envelope ramp. FT8, FT4 and FST4 use the pre-computed,
//! filtered waveform instead ([`super::gfsk`]).
//!
//! Each of the four `tx.rs` files used to carry its own copy of this
//! loop. The body below is that loop exactly, so the output is
//! bit-identical to what they produced.

use alloc::vec;
use alloc::vec::Vec;
use core::f32::consts::TAU;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::envelope;

/// Samples per symbol at `sample_rate`, from a symbol duration in
/// seconds (`ModulationParams::SYMBOL_DT`). The protocols' own `NSPS`
/// constants are for 12 kHz; this scales them to any rate.
#[inline]
pub fn nsps(sample_rate: u32, symbol_dt: f32) -> usize {
    (sample_rate as f32 * symbol_dt).round() as usize
}

/// Synthesise `tones` into `out` as continuous-phase FSK, then apply
/// the transmit-envelope ramp ([`envelope::apply_ramp`]).
///
/// Symbol `k` is a sinusoid at `f0_hz + tones[k] · tone_spacing_hz`
/// lasting `nsps` samples. Phase carries across symbol boundaries.
/// No allocation.
///
/// # Panics
///
/// Panics if `out.len() != nsps · tones.len()`.
pub fn synth_f32_into(
    out: &mut [f32],
    tones: &[u8],
    nsps: usize,
    f0_hz: f32,
    tone_spacing_hz: f32,
    sample_rate: u32,
    amplitude: f32,
) {
    assert_eq!(
        out.len(),
        nsps * tones.len(),
        "cpfsk::synth_f32_into: out.len() must equal nsps * tones.len()"
    );
    let mut phase = 0.0f32;
    let mut idx = 0usize;
    for &sym in tones {
        let freq = f0_hz + sym as f32 * tone_spacing_hz;
        let dphi = TAU * freq / sample_rate as f32;
        for _ in 0..nsps {
            out[idx] = amplitude * phase.cos();
            idx += 1;
            phase += dphi;
            if phase > TAU {
                phase -= TAU;
            } else if phase < -TAU {
                phase += TAU;
            }
        }
    }

    // Transmit-envelope ramp (issue #259). Without it the burst starts
    // and ends on a step discontinuity, a broadband click at both edges.
    envelope::apply_ramp(out, envelope::ramp_samples(sample_rate, nsps));
}

/// Allocating form of [`synth_f32_into`].
pub fn synth_f32(
    tones: &[u8],
    nsps: usize,
    f0_hz: f32,
    tone_spacing_hz: f32,
    sample_rate: u32,
    amplitude: f32,
) -> Vec<f32> {
    let mut out = vec![0.0f32; nsps * tones.len()];
    synth_f32_into(
        &mut out,
        tones,
        nsps,
        f0_hz,
        tone_spacing_hz,
        sample_rate,
        amplitude,
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn length_and_ramped_edges() {
        let tones = [0u8, 1, 2, 3];
        let out = synth_f32(&tones, 1000, 1500.0, 1.5, 12_000, 0.5);
        assert_eq!(out.len(), 4000);
        // The ramp starts at zero envelope.
        assert_eq!(out[0], 0.0);
        assert!(out.iter().all(|s| s.abs() <= 0.5));
    }

    #[test]
    fn phase_is_continuous_across_symbols() {
        // At a symbol boundary the sample-to-sample step stays below the
        // largest per-sample step either tone can take: no phase jump.
        let nsps = 1000;
        let out = synth_f32(&[0u8, 7], nsps, 1000.0, 50.0, 12_000, 1.0);
        let max_dphi = TAU * 1350.0 / 12_000.0;
        let step = (out[nsps] - out[nsps - 1]).abs();
        assert!(step <= max_dphi, "step {step} > {max_dphi}");
    }
}
