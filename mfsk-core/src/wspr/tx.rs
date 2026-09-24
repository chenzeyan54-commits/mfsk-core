//! WSPR transmitter path: channel symbols → audio samples.
//!
//! Pragmatic first-pass synthesiser for end-to-end decoder tests. Each
//! symbol emits one continuous-phase sinusoid at
//! `base_freq + symbol * tone_spacing` for `NSPS / sample_rate` seconds.
//! **No GFSK symbol shaping, deliberately** — WSJT-X does not shape
//! WSPR either. `mainwindow.cpp` passes a *positive* `toneSpacing` for
//! WSPR, selecting `Modulator::modulate`'s plain-CPFSK branch rather
//! than the `toneSpacing < 0` "pre-computed, filtered waveform" branch
//! that FT8/FT4/FST4 use. Adding a raised-cosine frequency pulse here
//! would measurably lower close-in sidelobes (issue #259 measured
//! −53.8 → −85.9 dBc at +25 Hz for a T/8 pulse) but would emit a
//! different waveform than the reference implementation. See
//! `engine::dsp::envelope`'s module doc for the full comparison.
//!
//! The burst envelope *is* ramped — see [`crate::engine::dsp::envelope`].
//! Synthesis itself is [`crate::engine::tx::synthesize`]`::<Wspr>` since
//! #391; what stays here is the message-level convenience.

use alloc::vec::Vec;

/// Convenience wrapper that packs a message and synthesises in one step.
/// Returns `None` if the message can't fit the Type 1 layout.
pub fn synthesize_type1(
    callsign: &str,
    grid: &str,
    power_dbm: i32,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let info = crate::msg::wspr::pack_type1(callsign, grid, power_dbm)?;
    let symbols = super::encode_channel_symbols(&info);
    Some(crate::engine::tx::synthesize::<crate::wspr::Wspr>(
        &symbols,
        sample_rate,
        base_freq_hz,
        amplitude,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthesizes_162_symbol_buffer_at_12k() {
        let symbols = [0u8; 162];
        let audio =
            crate::engine::tx::synthesize::<crate::wspr::Wspr>(&symbols, 12_000, 1500.0, 0.5);
        // 8192 samples/symbol × 162 symbols = 1_327_104 samples
        assert_eq!(audio.len(), 8192 * 162);
    }

    #[test]
    fn synthesizes_valid_message() {
        let audio =
            synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3).expect("valid message");
        assert_eq!(audio.len(), 8192 * 162);
        // Basic sanity: peak amplitude close to the requested level.
        let peak = audio.iter().cloned().fold(0.0f32, f32::max);
        assert!(
            peak > 0.28 && peak < 0.32,
            "peak amplitude out of range: {}",
            peak
        );
    }
}
