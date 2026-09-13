//! Every `MfskDecodeParams` field must reach the decoder.
//!
//! The pre-v2 ABI's fields were set through eight fallible setters on
//! an opaque handle, and six of the eleven were silently dropped
//! depending on protocol. That is the failure this file exists to stop
//! coming back: a field that is accepted and ignored looks identical to
//! a field that works, from C, forever.
//!
//! So each test drives a *behavioural* difference rather than checking
//! that a setter returned OK — `sic_rounds` has to find more stations,
//! an AP hint has to recover one, `freq_hint` must not exclude
//! candidates.

mod common;

use common::*;
use mfsk::*;

fn synth_ft8_i16(call1: &str, call2: &str, report: &str, freq_hz: f32) -> Vec<i16> {
    synth_slot_i16(MfskMode::Ft8, call1, call2, report, freq_hz)
}

fn synth_ft8_i16_scaled(
    call1: &str,
    call2: &str,
    report: &str,
    freq_hz: f32,
    scale: f32,
) -> Vec<i16> {
    synth_ft8_i16(call1, call2, report, freq_hz)
        .iter()
        .map(|&s| ((s as f32) * scale / 32_767.0) as i16)
        .collect()
}

fn mix(a: &[i16], b: &[i16]) -> Vec<i16> {
    let len = a.len().max(b.len());
    (0..len)
        .map(|i| {
            let va = *a.get(i).unwrap_or(&0) as i32;
            let vb = *b.get(i).unwrap_or(&0) as i32;
            (va + vb).clamp(-32_768, 32_767) as i16
        })
        .collect()
}

fn load_wav_i16(path: &str) -> Vec<i16> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    assert!(
        bytes.len() >= 12 && &bytes[0..4] == b"RIFF" && &bytes[8..12] == b"WAVE",
        "{path}: not a RIFF/WAVE file"
    );
    let mut offset = 12usize;
    let mut data: Option<&[u8]> = None;
    while offset + 8 <= bytes.len() {
        let id = &bytes[offset..offset + 4];
        let size = u32::from_le_bytes(bytes[offset + 4..offset + 8].try_into().unwrap()) as usize;
        let body = offset + 8;
        if id == b"data" {
            data = Some(&bytes[body..body + size]);
        }
        offset = body + size + (size % 2);
    }
    let data = data.unwrap_or_else(|| panic!("{path}: missing data chunk"));
    data.as_chunks::<2>()
        .0
        .iter()
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

fn qso3_busy() -> Vec<i16> {
    load_wav_i16(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../embedded-poc/assets/qso3_busy.wav"
    ))
}

/// Count decodes on the busy recording under a given parameter set.
fn count(p: &MfskDecodeParams, audio: &[i16]) -> usize {
    let dec = open(MfskMode::Ft8, Some(p));
    let n = decode_i16(dec, audio).len();
    unsafe { mfsk_session_close(dec) };
    n
}

#[test]
fn strictness_and_eq_mode_reach_the_decoder_without_breaking_a_clean_signal() {
    let samples = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1500.0);
    for strictness in [
        MfskStrictness::Strict,
        MfskStrictness::Normal,
        MfskStrictness::Deep,
    ] {
        for eq in [MfskEqMode::Off, MfskEqMode::Local] {
            let mut p = params(MfskMode::Ft8);
            p.strictness = strictness;
            p.eq_mode = eq;
            let dec = open(MfskMode::Ft8, Some(&p));
            let rows = decode_i16(dec, &samples);
            assert!(
                any_contains(&rows, "JA1ABC"),
                "{strictness:?}/{eq:?} lost a clean signal: {:?}",
                texts(&rows)
            );
            unsafe { mfsk_session_close(dec) };
        }
    }
}

#[test]
fn freq_hint_reaches_the_decoder_and_does_not_exclude_candidates() {
    let sig_a = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1500.0);
    let sig_b = synth_ft8_i16_scaled("CQ", "K1ABC", "FN42", 2200.0, 32_767.0 * 0.5);
    let mixed = mix(&sig_a, &sig_b);

    let mut p = params(MfskMode::Ft8);
    p.freq_hint_hz = 1500.0;
    let dec = open(MfskMode::Ft8, Some(&p));
    let rows = decode_i16(dec, &mixed);
    let t = texts(&rows);
    assert!(t.iter().any(|s| s.contains("JA1ABC")), "{t:?}");
    assert!(
        t.iter().any(|s| s.contains("K1ABC")),
        "a hint prioritises, it must not exclude: {t:?}"
    );
    unsafe { mfsk_session_close(dec) };
}

/// The two SIC strategies have to *do* something, not merely be
/// accepted. This is the phantom-prone code (`CLAUDE.md`: both
/// false-decode bugs this suite has shipped were in subtraction paths),
/// so it is checked against a real recording rather than a synthetic.
#[test]
fn sic_rounds_and_sic_early_recover_more_stations_than_default() {
    let audio = qso3_busy();
    let base = || {
        let mut p = params(MfskMode::Ft8);
        p.freq_min_hz = 200.0;
        p.freq_max_hz = 3_000.0;
        p.sync_min = 2.0;
        p.max_cand = 50;
        p
    };

    let default_n = count(&base(), &audio);

    let mut early = base();
    early.sic_early = true;
    let early_n = count(&early, &audio);

    let mut rounds = base();
    rounds.sic_rounds = 3;
    let rounds_n = count(&rounds, &audio);

    assert!(
        early_n > default_n,
        "sic_early ({early_n}) should beat default ({default_n}) on qso3_busy"
    );
    assert!(
        rounds_n > default_n,
        "sic_rounds(3) ({rounds_n}) should beat default ({default_n}) on qso3_busy"
    );
}

/// An AP hint must take no decode away, and must surface the blind-CQ
/// one this recording is known to carry.
///
/// AP's real risk is manufactured decodes, so the strict-superset half
/// is the one that matters. Note it is deliberately **not** a
/// count-increase assertion: at these settings the plain path already
/// reaches 14 on this recording, and AP changes which passes earn them
/// rather than how many there are.
#[test]
fn an_ap_hint_surfaces_the_blind_cq_and_loses_nothing() {
    let audio = qso3_busy();
    let base = || {
        let mut p = params(MfskMode::Ft8);
        p.freq_min_hz = 100.0;
        p.freq_max_hz = 3_000.0;
        p.sync_min = 1.3;
        p.max_cand = 50;
        p
    };

    let run = |p: &MfskDecodeParams| {
        let dec = open(MfskMode::Ft8, Some(p));
        let t = texts(&decode_i16(dec, &audio));
        unsafe { mfsk_session_close(dec) };
        t
    };

    let ap_off = run(&base());
    let mut on = base();
    with_ap(&mut on, "K1JT", "HA0DU", "");
    let ap_on = run(&on);

    for m in &ap_off {
        assert!(
            ap_on.contains(m),
            "AP dropped a decode the plain path found: {m:?}"
        );
    }
    assert!(
        ap_on.iter().any(|m| m.contains("F5RXL")),
        "AP-on (mycall=K1JT, hiscall=HA0DU) should surface CQ F5RXL IN94 via the \
         blind-CQ pass, same as mfsk-core's own qso3_apon_recall test \
         (ap_off={ap_off:?} ap_on={ap_on:?})"
    );
}

/// A per-call params override applies to that call only — the session
/// keeps what it was opened with.
#[test]
fn a_per_call_override_does_not_stick() {
    let audio = qso3_busy();
    let mut wide = params(MfskMode::Ft8);
    wide.freq_min_hz = 200.0;
    wide.freq_max_hz = 3_000.0;
    wide.sync_min = 2.0;

    let dec = open(MfskMode::Ft8, Some(&wide));
    let n_default = decode_i16(dec, &audio).len();

    let mut narrow = wide;
    narrow.freq_min_hz = 1_400.0;
    narrow.freq_max_hz = 1_600.0;
    let n_narrow = decode_i16_with(dec, &audio, Some(&narrow)).len();

    let n_again = decode_i16(dec, &audio).len();
    unsafe { mfsk_session_close(dec) };

    assert!(
        n_narrow < n_default,
        "a 200 Hz window should find fewer than the full band ({n_narrow} vs {n_default})"
    );
    assert_eq!(
        n_again, n_default,
        "the override leaked into the next call on the same session"
    );
}
