//! The narrow-band search, which is FT8's alone.
//!
//! `MfskDecodeParams::search_hz` selects it. What these tests pin, in
//! order of what would actually break:
//!
//! 1. **It still works for the one mode that has it.** With the FT4 and
//!    FST4 arms gone, an FT8 positive case is the only thing exercising
//!    this path at all.
//! 2. **FT4 and FST4 refuse it.** The sniper is the receive half of
//!    narrowing a transceiver's *analogue* roofing filter — a DX-chasing
//!    mode a contest protocol has no use for, and one FST4 has a better
//!    answer to in its own DDC channelizer. Refusing is the design, not
//!    a gap.
//! 3. **The AP hint the sniper looked like it was *for* reaches those
//!    protocols anyway**, through the ordinary wide-band decode. That
//!    coupling was an accident; this is the test that says it is over.

mod common;

use std::ffi::CString;
use std::ptr;

use common::*;
use mfsk::*;

fn synth(
    enc: unsafe extern "C" fn(
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        f32,
        *mut MfskSamples,
    ) -> MfskStatus,
    call1: &str,
    call2: &str,
    report: &str,
    freq_hz: f32,
) -> (Vec<i16>, Vec<f32>) {
    let (c1, c2, r) = (
        CString::new(call1).unwrap(),
        CString::new(call2).unwrap(),
        CString::new(report).unwrap(),
    );
    let mut pcm = MfskSamples {
        samples: ptr::null_mut(),
        len: 0,
        _cap: 0,
    };
    assert_eq!(
        unsafe { enc(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), freq_hz, &mut pcm) },
        MfskStatus::Ok
    );
    let f = unsafe { std::slice::from_raw_parts(pcm.samples, pcm.len) }.to_vec();
    let i = f
        .iter()
        .map(|&s| (s * 32767.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect();
    unsafe { mfsk_samples_free(&mut pcm) };
    (i, f)
}

fn sniper_params(target_hz: f32, width_hz: f32) -> MfskDecodeParams {
    let mut p = params(MfskMode::Ft8);
    p.freq_hint_hz = target_hz;
    p.search_hz = width_hz;
    p
}

#[test]
fn ft8_decodes_at_the_target_frequency() {
    let (audio, _) = synth(mfsk_encode_ft8, "CQ", "JA1ABC", "PM95", 1200.0);
    let p = sniper_params(1200.0, 250.0);
    let dec = open(MfskMode::Ft8, Some(&p));
    let rows = decode_i16(dec, &audio);
    assert!(any_contains(&rows, "JA1ABC"), "{:?}", texts(&rows));
    assert!(
        rows.iter().all(|r| (r.freq_hz - 1200.0).abs() < 50.0),
        "a narrow search should only return what is in its window: {:?}",
        rows.iter().map(|r| r.freq_hz).collect::<Vec<_>>()
    );
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn f32_and_i16_agree() {
    let (i16s, f32s) = synth(mfsk_encode_ft8, "CQ", "JA1ABC", "PM95", 1200.0);
    let p = sniper_params(1200.0, 250.0);

    let a = open(MfskMode::Ft8, Some(&p));
    let via_i16 = texts(&decode_i16(a, &i16s));
    unsafe { mfsk_session_close(a) };

    let b = open(MfskMode::Ft8, Some(&p));
    let via_f32 = texts(&decode_f32(b, &f32s));
    unsafe { mfsk_session_close(b) };

    assert!(!via_i16.is_empty());
    assert_eq!(via_i16, via_f32);
}

/// Widening the window admits a signal outside the default one — which
/// is what says `search_hz` is a parameter rather than a literal.
#[test]
fn the_window_width_is_honoured() {
    let (a, _) = synth(mfsk_encode_ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    let (b, _) = synth(mfsk_encode_ft8, "CQ", "VK3NV", "QF22", 2100.0);
    let mixed: Vec<i16> = a
        .iter()
        .zip(b.iter())
        .map(|(&x, &y)| x.saturating_add(y))
        .collect();

    let narrow = sniper_params(1500.0, 250.0);
    let d = open(MfskMode::Ft8, Some(&narrow));
    let t = texts(&decode_i16(d, &mixed));
    unsafe { mfsk_session_close(d) };
    assert!(t.iter().any(|s| s.contains("JA1ABC")), "{t:?}");
    assert!(
        !t.iter().any(|s| s.contains("VK3NV")),
        "600 Hz away must be outside a ±250 Hz window: {t:?}"
    );

    let wide = sniper_params(1500.0, 700.0);
    let d = open(MfskMode::Ft8, Some(&wide));
    let t = texts(&decode_i16(d, &mixed));
    unsafe { mfsk_session_close(d) };
    assert!(t.iter().any(|s| s.contains("VK3NV")), "±700 Hz: {t:?}");
}

/// The sniper is FT8's alone, and the AP hint it looked like it was for
/// reaches FT4 through the ordinary wide-band decode.
#[test]
fn ft4_has_no_sniper_but_does_have_ap() {
    let (audio, _) = synth(mfsk_encode_ft4, "CQ", "JA1ABC", "PM95", 1200.0);

    let mut narrow = params(MfskMode::Ft4);
    narrow.freq_hint_hz = 1200.0;
    narrow.search_hz = 250.0;
    let mut st = MfskStatus::Ok;
    assert!(
        unsafe { mfsk_session_open(MfskMode::Ft4 as u32, &narrow, &mut st) }.is_null(),
        "FT4 must not offer a narrow-band search"
    );
    assert_eq!(st, MfskStatus::Unsupported);

    let mut wide = params(MfskMode::Ft4);
    with_ap(&mut wide, "JA1ABC", "CQ", "");
    let dec = open(MfskMode::Ft4, Some(&wide));
    let rows = decode_i16(dec, &audio);
    assert!(
        any_contains(&rows, "JA1ABC"),
        "the AP hint must reach FT4's wide-band decode: {:?}",
        texts(&rows)
    );
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn no_other_mode_offers_one() {
    for i in 0..mfsk_mode_count() {
        let mut m = MfskMode::Ft8;
        assert_eq!(unsafe { mfsk_mode_at(i, &mut m) }, MfskStatus::Ok);
        if m == MfskMode::Ft8 || mfsk_mode_caps(m as u32) & MFSK_CAP_DECODE_HANDLE == 0 {
            continue;
        }
        let mut p = params(m);
        p.freq_hint_hz = 1200.0;
        p.search_hz = 250.0;
        let mut st = MfskStatus::Ok;
        assert!(
            unsafe { mfsk_session_open(m as u32, &p, &mut st) }.is_null(),
            "{m:?} accepted a narrow-band search it does not have"
        );
        assert_eq!(st, MfskStatus::Unsupported, "{m:?}");
    }
}

/// `search_hz` says how wide, `freq_hint_hz` says where. Half a request
/// is an error rather than a guess.
#[test]
fn a_width_without_a_target_is_refused() {
    let mut p = params(MfskMode::Ft8);
    p.search_hz = 250.0;
    let mut st = MfskStatus::Ok;
    assert!(unsafe { mfsk_session_open(MfskMode::Ft8 as u32, &p, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::Unsupported);
    let msg = unsafe { std::ffi::CStr::from_ptr(mfsk_last_error()) }.to_string_lossy();
    assert!(msg.contains("freq_hint_hz"), "{msg}");
}
