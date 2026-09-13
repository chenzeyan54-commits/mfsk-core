//! WSJT-family (non-Q65) FFI surface integration tests.
//!
//! `q65_ffi.rs` covers the dedicated `mfsk_q65_*` family; this file
//! exercises the rest — FT8 / FT4 / FST4 through the decode session,
//! and WSPR / JT9 / JT65 through their own entry points, plus the
//! encode calls and the NULL paths. Test vectors mirror
//! `examples/cpp_smoke/main.cpp` so a break here and a break in the
//! C++ driver should correlate.
//!
//! **All five FST4 sub-modes are exercised here now.** The pre-v2 ABI
//! had one FST4 entry in `MfskProtocol`, so 15/30/120/300 were
//! unreachable from C entirely — the single largest hole the mode
//! redesign closes, and worth a test that would notice if it reopened.

mod common;

use std::ffi::CString;
use std::ptr;

use common::*;
use mfsk::*;

fn empty_samples() -> MfskSamples {
    MfskSamples {
        samples: ptr::null_mut(),
        len: 0,
        _cap: 0,
    }
}

/// PCM from an `mfsk_encode_*` call, as i16 at 12 kHz.
fn pcm_i16(pcm: &MfskSamples) -> Vec<i16> {
    let f = unsafe { std::slice::from_raw_parts(pcm.samples, pcm.len) };
    f.iter()
        .map(|&s| (s * 32767.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

fn encoded(
    f: unsafe extern "C" fn(
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        f32,
        *mut MfskSamples,
    ) -> MfskStatus,
    a: &str,
    b: &str,
    c: &str,
    freq: f32,
) -> MfskSamples {
    let (a, b, c) = (
        CString::new(a).unwrap(),
        CString::new(b).unwrap(),
        CString::new(c).unwrap(),
    );
    let mut pcm = empty_samples();
    assert_eq!(
        unsafe { f(a.as_ptr(), b.as_ptr(), c.as_ptr(), freq, &mut pcm) },
        MfskStatus::Ok
    );
    pcm
}

fn check_session(mode: MfskMode, audio: &[i16], needles: &[&str]) {
    let dec = open(mode, None);
    let rows = decode_i16(dec, audio);
    for needle in needles {
        assert!(
            any_contains(&rows, needle),
            "expected '{needle}' in {mode:?} output: {:?}",
            texts(&rows)
        );
    }
    // Every row must name the sub-mode that produced it.
    assert!(rows.iter().all(|r| r.mode == mode));
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn ft8_roundtrip() {
    let mut pcm = encoded(mfsk_encode_ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    check_session(MfskMode::Ft8, &pcm_i16(&pcm), &["JA1ABC", "PM95"]);
    unsafe { mfsk_samples_free(&mut pcm) };
}

#[test]
fn ft8_f32_and_i16_agree() {
    let mut pcm = encoded(mfsk_encode_ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    let f = unsafe { std::slice::from_raw_parts(pcm.samples, pcm.len) }.to_vec();

    let a = open(MfskMode::Ft8, None);
    let via_i16 = texts(&decode_i16(a, &pcm_i16(&pcm)));
    unsafe { mfsk_session_close(a) };

    let b = open(MfskMode::Ft8, None);
    let via_f32 = texts(&decode_f32(b, &f));
    unsafe { mfsk_session_close(b) };

    assert!(!via_i16.is_empty());
    assert_eq!(via_i16, via_f32, "the two sample types must agree");
    unsafe { mfsk_samples_free(&mut pcm) };
}

#[test]
fn ft4_roundtrip() {
    let mut pcm = encoded(mfsk_encode_ft4, "CQ", "JA1ABC", "PM95", 1500.0);
    check_session(MfskMode::Ft4, &pcm_i16(&pcm), &["JA1ABC", "PM95"]);
    unsafe { mfsk_samples_free(&mut pcm) };
}

/// FST4: outer FFTs up to 4 194 304 points make this multi-second even
/// in `--release`, so it is gated the way the C++ driver gates its own.
/// See the `feedback_fst4_release_build` convention.
#[test]
fn every_fst4_submode_is_reachable_and_decodes() {
    if std::env::var_os("RUN_FST4_ROUNDTRIP").is_none() {
        eprintln!("skipping FST4 round-trips (set RUN_FST4_ROUNDTRIP=1 to run)");
        return;
    }
    let mut pcm = encoded(mfsk_encode_fst4s60, "CQ", "JA1ABC", "PM95", 1500.0);
    let frame = pcm_i16(&pcm);

    // The encoder emits FST4-60A; the other sub-modes have different
    // symbol rates, so only 60A can be decoded from this waveform. What
    // the rest are checked for here is that they are *addressable* and
    // open a session at all — the hole the pre-v2 ABI had.
    const SLOT: usize = 60 * FS as usize;
    const OFFSET: usize = FS as usize;
    let mut slot = vec![0i16; SLOT];
    let n = frame.len().min(SLOT - OFFSET);
    slot[OFFSET..OFFSET + n].copy_from_slice(&frame[..n]);
    check_session(MfskMode::Fst4s60, &slot, &["JA1ABC", "PM95"]);

    for m in [
        MfskMode::Fst4s15,
        MfskMode::Fst4s30,
        MfskMode::Fst4s120,
        MfskMode::Fst4s300,
    ] {
        assert_ne!(
            mfsk_mode_caps(m as u32) & MFSK_CAP_DECODE_HANDLE,
            0,
            "{m:?} must be addressable — four of five FST4 sub-modes were not, pre-v2"
        );
        unsafe { mfsk_session_close(open(m, None)) };
    }
    unsafe { mfsk_samples_free(&mut pcm) };
}

#[test]
fn wspr_roundtrip() {
    let call = CString::new("K1ABC").unwrap();
    let grid = CString::new("FN42").unwrap();
    let mut pcm = empty_samples();
    assert_eq!(
        unsafe { mfsk_encode_wspr(call.as_ptr(), grid.as_ptr(), 37, 1500.0, &mut pcm) },
        MfskStatus::Ok
    );
    let audio = pcm_i16(&pcm);
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_wspr_decode(
                audio.as_ptr(),
                audio.len(),
                FS,
                rows.as_mut_ptr(),
                rows.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    rows.truncate(n);
    assert!(any_contains(&rows, "K1ABC"), "{:?}", texts(&rows));
    assert!(rows.iter().all(|r| r.mode == MfskMode::Wspr));
    unsafe { mfsk_samples_free(&mut pcm) };
}

/// JT9 and JT65 are point decodes at a known carrier, not searches.
/// The pre-v2 ABI reached them only through the generic decode call and
/// hardcoded 1500 / 1270 Hz with no way to say otherwise, so the
/// frequency being an argument is itself the thing under test.
#[test]
fn jt9_and_jt65_decode_at_a_caller_chosen_carrier() {
    for (enc, dec_at, mode, freq, tag) in [
        (
            mfsk_encode_jt9 as unsafe extern "C" fn(_, _, _, f32, _) -> MfskStatus,
            mfsk_jt9_decode_at as unsafe extern "C" fn(_, _, _, f32, _, _, _) -> MfskStatus,
            MfskMode::Jt9,
            1350.0f32,
            "JT9",
        ),
        (
            mfsk_encode_jt65,
            mfsk_jt65_decode_at,
            MfskMode::Jt65,
            1270.0,
            "JT65",
        ),
    ] {
        let mut pcm = encoded(enc, "CQ", "K1ABC", "FN42", freq);
        let audio = pcm_i16(&pcm);
        let mut rows = vec![blank_row(); 4];
        let mut n = 0usize;
        assert_eq!(
            unsafe {
                dec_at(
                    audio.as_ptr(),
                    audio.len(),
                    FS,
                    freq,
                    rows.as_mut_ptr(),
                    rows.len(),
                    &mut n,
                )
            },
            MfskStatus::Ok,
            "{tag}"
        );
        rows.truncate(n);
        assert!(any_contains(&rows, "K1ABC"), "{tag}: {:?}", texts(&rows));
        assert!(rows.iter().all(|r| r.mode == mode));
        unsafe { mfsk_samples_free(&mut pcm) };
    }
}

#[test]
fn encode_ft8_bad_callsign_returns_invalid_arg() {
    // "XXX"/"Y2Z" are not packable by `pack77` — the encoder must
    // report failure rather than silently emitting garbage PCM.
    let c1 = CString::new("XXX").unwrap();
    let c2 = CString::new("Y2Z").unwrap();
    let r = CString::new("FN42").unwrap();
    let mut pcm = empty_samples();
    let st = unsafe { mfsk_encode_ft8(c1.as_ptr(), c2.as_ptr(), r.as_ptr(), 1500.0, &mut pcm) };
    assert_eq!(st, MfskStatus::InvalidArg);
    assert!(pcm.samples.is_null());
}

#[test]
fn null_pointers_are_rejected_or_ignored() {
    let mut n = 0usize;
    assert_eq!(
        unsafe { mfsk_wspr_decode(ptr::null(), 0, FS, ptr::null_mut(), 0, &mut n) },
        MfskStatus::InvalidArg
    );
    assert_eq!(
        unsafe { mfsk_jt9_decode_at(ptr::null(), 0, FS, 1500.0, ptr::null_mut(), 0, &mut n) },
        MfskStatus::InvalidArg
    );
    assert_eq!(
        unsafe { mfsk_jt65_decode_at(ptr::null(), 0, FS, 1270.0, ptr::null_mut(), 0, &mut n) },
        MfskStatus::InvalidArg
    );
    unsafe {
        mfsk_session_close(ptr::null_mut());
        mfsk_samples_free(ptr::null_mut());
    }
}
