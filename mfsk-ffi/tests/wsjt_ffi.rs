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
    check_session(
        MfskMode::Ft8,
        &synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0),
        &["JA1ABC", "PM95"],
    );
}

#[test]
fn ft8_f32_and_i16_agree() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    let as_f32: Vec<f32> = slot.iter().map(|&s| s as f32 / 32768.0).collect();

    let a = open(MfskMode::Ft8, None);
    let via_i16 = texts(&decode_i16(a, &slot));
    unsafe { mfsk_session_close(a) };

    let b = open(MfskMode::Ft8, None);
    let via_f32 = texts(&decode_f32(b, &as_f32));
    unsafe { mfsk_session_close(b) };

    assert!(!via_i16.is_empty());
    assert_eq!(via_i16, via_f32, "the two sample types must agree");
}

#[test]
fn ft4_roundtrip() {
    check_session(
        MfskMode::Ft4,
        &synth_slot_i16(MfskMode::Ft4, "CQ", "JA1ABC", "PM95", 1500.0),
        &["JA1ABC", "PM95"],
    );
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
    check_session(
        MfskMode::Fst4s60,
        &synth_slot_i16(MfskMode::Fst4s60, "CQ", "JA1ABC", "PM95", 1500.0),
        &["JA1ABC", "PM95"],
    );

    // The four sub-modes the pre-v2 ABI could not address at all.
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
}

#[test]
fn wspr_roundtrip() {
    let call = CString::new("K1ABC").unwrap();
    let grid = CString::new("FN42").unwrap();
    let mut need = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode_wspr(
                call.as_ptr(),
                grid.as_ptr(),
                37,
                1500.0,
                std::ptr::null_mut(),
                0,
                &mut need,
            )
        },
        MfskStatus::InvalidArg,
        "a zero-capacity call should report the size it needs"
    );
    let mut pcm = vec![0.0f32; need];
    let mut got = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode_wspr(
                call.as_ptr(),
                grid.as_ptr(),
                37,
                1500.0,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut got,
            )
        },
        MfskStatus::Ok
    );

    let audio = f32_to_i16(&pcm[..got]);
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
}

/// JT9 and JT65 are point decodes at a known carrier, not searches.
/// The pre-v2 ABI reached them only through the generic decode call and
/// hardcoded 1500 / 1270 Hz with no way to say otherwise, so the
/// frequency being an argument is itself the thing under test.
#[test]
fn jt9_and_jt65_decode_at_a_caller_chosen_carrier() {
    type DecAt = unsafe extern "C" fn(
        *const i16,
        usize,
        u32,
        f32,
        *mut MfskDecode,
        usize,
        *mut usize,
    ) -> MfskStatus;
    for (enc, dec_at, mode, freq, tag) in [
        (
            mfsk_encode_jt9 as _,
            mfsk_jt9_decode_at as DecAt,
            MfskMode::Jt9,
            1350.0f32,
            "JT9",
        ),
        (
            mfsk_encode_jt65 as _,
            mfsk_jt65_decode_at as DecAt,
            MfskMode::Jt65,
            1270.0,
            "JT65",
        ),
    ] {
        let audio = f32_to_i16(&encode_f32(enc, "CQ", "K1ABC", "FN42", freq));
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
    }
}

#[test]
fn encode_ft8_bad_callsign_returns_invalid_arg() {
    // "XXX"/"Y2Z" are not packable by `pack77` — the encoder must
    // report failure rather than silently emitting garbage PCM.
    let c1 = CString::new("XXX").unwrap();
    let c2 = CString::new("Y2Z").unwrap();
    let r = CString::new("FN42").unwrap();
    let mut pcm = [0.0f32; 8];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_encode_ft8(
            c1.as_ptr(),
            c2.as_ptr(),
            r.as_ptr(),
            1500.0,
            pcm.as_mut_ptr(),
            pcm.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::InvalidArg);
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
    unsafe { mfsk_session_close(ptr::null_mut()) };
}
