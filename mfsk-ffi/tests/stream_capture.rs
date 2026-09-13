//! Streaming capture and the slot grid.
//!
//! Two properties carry this: the slot a stream hands over must decode
//! to what was fed in, and **the library must never read a clock**. The
//! host says what UTC second the next sample belongs to and the grid
//! does arithmetic — which is what keeps this usable from wasm, from
//! `no_std`, and from a phone that was backgrounded for four minutes.
//!
//! Generalised from `mfsk-ffi-ft8`'s FT8-only, i16-only front end: the
//! ring is sized from `slot_samples_12k`, so FST4-300's 3.6 M-sample
//! slot works the same way FT4's 90 000-sample one does.

mod common;

use common::*;
use mfsk::*;

fn open_stream(mode: MfskMode, rate: u32) -> *mut MfskStream {
    let mut st = MfskStatus::Internal;
    let s = unsafe { mfsk_stream_open(mode as u32, rate, &mut st) };
    assert_eq!(st, MfskStatus::Ok, "{mode:?} @ {rate}");
    assert!(!s.is_null());
    s
}

fn push(s: *mut MfskStream, pcm: &[i16]) {
    assert_eq!(
        unsafe { mfsk_stream_push_i16(s, pcm.as_ptr(), pcm.len()) },
        MfskStatus::Ok
    );
}

#[test]
fn a_taken_slot_decodes_to_what_was_pushed() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    let s = open_stream(MfskMode::Ft8, FS);

    assert!(!mfsk_stream_slot_ready(s), "nothing pushed yet");
    push(s, &slot[..slot.len() / 2]);
    assert!(!mfsk_stream_slot_ready(s), "half a slot is not a slot");
    push(s, &slot[slot.len() / 2..]);
    assert!(mfsk_stream_slot_ready(s));
    assert_eq!(mfsk_stream_buffered(s), slot.len());

    let mut out = vec![0i16; slot.len()];
    let mut utc = 0.0f64;
    let n = unsafe { mfsk_stream_take_slot_i16(s, out.as_mut_ptr(), out.len(), &mut utc) };
    assert_eq!(n, slot.len());
    assert_eq!(out, slot, "the slot must come back unchanged");
    assert!(!mfsk_stream_slot_ready(s), "taking consumes it");

    let dec = open(MfskMode::Ft8, None);
    let rows = decode_i16(dec, &out);
    assert!(any_contains(&rows, "JA1ABC"), "{:?}", texts(&rows));
    unsafe {
        mfsk_session_close(dec);
        mfsk_stream_close(s);
    }
}

/// The fused call exists so FST4-300's 3.6 M-sample slot is not copied
/// out and back in. It must agree with take-then-decode.
#[test]
fn decoding_the_stream_directly_agrees_with_taking_it_first() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0);

    let s = open_stream(MfskMode::Ft8, FS);
    push(s, &slot);
    let dec = open(MfskMode::Ft8, None);
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let mut utc = -1.0f64;
    assert_eq!(
        unsafe {
            mfsk_session_decode_stream(
                dec,
                s,
                std::ptr::null(),
                rows.as_mut_ptr(),
                rows.len(),
                &mut n,
                &mut utc,
            )
        },
        MfskStatus::Ok
    );
    rows.truncate(n);
    let fused = texts(&rows);
    assert!(!fused.is_empty());
    assert!(
        !mfsk_stream_slot_ready(s),
        "the fused call consumes the slot"
    );
    unsafe {
        mfsk_session_close(dec);
        mfsk_stream_close(s);
    }

    let dec2 = open(MfskMode::Ft8, None);
    let separate = texts(&decode_i16(dec2, &slot));
    unsafe { mfsk_session_close(dec2) };
    assert_eq!(fused, separate);
}

/// Polling the fused call before a slot is ready is not an error a
/// caller has to avoid — it is the "not yet" answer.
#[test]
fn the_fused_call_says_not_yet_rather_than_failing() {
    let s = open_stream(MfskMode::Ft8, FS);
    let dec = open(MfskMode::Ft8, None);
    let mut rows = vec![blank_row(); 4];
    let mut n = 99usize;
    assert_eq!(
        unsafe {
            mfsk_session_decode_stream(
                dec,
                s,
                std::ptr::null(),
                rows.as_mut_ptr(),
                rows.len(),
                &mut n,
                std::ptr::null_mut(),
            )
        },
        MfskStatus::Unsupported
    );
    assert_eq!(
        n, 0,
        "*out_len must be cleared, not left at whatever it was"
    );
    unsafe {
        mfsk_session_close(dec);
        mfsk_stream_close(s);
    }
}

/// A stream and a session for different modes must not be paired.
#[test]
fn a_mismatched_stream_and_session_are_refused() {
    let s = open_stream(MfskMode::Ft4, FS);
    let dec = open(MfskMode::Ft8, None);
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_session_decode_stream(
                dec,
                s,
                std::ptr::null(),
                std::ptr::null_mut(),
                0,
                &mut n,
                std::ptr::null_mut(),
            )
        },
        MfskStatus::InvalidArg
    );
    let msg = unsafe { std::ffi::CStr::from_ptr(mfsk_session_last_error(dec)) }.to_string_lossy();
    assert!(msg.contains("different modes"), "{msg}");
    unsafe {
        mfsk_session_close(dec);
        mfsk_stream_close(s);
    }
}

/// **The library never reads a clock.** Without an epoch the grid
/// free-runs from the first sample — right for replaying a recording.
/// With one, the reported slot start is arithmetic on what the host
/// said, and nothing else.
#[test]
fn time_is_a_parameter_and_is_never_read() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0);

    // Free-running: the first slot starts at t = 0 by construction.
    let s = open_stream(MfskMode::Ft8, FS);
    push(s, &slot);
    let mut out = vec![0i16; slot.len()];
    let mut utc = -1.0;
    unsafe { mfsk_stream_take_slot_i16(s, out.as_mut_ptr(), out.len(), &mut utc) };
    assert_eq!(utc, 0.0, "free-running starts at zero");
    unsafe { mfsk_stream_close(s) };

    // With an epoch the answer is exactly what the host declared, with
    // no wall-clock component — run twice and compare to prove it.
    let run = || {
        let s = open_stream(MfskMode::Ft8, FS);
        unsafe { mfsk_stream_set_epoch(s, 1_700_000_000.0) };
        push(s, &slot);
        let mut out = vec![0i16; slot.len()];
        let mut t = -1.0f64;
        unsafe { mfsk_stream_take_slot_i16(s, out.as_mut_ptr(), out.len(), &mut t) };
        unsafe { mfsk_stream_close(s) };
        t
    };
    let a = run();
    let b = run();
    assert_eq!(a, 1_700_000_000.0, "the epoch is the answer, verbatim");
    assert_eq!(a, b, "two identical runs must report an identical time");

    // A second slot advances by exactly one slot length.
    let s = open_stream(MfskMode::Ft8, FS);
    unsafe { mfsk_stream_set_epoch(s, 1_700_000_000.0) };
    push(s, &slot);
    push(s, &slot);
    let mut out = vec![0i16; slot.len()];
    let mut t1 = 0.0;
    unsafe { mfsk_stream_take_slot_i16(s, out.as_mut_ptr(), out.len(), &mut t1) };
    // The ring holds one slot, so the first push was displaced; the
    // slot now present is the newer one.
    assert!(
        (t1 - 1_700_000_015.0).abs() < 1e-6,
        "one FT8 slot later, got {t1}"
    );
    unsafe { mfsk_stream_close(s) };
}

/// Overrunning the ring keeps the newest audio, which is what a live
/// receiver wants — an old slot is not worth decoding late.
#[test]
fn an_overrun_keeps_the_newest_slot() {
    let old = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    let new = synth_slot_i16(MfskMode::Ft8, "CQ", "VK3NV", "QF22", 1700.0);

    let s = open_stream(MfskMode::Ft8, FS);
    push(s, &old);
    push(s, &new);
    assert_eq!(
        mfsk_stream_buffered(s),
        new.len(),
        "the ring holds one slot"
    );

    let dec = open(MfskMode::Ft8, None);
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    unsafe {
        mfsk_session_decode_stream(
            dec,
            s,
            std::ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
            std::ptr::null_mut(),
        )
    };
    rows.truncate(n);
    assert!(any_contains(&rows, "VK3NV"), "{:?}", texts(&rows));
    assert!(
        !any_contains(&rows, "JA1ABC"),
        "the old slot should be gone"
    );
    unsafe {
        mfsk_session_close(dec);
        mfsk_stream_close(s);
    }
}

/// The ring is sized per mode, which is the whole point of generalising
/// an FT8-only front end.
#[test]
fn the_ring_is_sized_from_the_mode() {
    for m in [
        MfskMode::Ft4,
        MfskMode::Ft8,
        MfskMode::Fst4s15,
        MfskMode::Fst4s300,
    ] {
        let mut info = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
        assert_eq!(
            unsafe { mfsk_mode_info(m as u32, info.as_mut_ptr()) },
            MfskStatus::Ok
        );
        let want = unsafe { info.assume_init() }.slot_samples_12k as usize;

        let s = open_stream(m, FS);
        let quiet = vec![0i16; want];
        push(s, &quiet);
        assert!(
            mfsk_stream_slot_ready(s),
            "{m:?}: a full slot should be ready"
        );
        assert_eq!(mfsk_stream_buffered(s), want, "{m:?}");
        unsafe { mfsk_stream_close(s) };
    }
    // FT4 90 000 vs FST4-300 3 600 000 — a factor of 40, which an
    // FT8-sized ring would have got wrong in both directions.
}

/// Audio arriving at some other rate is resampled on the way in.
#[test]
fn a_non_12k_source_is_resampled() {
    let s = open_stream(MfskMode::Ft8, 48_000);
    let quiet = vec![0i16; 48_000];
    push(s, &quiet);
    let buffered = mfsk_stream_buffered(s);
    assert!(
        (11_000..=13_000).contains(&buffered),
        "1 s at 48 kHz should become about 12 000 samples, got {buffered}"
    );
    unsafe { mfsk_stream_close(s) };
}

/// f32 input reaches the same ring.
#[test]
fn float_input_works_too() {
    let slot = synth_slot_i16(MfskMode::Ft8, "CQ", "JA1ABC", "PM95", 1500.0);
    let as_f32: Vec<f32> = slot.iter().map(|&x| x as f32 / 32768.0).collect();
    let s = open_stream(MfskMode::Ft8, FS);
    assert_eq!(
        unsafe { mfsk_stream_push_f32(s, as_f32.as_ptr(), as_f32.len()) },
        MfskStatus::Ok
    );
    assert!(mfsk_stream_slot_ready(s));

    let dec = open(MfskMode::Ft8, None);
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    unsafe {
        mfsk_session_decode_stream(
            dec,
            s,
            std::ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
            std::ptr::null_mut(),
        )
    };
    rows.truncate(n);
    assert!(any_contains(&rows, "JA1ABC"), "{:?}", texts(&rows));
    unsafe {
        mfsk_session_close(dec);
        mfsk_stream_close(s);
    }
}

#[test]
fn nulls_and_bad_modes_are_rejected() {
    let mut st = MfskStatus::Ok;
    assert!(unsafe { mfsk_stream_open(9999, FS, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);

    // A mode with no decode handle has nothing to feed.
    assert!(unsafe { mfsk_stream_open(MfskMode::Wspr as u32, FS, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::Unsupported);

    assert!(unsafe { mfsk_stream_open(MfskMode::Ft8 as u32, 0, &mut st) }.is_null());
    assert_eq!(st, MfskStatus::InvalidArg);

    assert_eq!(
        unsafe { mfsk_stream_push_i16(std::ptr::null_mut(), std::ptr::null(), 0) },
        MfskStatus::InvalidArg
    );
    assert_eq!(mfsk_stream_buffered(std::ptr::null()), 0);
    assert!(!mfsk_stream_slot_ready(std::ptr::null()));
    unsafe {
        mfsk_stream_clear(std::ptr::null_mut());
        mfsk_stream_close(std::ptr::null_mut());
    }
}
