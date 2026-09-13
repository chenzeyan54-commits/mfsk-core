//! Streaming delivery through `mfsk_session_set_on_decode`.
//!
//! `.on_result()` was never exposed across this FFI before the
//! pre-v2 `mfsk_decode_i16_streaming` existed (issue #246 follow-up).
//! That function took the callback per call and applied only to FT8;
//! the session takes it once and applies it to every mode the decode
//! handle drives, which is what makes it useful for FT4 and FST4 too.
//!
//! Self-synthesised signals, no WAV fixtures: the callback's own
//! delivery is checked against the array the same call fills in, which
//! is the property `STREAMING.md` §3 actually promises.

mod common;

use std::ffi::c_void;
use std::ptr;
use std::sync::Mutex;

use common::*;
use mfsk::*;

fn synth_ft8_i16(call1: &str, call2: &str, report: &str, freq_hz: f32) -> Vec<i16> {
    let msg = mfsk_core::msg::wsjt77::pack77(call1, call2, report).expect("pack77");
    let tones = mfsk_core::ft8::wave_gen::message_to_tones(&msg);
    let wave = mfsk_core::ft8::wave_gen::tones_to_i16(&tones, freq_hz, 8_000);
    let mut slot = vec![0i16; 15 * FS as usize];
    let start = (0.5 * FS as f32) as usize;
    for (i, s) in wave.iter().enumerate() {
        if let Some(d) = slot.get_mut(start + i) {
            *d = d.saturating_add(*s);
        }
    }
    slot
}

unsafe extern "C" fn collect_cb(row: *const MfskDecode, user_data: *mut c_void) {
    assert!(!row.is_null());
    let sink = unsafe { &*(user_data as *const Mutex<Vec<String>>) };
    sink.lock().unwrap().push(text_of(unsafe { &*row }));
}

#[test]
fn the_callback_fires_and_agrees_with_the_array() {
    let samples = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1500.0);
    let dec = open(MfskMode::Ft8, None);

    let collected: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let ud = &collected as *const Mutex<Vec<String>> as *mut c_void;
    assert_eq!(
        unsafe { mfsk_session_set_on_decode(dec, Some(collect_cb), ud) },
        MfskStatus::Ok
    );

    let rows = decode_i16(dec, &samples);
    let streamed = collected.into_inner().unwrap();

    assert!(!streamed.is_empty(), "the callback never fired");
    assert!(
        streamed.iter().any(|s| s.contains("JA1ABC")),
        "streamed {streamed:?} missing JA1ABC"
    );
    assert!(any_contains(&rows, "JA1ABC") && any_contains(&rows, "PM95"));
    // One well-separated candidate: streamed count and array count must
    // match exactly — no duplicate firing, no revoke-less retract.
    // (`STREAMING.md` §3's two contracts both collapse to this for a
    // single-candidate decode.)
    assert_eq!(streamed.len(), rows.len());
    unsafe { mfsk_session_close(dec) };
}

/// Setting no callback must behave exactly like not having the feature.
#[test]
fn no_callback_is_the_same_decode() {
    let samples = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1500.0);

    let a = open(MfskMode::Ft8, None);
    let plain = texts(&decode_i16(a, &samples));
    unsafe { mfsk_session_close(a) };

    let b = open(MfskMode::Ft8, None);
    assert_eq!(
        unsafe { mfsk_session_set_on_decode(b, None, ptr::null_mut()) },
        MfskStatus::Ok
    );
    let with_null = texts(&decode_i16(b, &samples));
    unsafe { mfsk_session_close(b) };

    assert!(!plain.is_empty());
    assert_eq!(plain, with_null);
}

/// Clearing the callback stops delivery — the session keeps the setting
/// across calls, so "set once" has to mean "unset once" too.
#[test]
fn clearing_the_callback_stops_delivery() {
    let samples = synth_ft8_i16("CQ", "JA1ABC", "PM95", 1500.0);
    let dec = open(MfskMode::Ft8, None);

    let collected: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let ud = &collected as *const Mutex<Vec<String>> as *mut c_void;
    unsafe { mfsk_session_set_on_decode(dec, Some(collect_cb), ud) };
    let _ = decode_i16(dec, &samples);
    let after_first = collected.lock().unwrap().len();
    assert!(after_first > 0);

    unsafe { mfsk_session_set_on_decode(dec, None, ptr::null_mut()) };
    let rows = decode_i16(dec, &samples);
    assert!(!rows.is_empty(), "the decode itself must still happen");
    assert_eq!(
        collected.lock().unwrap().len(),
        after_first,
        "the callback fired after being cleared"
    );
    unsafe { mfsk_session_close(dec) };
}

/// The callback reaches modes the pre-v2 streaming call refused: it was
/// FT8-only and returned UNKNOWN_PROTOCOL for everything else.
#[test]
fn the_callback_is_not_ft8_only_any_more() {
    let msg = mfsk_core::msg::wsjt77::pack77("CQ", "JA1ABC", "PM95").expect("pack77");
    let tones = mfsk_core::ft4::encode::message_to_tones(&msg);
    let wave = mfsk_core::ft4::encode::tones_to_i16(&tones, 1500.0, 8_000);
    let mut slot = vec![0i16; (7.5 * FS as f32) as usize];
    let start = (0.5 * FS as f32) as usize;
    for (i, s) in wave.iter().enumerate() {
        if let Some(d) = slot.get_mut(start + i) {
            *d = d.saturating_add(*s);
        }
    }

    let dec = open(MfskMode::Ft4, None);
    let collected: Mutex<Vec<String>> = Mutex::new(Vec::new());
    let ud = &collected as *const Mutex<Vec<String>> as *mut c_void;
    unsafe { mfsk_session_set_on_decode(dec, Some(collect_cb), ud) };
    let rows = decode_i16(dec, &slot);
    let streamed = collected.into_inner().unwrap();

    assert!(any_contains(&rows, "JA1ABC"), "{:?}", texts(&rows));
    assert!(
        streamed.iter().any(|s| s.contains("JA1ABC")),
        "FT4 streamed nothing: {streamed:?}"
    );
    unsafe { mfsk_session_close(dec) };
}

#[test]
fn a_null_session_is_rejected() {
    assert_eq!(
        unsafe { mfsk_session_set_on_decode(ptr::null_mut(), None, ptr::null_mut()) },
        MfskStatus::InvalidArg
    );
}
