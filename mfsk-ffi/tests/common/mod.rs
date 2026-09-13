//! Shared helpers for the C-ABI tests.
//!
//! The v2 surface hands rows to caller memory and takes one parameter
//! struct, which is better for a C consumer and slightly wordier from
//! Rust. These wrappers keep the tests about behaviour rather than
//! about marshalling.
#![allow(dead_code)]

use mfsk::*;

pub const FS: u32 = 12_000;

/// A row the library will overwrite. `size` is the caller's half of the
/// growth contract; everything else is don't-care.
pub fn blank_row() -> MfskDecode {
    MfskDecode {
        size: std::mem::size_of::<MfskDecode>() as u32,
        mode: MfskMode::Ft8,
        text: [0; MFSK_DECODE_TEXT_LEN],
        freq_hz: 0.0,
        dt_sec: 0.0,
        snr_db: 0.0,
        sync_score: 0.0,
        sync_cv: 0.0,
        hard_errors: 0,
        info_bits: 0,
        pass: 0,
        flags: 0,
    }
}

/// `mode`'s published defaults.
pub fn params(mode: MfskMode) -> MfskDecodeParams {
    let mut p = std::mem::MaybeUninit::<MfskDecodeParams>::zeroed();
    assert_eq!(
        unsafe { mfsk_decode_params_init(mode as u32, p.as_mut_ptr()) },
        MfskStatus::Ok,
        "{mode:?} has no defaults to initialise from"
    );
    unsafe { p.assume_init() }
}

/// Open a session, asserting it succeeded.
pub fn open(mode: MfskMode, p: Option<&MfskDecodeParams>) -> *mut MfskDecodeSession {
    let mut st = MfskStatus::Internal;
    let d = unsafe {
        mfsk_session_open(
            mode as u32,
            p.map(|p| p as *const _).unwrap_or(std::ptr::null()),
            &mut st,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "{mode:?} failed to open");
    assert!(!d.is_null());
    d
}

pub fn text_of(r: &MfskDecode) -> String {
    let b: &[u8] =
        unsafe { std::slice::from_raw_parts(r.text.as_ptr() as *const u8, r.text.len()) };
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

/// Decode i16 PCM through a session, returning the rows.
pub fn decode_i16(dec: *mut MfskDecodeSession, audio: &[i16]) -> Vec<MfskDecode> {
    decode_i16_with(dec, audio, None)
}

pub fn decode_i16_with(
    dec: *mut MfskDecodeSession,
    audio: &[i16],
    p: Option<&MfskDecodeParams>,
) -> Vec<MfskDecode> {
    let mut rows = vec![blank_row(); 64];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_session_decode_i16(
            dec,
            audio.as_ptr(),
            audio.len(),
            FS,
            p.map(|p| p as *const _).unwrap_or(std::ptr::null()),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "decode failed");
    rows.truncate(n);
    rows
}

pub fn decode_f32(dec: *mut MfskDecodeSession, audio: &[f32]) -> Vec<MfskDecode> {
    let mut rows = vec![blank_row(); 64];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_session_decode_f32(
            dec,
            audio.as_ptr(),
            audio.len(),
            FS,
            std::ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "decode failed");
    rows.truncate(n);
    rows
}

pub fn texts(rows: &[MfskDecode]) -> Vec<String> {
    rows.iter().map(text_of).collect()
}

pub fn any_contains(rows: &[MfskDecode], needle: &str) -> bool {
    rows.iter().any(|r| text_of(r).contains(needle))
}

/// Set an AP hint on a params struct.
pub fn with_ap(p: &mut MfskDecodeParams, call1: &str, call2: &str, grid: &str) {
    fn put(dst: &mut [std::ffi::c_char], s: &str) {
        let b = s.as_bytes();
        let n = b.len().min(dst.len() - 1);
        for (d, &c) in dst.iter_mut().zip(&b[..n]) {
            *d = c as std::ffi::c_char;
        }
        dst[n] = 0;
    }
    p.has_ap_hint = true;
    put(&mut p.ap_call1, call1);
    put(&mut p.ap_call2, call2);
    put(&mut p.ap_grid, grid);
}
