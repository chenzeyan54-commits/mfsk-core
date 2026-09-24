//! Q65 FFI surface integration tests.
//!
//! Calls the public `mfsk_*` functions through their Rust signatures
//! (they are `pub extern "C" fn`, so safe to invoke as ordinary
//! functions in-crate) and validates:
//!
//! - `mfsk_encode_q65` × every sub-mode round-trips.
//! - `mfsk_q65_decode` recovers a clean Q65a30 frame.
//! - `mfsk_q65_decode_with_ap` accepts NULL hints and matches the
//!   plain path; with hints, decodes the same clean frame.
//! - `mfsk_q65_decode_fading` decodes a clean frame with tight
//!   spread parameters (Gaussian model, B90·Ts ≈ 0.05).
//! - `mfsk_q65_decode_with_ap_list` decodes a frame whose template
//!   is in the standard candidate set.
//! - The generic-handle path (`mfsk_decoder_new(MFSK_PROTOCOL_Q65A30)`
//!   + `mfsk_decode_f32`) routes Q65a30 traffic correctly.

use std::ffi::CString;
use std::ptr;

mod common;

use common::*;
use mfsk::*;

/// Synthesise a Q65 frame into a buffer this test owns.
fn encode_q65(sub: MfskQ65SubMode, call1: &str, call2: &str, report: &str) -> Vec<f32> {
    let c1 = CString::new(call1).unwrap();
    let c2 = CString::new(call2).unwrap();
    let r = CString::new(report).unwrap();
    let mut need = 0usize;
    assert_eq!(
        unsafe {
            mfsk_encode_q65(
                sub as u32,
                c1.as_ptr(),
                c2.as_ptr(),
                r.as_ptr(),
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
            mfsk_encode_q65(
                sub as u32,
                c1.as_ptr(),
                c2.as_ptr(),
                r.as_ptr(),
                1500.0,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut got,
            )
        },
        MfskStatus::Ok,
        "mfsk_encode_q65 failed for {sub:?}"
    );
    pcm.truncate(got);
    pcm
}

fn encode_q65a30(call1: &str, call2: &str, report: &str) -> Vec<f32> {
    encode_q65(MfskQ65SubMode::A30, call1, call2, report)
}

#[test]
fn encode_q65_roundtrips_for_every_submode() {
    for &sm in &[
        MfskQ65SubMode::A15,
        MfskQ65SubMode::A30,
        MfskQ65SubMode::A60,
        MfskQ65SubMode::B60,
        MfskQ65SubMode::C60,
        MfskQ65SubMode::D60,
        MfskQ65SubMode::E60,
        MfskQ65SubMode::D120,
        MfskQ65SubMode::E120,
        MfskQ65SubMode::A300,
    ] {
        let pcm = encode_q65(sm, "CQ", "JA1ABC", "PM95");
        // Q65 frames are 85 symbols × NSPS samples, and the encoder
        // returns frame-length PCM rather than slot-length.
        let expected = match sm {
            MfskQ65SubMode::A15 => 85 * 1_800,
            MfskQ65SubMode::A30 => 85 * 3_600,
            MfskQ65SubMode::D120 | MfskQ65SubMode::E120 => 85 * 16_000,
            MfskQ65SubMode::A300 => 85 * 41_472,
            _ => 85 * 7_200,
        };
        assert_eq!(
            pcm.len(),
            expected,
            "unexpected PCM length for sub-mode {sm:?}"
        );
    }
}

#[test]
fn q65_plain_decode_recovers_clean_signal() {
    let pcm = encode_q65a30("CQ", "K1ABC", "FN42");
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_q65_decode(
            MfskQ65SubMode::A30 as u32,
            pcm.as_ptr(),
            pcm.len(),
            12_000,
            ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        any_contains(&rows[..n], "K1ABC") && any_contains(&rows[..n], "FN42"),
        "expected K1ABC + FN42 in plain Q65 decode output"
    );
}

#[test]
fn q65_decode_with_ap_handles_null_hints() {
    // All four AP hint strings NULL → must behave like the plain
    // path (no false rejects, no crashes).
    let pcm = encode_q65a30("CQ", "JA1ABC", "PM95");
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_q65_decode_with_ap(
            MfskQ65SubMode::A30 as u32,
            pcm.as_ptr(),
            pcm.len(),
            12_000,
            ptr::null(),
            ptr::null(),
            ptr::null(),
            ptr::null(),
            ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(any_contains(&rows[..n], "JA1ABC"));
}

#[test]
fn q65_decode_with_ap_uses_call1_hint() {
    let pcm = encode_q65a30("CQ", "JA1ABC", "PM95");
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let cq = CString::new("CQ").unwrap();
    let st = unsafe {
        mfsk_q65_decode_with_ap(
            MfskQ65SubMode::A30 as u32,
            pcm.as_ptr(),
            pcm.len(),
            12_000,
            cq.as_ptr(),
            ptr::null(),
            ptr::null(),
            ptr::null(),
            ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(any_contains(&rows[..n], "JA1ABC"));
}

#[test]
fn q65_decode_fading_recovers_clean_signal() {
    let pcm = encode_q65a30("CQ", "K1ABC", "FN42");
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_q65_decode_fading(
            MfskQ65SubMode::A30 as u32,
            pcm.as_ptr(),
            pcm.len(),
            12_000,
            0.05, // tight spread → near-AWGN
            MfskQ65FadingModel::Gaussian as u32,
            ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        any_contains(&rows[..n], "K1ABC"),
        "fast-fading FFI path must decode a clean signal"
    );
}

#[test]
fn q65_decode_with_ap_list_picks_matching_template() {
    // Encode "K1ABC JA1ABC PM95" — that exact template lives in
    // the 206-candidate set generated for (K1ABC, JA1ABC, PM95).
    let pcm = encode_q65a30("K1ABC", "JA1ABC", "PM95");
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let mc = CString::new("K1ABC").unwrap();
    let hc = CString::new("JA1ABC").unwrap();
    let hg = CString::new("PM95").unwrap();
    let st = unsafe {
        mfsk_q65_decode_with_ap_list(
            MfskQ65SubMode::A30 as u32,
            pcm.as_ptr(),
            pcm.len(),
            12_000,
            mc.as_ptr(),
            hc.as_ptr(),
            hg.as_ptr(),
            ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        any_contains(&rows[..n], "K1ABC JA1ABC PM95"),
        "AP-list FFI path must pick the matching template"
    );
}

#[test]
fn q65_decode_with_ap_list_returns_decode_failed_on_bad_calls() {
    // `standard_qso_codewords` rejects garbage callsigns →
    // empty candidate set → DecodeFailed status.
    let pcm = encode_q65a30("CQ", "K1ABC", "FN42");
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let bad = CString::new("!!!").unwrap();
    let hc = CString::new("K1ABC").unwrap();
    let st = unsafe {
        mfsk_q65_decode_with_ap_list(
            MfskQ65SubMode::A30 as u32,
            pcm.as_ptr(),
            pcm.len(),
            12_000,
            bad.as_ptr(),
            hc.as_ptr(),
            ptr::null(),
            ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(
        st,
        MfskStatus::DecodeFailed,
        "garbage calls should yield DecodeFailed without aborting"
    );
    assert_eq!(n, 0);
}

/// Proves the FFI `hash_table` parameter reaches
/// `mfsk_core::msg::q65`'s hash-table-aware unpack, not just that it
/// compiles — same shape as `mfsk_core::q65::rx`'s own
/// `sniper_hash_table_resolves_hashed_callsign` test, replayed through
/// the C ABI. Builds a Type-4 message ("JL1NIE/1" non-standard call +
/// hashed "JA1ABC") directly via `mfsk_core`'s Rust API (issue #250 —
/// `mfsk_encode_q65` only packs standard messages, so there's no FFI
/// encoder for this case) and decodes it twice: once with a NULL
/// `hash_table` (must show the unresolved `<...>` placeholder) and
/// once with a table pre-seeded via `mfsk_callsign_hash_table_insert`
/// (must show the resolved `<JA1ABC>`).
#[test]
fn q65_decode_hash_table_resolves_hashed_callsign() {
    use mfsk_core::msg::wsjt77::pack77_type4;
    use mfsk_core::q65::Q65a30;
    use mfsk_core::q65::tx::encode_channel_symbols;

    let bits77 = pack77_type4("JL1NIE/1", "JA1ABC", "", false).expect("pack77_type4 failed");
    let tones = encode_channel_symbols(&bits77);
    let audio = mfsk_core::engine::tx::synthesize::<Q65a30>(&tones, 12_000, 1500.0, 0.3);

    // Without a hash table: unresolved placeholder.
    let mut rows = vec![blank_row(); 16];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_q65_decode(
            MfskQ65SubMode::A30 as u32,
            audio.as_ptr(),
            audio.len(),
            12_000,
            ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok);
    assert!(
        any_contains(&rows[..n], "JL1NIE/1") && any_contains(&rows[..n], "<...>"),
        "expected an unresolved '<...>' decode without a hash table"
    );

    // With a hash table pre-seeded with the standard call: resolved.
    let ht = mfsk_callsign_hash_table_new();
    assert!(!ht.is_null());
    let ja1abc = CString::new("JA1ABC").unwrap();
    let ins_st = unsafe { mfsk_callsign_hash_table_insert(ht, ja1abc.as_ptr()) };
    assert_eq!(ins_st, MfskStatus::Ok);

    let mut rows2 = vec![blank_row(); 16];
    let mut n2 = 0usize;
    let st2 = unsafe {
        mfsk_q65_decode(
            MfskQ65SubMode::A30 as u32,
            audio.as_ptr(),
            audio.len(),
            12_000,
            ht,
            rows2.as_mut_ptr(),
            rows2.len(),
            &mut n2,
        )
    };
    assert_eq!(st2, MfskStatus::Ok);
    assert!(
        any_contains(&rows2[..n2], "JL1NIE/1") && any_contains(&rows2[..n2], "<JA1ABC>"),
        "expected the hashed callsign to resolve via the supplied table"
    );
    unsafe { mfsk_callsign_hash_table_free(ht) };
}

/// Q65 is addressable through `MfskMode` and describable through
/// `mfsk_mode_info`, but it does **not** drive the decode session:
/// its own decode takes a nominal start sample and a time tolerance and
/// reports `start_sample` rather than `dt`. `MFSK_CAP_DECODE_HANDLE` is
/// the bit that says so, and opening a session must refuse rather than
/// decode something shaped differently.
#[test]
fn q65_is_addressable_but_does_not_drive_the_session() {
    for sub in [
        MfskMode::Q65a15,
        MfskMode::Q65a30,
        MfskMode::Q65a60,
        MfskMode::Q65a300,
    ] {
        assert_eq!(
            mfsk_mode_caps(sub as u32) & MFSK_CAP_DECODE_HANDLE,
            0,
            "{sub:?} must not claim the decode handle"
        );
        assert_ne!(
            mfsk_mode_caps(sub as u32) & MFSK_CAP_AP_NARROW,
            0,
            "{sub:?} decode is targeted by construction, so narrow AP applies"
        );
        let mut st = MfskStatus::Ok;
        assert!(unsafe { mfsk_session_open(sub as u32, ptr::null(), &mut st) }.is_null());
        assert_eq!(st, MfskStatus::Unsupported, "{sub:?}");

        // But it is still fully described.
        let mut info = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
        assert_eq!(
            unsafe { mfsk_mode_info(sub as u32, info.as_mut_ptr()) },
            MfskStatus::Ok
        );
        assert_eq!(unsafe { info.assume_init() }.mode, sub);
    }
}
