//! The message-acceptance policy on the **generic** pipeline
//! (issue #383, step 5a).
//!
//! FT8 reaches its text stage inside its own bespoke engine; FT4 and
//! every FST4 sub-mode reach theirs through
//! `engine::pipeline::InfoAccept`, a seam that exists because `engine`
//! cannot depend on `msg` and so cannot unpack a codeword itself. This
//! file holds that seam down from the outside:
//!
//! 1. **The default is bit-identical**, and for FT4/FST4 that is a
//!    stronger claim than for FT8: `MESSAGE_FILTER_DEFAULT` is `false`
//!    for FST4, so the seam must not even build the message string.
//!    `PolicyAccept` folds itself away on two compile-time constants;
//!    the behavioural half is checked here.
//! 2. **`.message_filter(|_| false)` reaches every rung.** Not a
//!    tautology: the pipeline accepts candidates at four separate
//!    points (BP ladder, OSD-2/3, OSD-4 Top-K, a-priori), and an
//!    unwired one would leak rows.
//! 3. **`.also_accept()` turns the codec verdict on** for a protocol
//!    that does not filter by default — the semantics that let
//!    `.also_accept(f)` mean "the usual filter, plus this" everywhere.
//! 4. **`.sic_rounds()` carries the policy too.** FT4's SIC path runs
//!    a second engine; before this it called the `AcceptAll` wrapper
//!    and would have ignored the caller silently.
//!
//! Run:
//! ```sh
//! MFSK_REQUIRE_CORPUS=1 cargo test --release -p mfsk-core \
//!     --features full,internal-testing --test ft4_message_policy
//! ```
#![cfg(all(feature = "fft-rustfft", feature = "ft4"))]

use mfsk_core::ft4::Ft4;
use mfsk_core::msg::decode_request::{DecodeOutcome, DecodeRequest};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000; // 7.5 s @ 12 kHz

fn slot_audio() -> Option<Vec<i16>> {
    let path = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    )?;
    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

fn req(audio: &[i16]) -> DecodeRequest<'_, Ft4> {
    DecodeRequest::<Ft4>::new(audio, 300.0, 2800.0, 1.2, 100)
}

fn rows(out: &DecodeOutcome<Ft4>) -> Vec<String> {
    let mut v: Vec<String> = out
        .results
        .iter()
        .map(|r| {
            format!(
                "{}|{:.3}|{:.3}|{}",
                unpack77(r.message77()).unwrap_or_default(),
                r.freq_hz,
                r.dt_sec,
                r.pass
            )
        })
        .collect();
    v.sort();
    v
}

#[test]
fn a_no_op_policy_changes_nothing() {
    let Some(a) = slot_audio() else {
        common::corpus::missing("ft4_message_policy", "ft4/000000_000002.wav");
        return;
    };
    let base = rows(&req(&a).decode());
    assert!(!base.is_empty(), "the golden recording must decode");

    // FT4 does not apply the verdict by default, so opting out of
    // everything is the default.
    let all = rows(&req(&a).message_filter(|_| true).decode());
    assert_eq!(base, all, "FT4 must not be filtering by default");

    // And opting *in* must cost it nothing on this recording — half of
    // it is ARRL RTTY Roundup, which the verdict refused outright until
    // issue #383 gave the type a structural rule. That regression would
    // be invisible without this assertion, because FT4 never ran the
    // verdict before.
    let verdict = rows(&req(&a).codec_filter().decode());
    assert_eq!(base, verdict, "the codec verdict dropped an FT4 decode");

    let widened = rows(&req(&a).also_accept(|_| false).decode());
    assert_eq!(
        base, widened,
        "also_accept over a passing verdict is a no-op"
    );
}

/// Four acceptance points in `process_candidate_basic_impl`; an unwired
/// one leaks rows here.
#[test]
fn a_rejecting_filter_reaches_every_rung() {
    let Some(a) = slot_audio() else {
        common::corpus::missing("ft4_message_policy", "ft4/000000_000002.wav");
        return;
    };
    let none = req(&a).message_filter(|_| false).decode();
    assert!(
        none.results.is_empty(),
        "a rung is not wired: {} rows survived a reject-all filter",
        none.results.len()
    );

    // The SIC strategy runs its own engine — it used to call the
    // `AcceptAll` wrapper and would have ignored the caller.
    let none_sic = req(&a).message_filter(|_| false).sic_rounds(2).decode();
    assert!(
        none_sic.results.is_empty(),
        "the SIC path ignored the policy: {} rows",
        none_sic.results.len()
    );
}

#[test]
fn a_permissive_filter_can_only_add() {
    let Some(a) = slot_audio() else {
        common::corpus::missing("ft4_message_policy", "ft4/000000_000002.wav");
        return;
    };
    let base = rows(&req(&a).decode());
    let all = rows(&req(&a).message_filter(|_| true).decode());
    for r in &base {
        assert!(all.contains(r), "removing the filter lost {r}");
    }
    assert!(all.len() >= base.len());
}
