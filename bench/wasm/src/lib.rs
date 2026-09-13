//! Thin wasm-bindgen wrapper around mfsk-core's FT8 decode entry point,
//! for the Node-based `+simd128` before/after benchmark. Not a real API
//! surface — see `bench/wasm/README.md`.

use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;
use wasm_bindgen::prelude::*;

/// Decode one FT8 slot from mono 16-bit PCM samples.
///
/// Mirrors `mfsk-core/tests/ft8_sweep.rs`'s `decode_wav_ft8` call shape
/// (`DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 50)`, OSD on by
/// default) so the benchmark exercises the same pipeline the crate's own
/// tests do.
///
/// Returns one `message|freq_hz|dt_sec` line per decode, newline-joined —
/// good enough for `bench.mjs` to print/diff, not a real API contract.
#[wasm_bindgen]
pub fn decode_wav(audio: &[i16]) -> String {
    let outcome = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 50).decode();

    outcome
        .results
        .iter()
        .map(|d| {
            let msg = unpack77(d.message77()).unwrap_or_default();
            format!("{msg}|{:.1}|{:.2}", d.freq_hz, d.dt_sec)
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Same as [`decode_wav`] but via `.sic_early()` (the staged-checkpoint
/// multipass strategy `decode_phase2`-style profiles use) — for
/// comparing single-pass vs multipass decode cost under
/// `wasm32-unknown-unknown` specifically (this crate is built with
/// `default-features = false` and no `parallel`, so this strategy's
/// 3-checkpoint structure has no rayon to hide its per-pass
/// coarse-sync/FFT-cache-rebuild cost behind — see issue #246 for the
/// measured ratio vs [`decode_wav`], both under Node and native).
#[wasm_bindgen]
pub fn decode_wav_subtract(audio: &[i16]) -> String {
    let outcome = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 50)
        .sic_early()
        .decode();

    outcome
        .results
        .iter()
        .map(|d| {
            let msg = unpack77(d.message77()).unwrap_or_default();
            format!("{msg}|{:.1}|{:.2}", d.freq_hz, d.dt_sec)
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// One JS clock call. `mfsk-core` holds no clock of its own —
/// `std::time::Instant::now` is unimplemented on
/// `wasm32-unknown-unknown` and would panic — so the budget predicate
/// below reaches for the host's.
///
/// `inline_js` rather than a `js-sys`/`web-time` dependency: one
/// function is not worth a crate, and this keeps the harness's
/// dependency list honest about what the benchmark measures.
#[wasm_bindgen(inline_js = "export function now_ms() { return Date.now(); }")]
extern "C" {
    fn now_ms() -> f64;
}

/// [`decode_wav`] under a wall-clock budget — the browser case
/// `DecodeRequest::budget` exists for: a tab that must stay responsive
/// cannot afford the worst slot's decode, and configuring `max_cand`
/// down for the worst case pays that cost on every slot instead.
///
/// Returns the same `message|freq_hz|dt_sec` lines, preceded by one
/// `#budget|…` line carrying the `BudgetReport`, so a caller can see
/// whether the cut took noise (a low `cut_at_sync`) or a station.
///
/// The decode degrades *cheapest-first*: every candidate gets the cheap
/// sync triage, and the budget is then spent on the survivors strongest
/// first — so a short budget returns the loudest stations, not the
/// low-frequency end of the band.
#[wasm_bindgen]
pub fn decode_wav_budget(audio: &[i16], budget_ms: f64) -> String {
    let deadline = now_ms() + budget_ms;
    let check = || now_ms() < deadline;

    let outcome = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 50)
        .budget(&check)
        .decode();

    let b = &outcome.budget;
    let mut lines = alloc_report(b);
    for d in &outcome.results {
        let msg = unpack77(d.message77()).unwrap_or_default();
        lines.push(format!("{msg}|{:.1}|{:.2}", d.freq_hz, d.dt_sec));
    }
    lines.join("\n")
}

fn alloc_report(b: &mfsk_core::msg::decode_request::BudgetReport) -> Vec<String> {
    vec![format!(
        "#budget|exhausted={}|ran={}|skipped={}|cut_at_sync={}",
        b.exhausted,
        b.stages_run,
        b.candidates_skipped,
        b.cut_at_sync
            .map(|v| v.to_string())
            .unwrap_or_else(|| "-".into()),
    )]
}
