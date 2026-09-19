//! WAV-fed decode pipeline for CoreS3 (Phase 0-Core).
//!
//! Structurally identical to `m5stack-core2-app/src/decode_pipeline.rs`.
//! All heavy lifting is in `embedded_shared` and `mfsk_app_shared` which
//! are board-agnostic. Board-specific touch points are `crate::log_free_internal`
//! and the `QSO_WAVS` slice (same qso3_busy.wav reference as Core2).
//! Phase 1-Core replaces the wav_sim source with a UAC host capture.

extern crate alloc;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{DEFAULT_Q_THRESH, NFFT_SPEC};
use core::fmt::Write as _;

use mfsk_core::msg::wsjt77::unpack77;

use embedded_shared::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};
use esp_idf_svc::sys::QueueHandle_t;

use mfsk_app_shared::qso::{self, QsoManager, QsoState};
use mfsk_app_shared::ui::state::{DecodedRow, UI};

/// Operator identity, from `cfg.toml`'s `[station]` section.
///
/// These were literals until 2026-08-23 — this repository shipped one
/// operator's callsign compiled into the source, so anyone else's build
/// identified as them. `m5stack-s3-app` has taken them from `cfg.toml`
/// since Phase 1.7; this crate was never brought across.
///
/// Empty is a valid value and leaves the QSO FSM idle, which is the
/// right behaviour for a receiver with no operator configured.
const MY_CALL: &str = env!("MY_CALL");
const MY_GRID: &str = env!("MY_GRID");

/// The baked FT8 slot. `pub` so the `MFSK_CORES3_SIM` harness can feed
/// it through `Ft8ChunkSink` (the real UAC sink) instead of the direct
/// `wav_sim` path.
pub static QSO_WAVS: &[&[u8]] = &[include_bytes!("../../assets/qso3_busy.wav")];

/// Pass-1 candidate cap and refined-candidate cap.
///
/// The embedded FT8 decode is deliberately lean — single-pass BP,
/// `LlrEffort::Minimal`, **no OSD, no SIC, no AP** (`stage3_split` →
/// `process_candidates_with_ap` runs one pass and never touches the raw
/// audio a subtract would need). It decodes 7 on qso3_busy where host
/// JTDX gets ~18; that gap is the cost of the leanness, and it is the
/// right trade for battery-budgeted field operation where a bounded,
/// phantom-free slot matters more than the last few dB (the phantom
/// bugs this suite has shipped were all in the subtraction paths this
/// config does not use). Not a bug to chase.
///
/// Compile-time knobs, kept for #357 investigation only —
/// `MFSK_FT8_MAX_CAND` / `MFSK_FT8_PASS1_LIMIT`. Defaults are what
/// ships.
const PASS1_LIMIT: usize = match option_env!("MFSK_FT8_PASS1_LIMIT") {
    Some(s) => parse_u32(s) as usize,
    None => 30,
};
const MAX_CAND: usize = match option_env!("MFSK_FT8_MAX_CAND") {
    Some(s) => parse_u32(s) as usize,
    None => 15,
};

/// Wall-clock budget for stage 3, milliseconds from the SpecBundle
/// arriving (#357). Bounds the worst-case slot so a dense period cannot
/// overrun and steal the next slot's headroom — the failure the live
/// radio showed (transmit-heavy period ~0.7 s past slot end, 8–11
/// candidates deferred and dropped, 0–2 decoded against the other
/// period's 4–8).
///
/// **1836 ms, derived from key-up — the same move FT4's
/// `TX_TURNAROUND_BUDGET_MS` made** (`embedded-poc/embedded-shared/src/
/// apps/ft4_rx.rs`), not a flat number chosen from a sweep. This
/// receiver does not transmit yet, but running the constraint a
/// QSO-capable build will have means the number on screen is the one
/// that stays true then, same rationale as FT4's own doc comment.
///
/// Within a slot beginning at 0, `TX_START_OFFSET_S = 0.5`:
///
/// ```text
///   0.50 s  the other station's transmission starts
///  13.14 s  its frame ends (79 symbols x 0.16 s)
///  14.14 s  ...plus the ±1.0 s `EMBEDDED_SYNC_LAG_S` reach
///  15.00 s  slot boundary
///  15.50 s  THIS station's transmission must start
/// ```
///
/// **But this deadline isn't anchored to that 14.14 s point — it's
/// anchored to `t_post_recv`, the SpecBundle's arrival**, which the
/// streaming pipeline (`stage1_inc`'s `SPEC_EMIT_PAIR`) already fires
/// well before slot end. Measured directly (`tail_win = slotend -
/// t_post_recv`, aligned steady-state slots, `logs/hw_*_2026-09-05.log`):
/// consistently 1336-1350 ms, no acquisition or grid-shift dependence
/// seen. So the budget from that real anchor to key-up is
/// `500 + 1336 = 1836` ms (the smallest observed `tail_win`, i.e. the
/// least slack seen) — **less than the old 2000 ms, and the old number
/// was never measuring this**: it was chosen from the qso3_busy sweep
/// (`logs/ft8_357_bud*`, 2026-09-04) where stage 3 measures ~985 ms and
/// the recall-vs-budget curve is flat at `dec=7` down to 800 ms (the
/// deadline sheds only the doomed tail — candidates run in descending
/// coarse score and the all-LLR-variant BP failures are last). 1836 ms
/// is still ~1.9x that measured work and clears the 800 ms floor with
/// margin, so this derivation doesn't cost anything the sweep would
/// show — it just replaces an arbitrary safety factor with the actual
/// constraint. `0` disables it; `MFSK_FT8_BUDGET_MS=` overrides.
/// Pending confirmation on a live radio (#357).
///
/// **What this does not do**: FT8's own audio capture still waits for
/// the full 15.0 s slot (`Ft8ChunkSink`'s `SLOT_SAMPLES_12K`), unlike
/// FT4's `CAPTURE_CLOSE_SAMPLES` early-close. The 0.86 s tail beyond
/// `EMBEDDED_SYNC_LAG_S`'s reach (14.14 s) is real slack, but
/// `Ft8ChunkSink`'s slot boundary is load-bearing for grid-lock (#356)
/// and cold acquisition (#358); shortening it needs the same care FT4's
/// `want_skip` carry-forward took, not a quick constant change. Left
/// alone here on purpose.
/// Per-candidate fine sync (WSJT-X `ft8b.f90` Stages A/B/C) plus a
/// coarse-position retry — `dual_core::DecodeConfig::fine_sync`, whose
/// doc carries the measurement. On unless `MFSK_FT8_FINE_SYNC=0`, which
/// is there to A/B the cost on this board against the same audio.
const FT8_FINE_SYNC: bool = match option_env!("MFSK_FT8_FINE_SYNC") {
    Some(s) => parse_u32(s) != 0,
    None => true,
};

/// `dual_core::DecodeConfig::key_up_guard_ms`: stop claiming stage-3
/// candidates this long before key-up (slot end + 0.5 s), so a candidate
/// already in BP when the bound hits still finishes before it. 320 ms
/// clears the largest run-on past a stopping deadline measured on this
/// board, 313 ms over three captures and 60 slots (2026-09-18,
/// `logs/sim_finesync_{fullslotvote,offset3000,noclock_offset3000}_*`).
/// `MFSK_FT8_KEY_UP_GUARD_MS=0` turns it off, leaving `FT8_BUDGET_MS`.
const FT8_KEY_UP_GUARD_MS: i64 = match option_env!("MFSK_FT8_KEY_UP_GUARD_MS") {
    Some(s) => parse_u32(s) as i64,
    None => 320,
};

/// `dual_core::DecodeConfig::slot_floor_ms`: the least time a slot may
/// have before key-up and still be worth starting.
///
/// 500 ms. `coarse` and fine sync run outside every deadline this
/// pipeline has, and on this board they measure 100-120 ms and 250-275
/// ms — ~370 ms together, to which one stage-3 candidate adds ~50-300
/// ms. A steady slot's bundle arrives 1.45 s before its boundary, i.e.
/// 1.63 s before the [`FT8_KEY_UP_GUARD_MS`]-adjusted key-up, so the
/// floor is nowhere near it; the slot after a cold acquisition arrived
/// with 354 ms and overran key-up by 763-1344 ms on every run of
/// 2026-09-18. Anything between ~0.4 s and ~1.3 s separates the two
/// cases; 500 ms is the low end of that, so the floor never costs a
/// slot that could have decoded. `MFSK_FT8_SLOT_FLOOR_MS=0` turns it
/// off.
/// `dual_core::DecodeConfig::fine_sync_late` — on unless
/// `MFSK_FT8_FINE_SYNC_LATE=0`. Its 292 ms does not fit the ~1.1 s
/// before key-up (5.50 decodes a slot with it there against 6.00
/// without, 2026-09-19); on the idle tail after key-up it has thirteen
/// seconds and what it adds — the marginal stations — is next period's
/// contact list rather than this period's reply.
const FT8_FINE_SYNC_LATE: bool = match option_env!("MFSK_FT8_FINE_SYNC_LATE") {
    Some(s) => parse_u32(s) != 0,
    None => true,
};

/// `dual_core::DecodeConfig::share_cand_budget` — off unless
/// `MFSK_FT8_SHARE_CAND=1`, pending the board measurement its doc asks
/// for (the mirror's gain is on `qso1`/`qso2`, and it is spent after
/// SlotEnd where this board has ~180 ms).
const FT8_SHARE_CAND: bool = match option_env!("MFSK_FT8_SHARE_CAND") {
    Some(s) => parse_u32(s) != 0,
    None => false,
};

const FT8_SLOT_FLOOR_MS: i64 = match option_env!("MFSK_FT8_SLOT_FLOOR_MS") {
    Some(s) => parse_u32(s) as i64,
    None => 500,
};

/// `dual_core::DecodeConfig::slot_end_hint` — when the slot now being
/// decoded ends, on `esp_timer_get_time()`'s clock.
///
/// Read from the capture-slot boundary the **audio sink** publishes
/// (`time_sync::publish_capture_slot`, called from `Ft8ChunkSink` as it
/// emits `SlotEnd`). That is the one clock here that cannot fall
/// behind: it advances with arriving audio, whatever the decode task
/// and stage1_inc are doing. Anything derived from the SpecBundle
/// instead slides late exactly when the decode task is busy — measured,
/// and the reason this function exists (see `SpecBundle::emit_us`).
///
/// While the sink is still capturing the slot being decoded its
/// boundary is ahead; once the sink has moved on, that slot ended
/// `into` ago. `time_sync::decoded_slot_end_us` picks between the two
/// at half a slot — deliberately far from both readings, because an
/// earlier version split at the emit point itself and a few ms of
/// jitter there flipped the answer by a whole slot (it threw away
/// every other slot on hardware; see that function).
///
/// The length comes from the sink too, so the one slot a cold
/// acquisition lengthens is measured as the longer slot it is instead
/// of losing its extra seconds of tail. Before the sink has published
/// a length — and on a board whose sink publishes none — the nominal
/// slot stands in.
fn slot_end_hint() -> Option<i64> {
    /// One FT8 slot in µs.
    const SLOT_US: i64 = 15_000_000;
    let now = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let (_, into, len) = mfsk_app_shared::time_sync::current_capture_info_with_len(now)?;
    Some(mfsk_app_shared::time_sync::decoded_slot_end_us(
        now,
        into,
        len.unwrap_or(SLOT_US),
    ))
}

const FT8_BUDGET_MS: i64 = match option_env!("MFSK_FT8_BUDGET_MS") {
    Some(s) => parse_u32(s) as i64,
    None => 1_836,
};

/// `const`-context unsigned parse — `str::parse` is not `const`. Digits
/// only; anything else is a build-time panic, which is what you want
/// for a typo in a sweep env var that would otherwise silently fall
/// back to the default.
const fn parse_u32(s: &str) -> u32 {
    let b = s.as_bytes();
    let mut i = 0;
    let mut v: u32 = 0;
    while i < b.len() {
        assert!(b[i] >= b'0' && b[i] <= b'9', "MFSK_FT8_* knob: digits only");
        v = v * 10 + (b[i] - b'0') as u32;
        i += 1;
    }
    v
}

/// `BootMode::Decode` entry — runs the decode pipeline with the baked
/// `QSO_WAVS` playlist as the audio source. Thin wrapper around
/// [`run_with_source`].
pub fn run() -> ! {
    run_with_source("wav", |q| wav_sim::spawn(QSO_WAVS, q))
}

/// Source-agnostic entry. Allocates the pipeline queues, spawns
/// `stage1_inc` + `wf_drain`, calls `source_spawn` (which must push
/// `ChunkMsg::Samples` + `ChunkMsg::SlotEnd` into the chunk queue),
/// then runs the decode loop. Never returns.
///
/// `Decode` mode passes `|q| wav_sim::spawn(QSO_WAVS, q)`.
/// `Uac` mode passes `|q| uac::set_chunk_q(q)` — the UAC reader thread
/// starts pushing once it sees the queue handle land in its static slot.
pub fn run_with_source<F: FnOnce(QueueHandle_t)>(source: &'static str, source_spawn: F) -> ! {
    crate::log_free_internal("pre-decode-loop (post-Goertzel: no BASIS alloc)");

    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    esp_dsp_fft::prewarm(NFFT_SPEC);
    dual_core::init();

    let chunk_q = pipeline::create_chunk_queue(4);
    let slot_q = pipeline::create_slot_queue(2);
    let spec_q = pipeline::create_spec_queue(2);
    let wf_q = pipeline::create_wf_queue(8);
    stage1_inc::spawn_with_wf(chunk_q, slot_q, spec_q, Some(wf_q));
    source_spawn(chunk_q);

    let wf_q_addr = wf_q as usize;
    crate::board::spawn_named(c"wf_drain", 4 * 1024, move || {
        wf_drain(wf_q_addr as esp_idf_svc::sys::QueueHandle_t)
    })
    .expect("spawn wf drainer");

    log::info!("decode pipeline ready (q_thresh={DEFAULT_Q_THRESH}, band 200..3000 Hz, cores3-app phase 0)");

    let mut qso = QsoManager::new(MY_CALL, MY_GRID);
    let initial = qso.call_cq(None);
    push_tx_line(&qso, Some(&initial));

    let mut slot_seq: u32 = 0;
    // Lock state and the under-par run that sends the receiver back
    // for a new grid (#356). The policy — and its host tests — live in
    // `mfsk_app_shared::grid_state`.
    let mut grid = mfsk_app_shared::grid_state::GridState::new();
    /// Coarse-candidate cap for *each* of `acquire_slot_phase`'s three
    /// tiled windows. Deliberately **not** `MAX_CAND` (15, the decode
    /// candidate cap) — `MFSK_CORES3_SIM` caught reusing it here: with
    /// only 15 pass1 candidates per ±2.5 s window, `qso3_busy`'s real
    /// signals get crowded out of the top-15 as often as not, and the
    /// circular estimate came back at `R 0.39` on a 4 s offset the
    /// phase was otherwise recovered correctly for (`dt -3.6 s`). 200 matches `tests/ft8_cold_acquisition.rs`,
    /// where the measurement that chose the tiled search over the wide
    /// one used it. One-time cost — this only runs during acquisition,
    /// never in the per-slot decode loop.
    const ACQUIRE_MAX_CAND: usize = 200;
    /// Candidate phases the acquisition tries before giving up and
    /// waiting for the next capture.
    ///
    /// Five. `ft8_cold_acquisition_fixed::cluster_then_decode_acquisition`,
    /// 40 offsets across the period on the receiver's own numeric path,
    /// finds a usable phase in the first cluster 23 times, within three
    /// 37 times, and within five every time — nothing needed a sixth.
    /// Each trial is one `decode_block_tuned` over the captured slot,
    /// so the cost is bounded by five of those, and only when
    /// acquisition runs, which is when the receiver is decoding nothing
    /// anyway.
    const ACQUIRE_MAX_TRIALS: usize = 5;
    /// One slot at 12 kHz — the window the acceptance trial decodes.
    const SLOT_TRIAL_SAMPLES: usize = 180_000;
    // **Why the correction is the median and not the earliest.**
    //
    // A frame that starts before the capture window is clipped and one
    // that ends early is not, so aligning on the *earliest* decode
    // looks right and measures right — until the measurement uses the
    // wrong search width. On the block driver's own +-2.5 s coarse
    // search it wins by 0.8 decodes a slot and reaches this
    // recording's best phase (`mfsk-core/tests/ft8_embedded_grid_
    // phase.rs`, 2026-09-16). But the per-slot path is not that
    // driver: `stage1_inc` runs `SYNC_LAG_S = 1.0`, and the earliest
    // rule pushes the station spread to +0.45..+1.75 s, where its late
    // half is outside the window the steady loop can see. Counting
    // only decodes at |dt| <= 1.0, the same sweep puts that phase at
    // **1** decode against the median rule's landing at **7**.
    //
    // Which is what this file already said, one screen down: the trial
    // searches +-2.5 s, the per-slot path does not, and the median
    // correction is what decouples them. Left here as well because the
    // earliest rule is the obvious-looking change, it was made, and
    // only a sweep that modelled the narrow window caught it.
    //
    // **And not some other average of the same sample.** The median is
    // taken over whichever stations one trial decoded, and a band's
    // stations do not share a clock (`qso3_busy` spreads 1.07 s end to
    // end), so the statistic moves with the mix. Four alternatives were
    // scored on the steady pipeline's own decodes over 24 capture
    // phases per recording, 2026-09-18
    // (`ft8_embedded_pipeline_mirror::mirror_acquisition_dt_statistic`):
    //
    // ```text
    //             qso3            qso1    qso2     landing sd (qso3/1/2)
    //   median    8.00 (88% >=8)  3.88    4.74     0.204 / 0.014 / 0.027
    //   midrange  8.38 (88%)      3.17    4.74     0.167 / 0.017 / 0.043
    //   trimmed   7.29 (38%)      3.88    4.39     0.188 / 0.013 / 0.035
    //   snrtop    8.04 (79%)      3.88    4.74     0.205 / 0.021 / 0.033
    //   pooled    7.96 (92%)      3.83    4.70     0.203 / 0.025 / 0.022
    // ```
    //
    // Nothing beats the median on more than one recording — midrange's
    // +0.38 on `qso3` is -0.71 on `qso1`, i.e. it lands 0.4 s earlier
    // and that happens to suit this one slot. The scatter the table
    // reports is not the mix either: on `qso3` all but one capture
    // phase land inside +0.10..+0.26 s, and the outlier is the one
    // trial that decoded a single station. Between `qso1` and `qso2`,
    // adjacent slots of one real session, the landing moves 29 ms.
    //
    // It costs little in any case: the phase response is flat at 7-9
    // decodes from -0.40 to +0.40 s, so the +-0.2 s this lands within
    // is worth about one station, and a landing bad enough to matter
    // decodes under `LOCK_MIN_DECODES` and re-acquires by itself.
    loop {
        let cfg = dual_core::DecodeConfig {
            freq_min: 100.0,
            freq_max: 3_000.0,
            sync_min: 1.0,
            pass1_limit: PASS1_LIMIT,
            max_cand: MAX_CAND,
            q_thresh: DEFAULT_Q_THRESH,
            bp_max_iter: mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
            depth: DecodeDepth::EMBEDDED,
            budget_ms: FT8_BUDGET_MS,
            fine_sync: FT8_FINE_SYNC,
            key_up_guard_ms: FT8_KEY_UP_GUARD_MS,
            fine_sync_late: FT8_FINE_SYNC_LATE,
            share_cand_budget: FT8_SHARE_CAND,
            slot_floor_ms: FT8_SLOT_FLOOR_MS,
            slot_end_hint: Some(slot_end_hint),
        };
        let out = dual_core::run_speculative_slot(spec_q, slot_q, &cfg);
        let dual_core::SpeculativeOut {
            spec,
            slot,
            results,
            n_pass1,
            n_cut,
            n_fallback,
            n_ready,
            n_deferred,
            // Not read any more: the ±0.2 s/slot nudge it fed was a
            // random walk, not an acquisition (see the lock-and-hold
            // comment below). Acquisition is cold acquisition's job.
            bootstrap_dt_med: _,
            t_post_recv,
            t_coarse_done,
            t_fine_done,
            t_early_done,
            t_slot_recv,
            t_done,
            skipped,
            leftover,
            failed_coarse,
            slot_end_hint_us,
        } = out;
        let wav_idx = slot.wav_idx;

        // Nothing was decoded and nothing should be read from this slot:
        // its bundle reached this task with less time than the
        // un-deadlined coarse + fine sync need (`FT8_SLOT_FLOOR_MS`).
        // The audio has been received and dropped; say so and wait for
        // the next bundle, which frees the cores for the audio pipeline
        // — after a cold acquisition it has a backlog to clear.
        if skipped {
            // Two readings of the same margin: the one the floor
            // decided on (the audio sink's clock, the only one
            // available before the slot ends) and the one stage1_inc's
            // own boundary gives once the Slot arrives. They agreeing
            // is what makes a skip right; the first run of this floor
            // skipped every other slot and only the second number
            // showed it (2026-09-18).
            let margin_at = |slotend: i64| {
                (slotend + dual_core::FT8_KEY_UP_AFTER_SLOT_END_US
                    - FT8_KEY_UP_GUARD_MS * 1_000
                    - t_post_recv)
                    / 1_000
            };
            log::warn!(
                "SLOT[{wav_idx}] src={source} skipped — bundle arrived with {} ms \
                 to key-up, under the {FT8_SLOT_FLOOR_MS} ms floor \
                 (slot says {} ms, q_wait={} us)",
                slot_end_hint_us.map(margin_at).unwrap_or(0),
                margin_at(slot.slotend_us),
                t_post_recv - spec.emit_us
            );
            slot_seq = slot_seq.wrapping_add(1);
            continue;
        }

        let slotend = slot.slotend_us;
        let tail_window = (slotend - t_post_recv).max(0);
        let coarse_us = t_coarse_done - t_post_recv;
        let tail_use = (slotend.min(t_early_done) - t_coarse_done).max(0);
        let post_slotend = (t_done - slotend).max(0);
        // **Warn only past key-up.** The line used to fire whenever
        // `slot_wait` was near zero — no idle before the next slot. With
        // per-candidate fine sync that is every slot: its coarse retries
        // are the lowest-value work and are meant to fill whatever the
        // budget leaves, and on hardware (2026-09-17,
        // `logs/sim_finesync_2026-09-17.log`) they did, finishing
        // 260-505 ms past slot end on each of seven slots that all
        // decoded 8. A warning that fires every slot teaches its reader
        // to skip it — two partial slots doing that is why `full_slot`
        // below exists.
        //
        // What the operator actually loses is the reply: the budget runs
        // to this station's own key-up, `TX_START_OFFSET_S` = 0.5 s after
        // the slot boundary (`FT8_BUDGET_MS`'s derivation). A decode that
        // lands later cannot be answered this period. So that is the
        // line — the deadline stops *claiming* candidates there, and a
        // candidate already in BP can still carry the slot past it.
        //
        // Busy periods crossing it are still an operating limit on FT8,
        // not a fault: stations alternate periods, and the denser one is
        // the one that runs out (measured 2026-08-23 on 40 m, seven
        // decodes on one period against one or two on the other).
        // WSPR's and FST4's monitor loops are the other case — built
        // with deliberate slack, so an overrun there is a fault. Do not
        // carry this framing across.
        //
        // Only full slots are judged: the first slots after the grid
        // anchors to UTC are partial by construction, and comparing one
        // against a whole slot's budget produces a warning about nothing.
        const FULL_SLOT_SAMPLES: usize = 180_000;
        let full_slot = slot.audio().len() >= FULL_SLOT_SAMPLES;
        const KEY_UP_AFTER_SLOT_END_US: i64 = dual_core::FT8_KEY_UP_AFTER_SLOT_END_US;
        if full_slot && post_slotend > KEY_UP_AFTER_SLOT_END_US {
            log::warn!(
                "SLOT[{wav_idx}] src={source} PAST KEY-UP — finished {post_slotend} us after slot \
                 end, {} ms past key-up ({n_cut} candidates cut, {n_deferred} deferred)",
                (post_slotend - KEY_UP_AFTER_SLOT_END_US) / 1_000
            );
        }
        // The key-up floor reads the boundary from the audio sink
        // before the slot ends; this is that reading against
        // stage1_inc's own, the only check on it that costs nothing.
        // A slot's worth of error here is the bug of 2026-09-18.
        let hint_err = match slot_end_hint_us {
            Some(h) => format!("{:+}ms", (h - slotend) / 1_000),
            None => "na".to_string(),
        };
        // **Two lines, because the UDP sink clips at
        // `log_sink::LINE_MAX` (160).** In UAC mode that sink is the
        // only channel off the board, and this line had grown to ~250
        // characters: on a radio it arrived as
        // `... coarse=98307us fine=114~`, with every timing after
        // `fine` gone — which is most of what the line is for. Asking
        // for a longer buffer would cost internal DRAM in 44 staged
        // copies; two lines cost nothing.
        log::info!(
            "SLOT[{wav_idx}] src={source} grid={} p1={n_pass1} ready={n_ready} defer={n_deferred} \
             cut={n_cut} fb={n_fallback} dec={}",
            mfsk_app_shared::time_sync::grid_lock().label(),
            results.len(),
        );
        log::info!(
            "SLOT[{wav_idx}] t: budget={FT8_BUDGET_MS}ms \
             tail_win={}us q_wait={}us hint_err={hint_err} coarse={}us fine={}us early={}us \
             tail_use={}us post_slotend={}us slot_wait={}us late={}us",
            tail_window,
            // How long this bundle sat in `spec_q` — a busy decode
            // task. Small while `tail_win` is also small means the
            // other case: stage1_inc emitted late, starved.
            t_post_recv - spec.emit_us,
            coarse_us,
            // Inside `early` below, which keeps its historical meaning
            // (from coarse done) so logs from before fine sync compare.
            t_fine_done - t_coarse_done,
            t_early_done - t_coarse_done,
            tail_use,
            post_slotend,
            t_slot_recv - t_early_done,
            t_done - t_slot_recv,
        );
        slot_seq = slot_seq.wrapping_add(1);

        for r in results.iter() {
            mfsk_app_shared::time_sync::record_decode_dt(r.dt_sec);
        }
        mfsk_app_shared::time_sync::finalize_slot();
        let n_dec = results.len();
        // This slot's median, or `None` if it produced no decodes — in
        // which case `finalize_slot` kept the previous estimate and
        // `slot_dt_offset` would report *that*, which is not this slot.
        let slot_median = if n_dec > 0 {
            mfsk_app_shared::time_sync::slot_dt_offset()
        } else {
            None
        };
        if let Some(off) = mfsk_app_shared::time_sync::slot_dt_offset() {
            log::info!(
                "  median DT = {:+.3} s ({} slots)",
                off,
                mfsk_app_shared::time_sync::slots_finalised()
            );
        }
        // Cross-slot phase filter (#356b) — tracks under NTP too, so the
        // panel's estimate is meaningful whatever the grid follows.
        // Fed from the *pooled* multi-slot median, not the raw
        // single-slot one — see `pooled_dt_median`'s doc comment for
        // why a single slot's median is too small a sample on a real,
        // multi-station band to smooth into a servo signal by itself.
        if source == "uac" {
            if slot_median.is_some() {
                if let Some(pooled) = mfsk_app_shared::time_sync::pooled_dt_median() {
                    mfsk_app_shared::time_sync::observe_slot_phase(pooled, 15.0);
                }
            }
        }

        // Self-align the slot grid from the air (#356), for the live
        // source before NTP has disciplined the clock. `Ft8ChunkSink`
        // hands the phase to the UTC drift check the moment
        // `clock_is_disciplined()` turns true; on a hilltop with no
        // network it never does, and the grid would otherwise free-run
        // at a phase uniform over 15 s against a mode that tolerates
        // ±2.5 s. Coarse sync's own DT — the confirmed-decode median
        // once decodes exist, the top-5 candidate median
        // (`bootstrap_dt_med`) before then — is a time reference present
        // wherever the receiver is, posted through
        // `set_bootstrap_slot_shift_12k`.
        //
        // Gated on `!clock_is_disciplined()` so that once NTP is up this
        // whole block — the median read, the shift maths, the log — does
        // not run at all, rather than computing a correction
        // `Ft8ChunkSink` would only drain. The `wav` source defines its
        // own boundaries and is never touched.
        if source == "uac" && !mfsk_app_shared::time_sync::clock_is_disciplined() {
            // **Lock and hold.** The grid is established once — by cold
            // acquisition below, or by NTP/RTC — and then left alone.
            // Nothing steers it per-slot any more, and that is the point.
            //
            // The board's own oscillator is ~3 ppm: 45 µs of drift per
            // 15 s slot, 11 ms per hour, 0.26 s per *day*, against a
            // search window measured in seconds. There is no physical
            // process fast enough to need a per-slot servo.
            //
            // What the old servo was actually tracking was noise. A
            // decode's DT is *that station's* clock error, not ours —
            // WSJT-X reports it and never feeds it back into its own
            // capture window. Which stations decode changes slot to
            // slot, so the median moves with the station mix; and since
            // fading correlates over tens of seconds, the same biased
            // subset can persist for several slots running, which no
            // amount of pooling or EMA smoothing can separate from a
            // real error. Measured on hardware via `MFSK_CORES3_SIM`:
            // the raw per-slot median jumped 0.205 s between adjacent
            // slots with *zero* shift applied in between; feeding it
            // back oscillated the grid to ±0.6 s; pooling the raw
            // per-decode DTs across slots and EMA-filtering the result
            // still wandered to -0.74 s over six consecutive slots,
            // with `dec` falling 8 → 4 while it did.
            //
            // So: no `set_bootstrap_slot_shift_12k` from decode DTs at
            // all, in any branch. `bootstrap_dt_med`'s ±0.2 s/slot
            // nudge goes too — its own doc admits it is "essentially
            // always `Some`, just a small near-random value when the
            // true signal is outside ±1 s", which is a random walk, not
            // an acquisition. Acquisition is cold acquisition's job
            // (25 s capture, ±2.5 s tiled search, circular statistics,
            // R ≥ 0.55 gate), and the same trigger doubles as the
            // recovery path if the grid ever is genuinely lost.
            //
            // `observe_slot_phase` above still runs: the filtered phase
            // is a panel readout, which is what #356b built it for.
            // Lock / re-acquire policy lives in
            // `mfsk_app_shared::grid_state`, which is host-tested —
            // both bugs it guards against (one decode counting as a
            // lock, and an under-par run that never reaches
            // acquisition) are reachable from a short sequence of
            // decode counts, and finding them by reflashing the board
            // cost several cycles apiece.
            //
            // **Only full slots vote.** A partial slot — the first after
            // the grid anchors, whose audio starts wherever the anchor
            // fell — places the signals somewhere a full slot on the
            // same grid does not, so what it decodes says nothing about
            // the grid. On hardware (`logs/sim_finesync_2026-09-17.log`,
            // `sim_finesync_retryorder_2026-09-18.log`) slot 0 held
            // 153 600 and 134 400 samples, decoded 8 and 5, and locked;
            // every full slot on that grid then decoded 0, and the
            // six-slot relock wait plus the acquisition kept the band
            // dark for about 2.5 minutes. The run before (2026-09-16)
            // had a slot 0 of 87 600 samples that decoded nothing, did
            // not lock, and reached steady decoding four slots sooner.
            // The partial slot's decodes are still shown; it just
            // neither locks nor counts as under par.
            let action = if full_slot {
                grid.observe(n_dec)
            } else {
                log::info!(
                    "  air-sync: partial slot ({} samples, {n_dec} decoded) — not counted \
                     toward the grid",
                    slot.audio().len()
                );
                mfsk_app_shared::grid_state::GridAction::Hold
            };
            match action {
                mfsk_app_shared::grid_state::GridAction::Lock { n_dec } => {
                    log::info!(
                        "  air-sync: grid locked (N={n_dec} ≥ {}) — holding",
                        mfsk_app_shared::grid_state::LOCK_MIN_DECODES
                    );
                    mfsk_app_shared::time_sync::note_grid_lock(
                        mfsk_app_shared::time_sync::GridLock::Air,
                    );
                }
                mfsk_app_shared::grid_state::GridAction::Acquire { slots, relock } => {
                    if relock {
                        // The panel stops claiming a grid the decoder
                        // can no longer demonstrate.
                        mfsk_app_shared::time_sync::note_grid_lock(
                            mfsk_app_shared::time_sync::GridLock::FreeRun,
                        );
                    }
                    crate::uac::arm_acquisition();
                    log::warn!(
                        "  cold-acquisition: {} {slots} slots — capturing 25 s of FT8",
                        if relock {
                            "no decode since the lock for"
                        } else {
                            "grid under par for"
                        }
                    );
                }
                mfsk_app_shared::grid_state::GridAction::Hold => {}
            }
            if grid.is_locked() {
                mfsk_app_shared::time_sync::note_grid_lock(
                    mfsk_app_shared::time_sync::GridLock::Air,
                );
            }

            // Keep the channel drained at 0 so nothing stale can reach
            // `Ft8ChunkSink` — the only writer left is cold acquisition,
            // through its own uncapped one-shot channel.
            mfsk_app_shared::time_sync::set_bootstrap_slot_shift_12k(0);

            if grid.is_acquiring() {
                // **Say so on the panel.** Nothing decodes for the 25 s
                // capture and the arithmetic after it, and the only
                // grid indicator on screen is one character saying what
                // the grid *is* — so a receiver working on a lock and a
                // receiver that has stopped looked identical.
                if let Some((have, want)) =
                    crate::uac::acquisition_fill(mfsk_core::ft8::acquire::REQUIRED_SAMPLES)
                {
                    let mut line: heapless::String<32> = heapless::String::new();
                    let _ = write!(
                        &mut line,
                        "SYNC: capturing {}/{} s",
                        have / 12_000,
                        want / 12_000
                    );
                    if let Ok(mut ui) = UI.lock() {
                        ui.set_acq_line(line.as_str());
                    }
                }
                if let Some(audio) = crate::uac::take_acquisition_audio(
                    mfsk_core::ft8::acquire::REQUIRED_SAMPLES,
                ) {
                    // **Try the clusters; the decoder decides.**
                    //
                    // Acquisition used to reduce the candidates to one
                    // phase and gate it on `r`. Neither part worked:
                    // the reduction returned a phase decoding nothing
                    // on 16 of 40 offsets, and `r` read up to 1.00 on
                    // exactly those — on hardware three acquisitions in
                    // a row were accepted at `r >= 0.98` while taking
                    // the receiver from one decode to none.
                    //
                    // What no statistic over the candidates can say is
                    // which cluster is the grid rather than a loud
                    // outlier station or a correlation artefact. What
                    // can is a decode, and the 25 s capture needed for
                    // one is already in hand. A phantom counts — its
                    // message is nonsense, its sync is real, and only
                    // its dt matters here.
                    //
                    // The trial searches the crate default ±2.5 s, not
                    // the narrower window the per-slot path runs, and
                    // it may: the phase applied is not the cluster
                    // centre but that centre corrected by the median DT
                    // of what the trial decoded. The correction is what
                    // decouples the two windows — measured, a 2.5 s
                    // trial with it accepts nothing outside ±1.0 s,
                    // where without it four of forty were.
                    // **Let the idle task run between the pieces.**
                    //
                    // Everything below is one uninterrupted stretch of
                    // compute on this task: three tiled searches inside
                    // `acquire_slot_phases` (543-635 ms each, measured
                    // on hardware 2026-09-16) and then up to
                    // `ACQUIRE_MAX_TRIALS` full-slot decodes at ~1.1 s.
                    // 10-15 s in total, during which `IDLE1` never gets
                    // scheduled and the task watchdog fires every 5 s —
                    // seven times in one 180 s capture the first time
                    // this path ran on a board at all. Nothing is
                    // wedged (`CONFIG_ESP_TASK_WDT_PANIC` is off, and
                    // the acquisition completes and locks), but the
                    // log then carries a backtrace per event, and that
                    // reads as a fault to whoever finds it next.
                    //
                    // One tick, four or five times per acquisition, is
                    // enough for `IDLE1` to feed its own watchdog. The
                    // slot-budget path never sees it: acquisition runs
                    // between slots, not inside one.
                    unsafe { esp_idf_svc::sys::vTaskDelay(1) };
                    let phases = mfsk_core::ft8::acquire::acquire_slot_phases(
                        &audio,
                        100.0,
                        3_000.0,
                        1.0,
                        ACQUIRE_MAX_CAND,
                        ACQUIRE_MAX_TRIALS,
                    );
                    let mut applied: Option<(f32, usize, usize)> = None;
                    // **Where in its slot the capture began.** The ring
                    // starts filling whenever `arm_acquisition` ran —
                    // after the slot that triggered it had decoded, so
                    // part-way into the next — and every phase below is
                    // measured from the capture's first sample. The grid
                    // shift is relative to a slot boundary, so the
                    // capture's own offset into its slot belongs in it.
                    //
                    // It was missing. On hardware (2026-09-18, clockless,
                    // feed 3.000 s late) the first acquisition armed
                    // ~0.98 s into a slot and applied +1.90 s, leaving the
                    // grid 1.10 s short — outside the per-slot ±1.0 s
                    // search, so a second acquisition was needed and the
                    // band stayed dark ~3 minutes. The next run recorded
                    // the offset directly: 0.395 s, with the grid landing
                    // ~0.2 s short. The remainder is the trial median's
                    // own station-mix bias, which this does not remove —
                    // measured since at ~0.2 s worst case and ~0.03 s
                    // between adjacent real slots, and no cheaper to
                    // remove by averaging differently (see the note
                    // beside `ACQUIRE_MAX_TRIALS`).
                    let start_s = crate::uac::acquisition_start_in_slot()
                        .map_or(0.0, |n| n as f32 / 12_000.0);
                    for (trial, &(centre, _)) in phases.iter().enumerate() {
                        unsafe { esp_idf_svc::sys::vTaskDelay(1) };
                        {
                            let mut line: heapless::String<32> = heapless::String::new();
                            let _ = write!(
                                &mut line,
                                "SYNC: trying {}/{}",
                                trial + 1,
                                phases.len()
                            );
                            if let Ok(mut ui) = UI.lock() {
                                ui.set_acq_line(line.as_str());
                            }
                        }
                        let off = ((centre * 12_000.0).round() as i64)
                            .rem_euclid(SLOT_TRIAL_SAMPLES as i64)
                            as usize;
                        if audio.len() < off + SLOT_TRIAL_SAMPLES {
                            continue;
                        }
                        let got = mfsk_core::ft8::decode_block::decode_block_tuned(
                            &audio[off..off + SLOT_TRIAL_SAMPLES],
                            100.0,
                            3_000.0,
                            1.0,
                            DecodeDepth::EMBEDDED,
                            MAX_CAND,
                            mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
                        );
                        if got.is_empty() {
                            continue;
                        }
                        // The trial's own decodes place the grid far
                        // better than the cluster centre does. The
                        // median of them, not the earliest — see the
                        // note beside the acquisition constants for the
                        // measurement that says so.
                        let mut dts: heapless::Vec<f32, 32> = heapless::Vec::new();
                        for r in got.iter() {
                            let _ = dts.push(r.dt_sec);
                        }
                        dts.sort_by(|a, b| {
                            a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal)
                        });
                        let med = dts[dts.len() / 2];
                        let mut dt = centre + med + start_s;
                        while dt > 7.5 {
                            dt -= 15.0;
                        }
                        while dt <= -7.5 {
                            dt += 15.0;
                        }
                        // **Best of the trials, not the first that
                        // decodes anything.**
                        //
                        // This used to `break` here. One decode was
                        // therefore enough to set the grid for the
                        // session — the same "one decode is not a
                        // lock" that `grid_state` refuses by name
                        // (`LOCK_MIN_DECODES = 3`, and its doc: a grid
                        // a full second out still decodes the odd
                        // station). Measured 2026-09-16 across four
                        // acquisitions on the board, the accepted
                        // trial decoded 7, 2, 1 and 6; the 1 came from
                        // trial 3 of 5, so two better phases were
                        // computed, ranked, and never tried. The grid
                        // it set then decoded 0-1 per slot until the
                        // next acquisition three minutes later.
                        //
                        // Ranking by decode count is the same evidence
                        // the acceptance rule already uses, applied to
                        // all of the candidates instead of to the
                        // first. The extra cost is bounded and small
                        // against what acquisition already spends: at
                        // most `ACQUIRE_MAX_TRIALS - 1` further
                        // full-slot decodes (~1.1 s each on this
                        // board) on top of a 25 s capture.
                        //
                        // Ties keep the earlier trial, which is the
                        // higher-scoring cluster from
                        // `acquire_slot_phases`.
                        if applied.is_none_or(|(_, _, best_n)| got.len() > best_n) {
                            applied = Some((dt, trial + 1, got.len()));
                        }
                        // **Stop once a trial clears the lock bar.**
                        //
                        // Best-of-N is about not settling for a trial
                        // that decoded one station; it is not about
                        // trying every cluster for its own sake, and
                        // trying them all is expensive: acquisition
                        // measured 15.2 s of compute on this task
                        // (2026-09-18), one whole slot period, which is
                        // what pushes the next slot past key-up.
                        //
                        // `LOCK_MIN_DECODES` is the receiver's own
                        // statement of what counts as a grid, so a
                        // trial that reaches it is not a lucky single
                        // decode and the remaining clusters have
                        // nothing to prove. Measured on the host mirror
                        // (`mirror_acquisition_early_stop`, 24 capture
                        // phases per recording): identical decodes on
                        // all three recordings — qso3 8.00, qso1 3.88,
                        // qso2 4.74, the same as trying all five — for
                        // 2.3 trials per acquisition instead of 5.0.
                        // Stopping at one decode instead, which is what
                        // this loop did before best-of-N, is the rule
                        // that cost minutes of dark band on hardware.
                        // Per-trial, because the summary below reports
                        // only the winner: the log that found this bug
                        // said "trial 3/5, 1 decoded" and could not say
                        // what 4 and 5 would have given.
                        log::info!(
                            "    acq trial {}/{}: centre={centre:+.3} decoded={} dt={dt:+.3}",
                            trial + 1,
                            phases.len(),
                            got.len(),
                        );
                        // Stop once a trial clears the lock bar — the
                        // measurement is in the note beside
                        // `ACQUIRE_MAX_TRIALS`. After the log line: the
                        // trial that ends the search is the one the log
                        // can least afford to be missing, and putting
                        // the break first silently dropped it
                        // (`logs/sim_slotfloor_*_2026-09-18.log` says
                        // "trial 3/5, 6 decoded" with no trial-3 line).
                        if got.len() >= mfsk_app_shared::grid_state::LOCK_MIN_DECODES {
                            break;
                        }
                    }
                    // An applied phase restarts the under-par run — the
                    // grid just moved, so the slots that led here say
                    // nothing about the new one. A run where nothing
                    // decoded leaves it standing, so the retry is
                    // immediate.
                    grid.acquisition_done(applied.is_some());
                    // Back to the QSO line; the link bar's `a` now says
                    // what the grid is.
                    if let Ok(mut ui) = UI.lock() {
                        ui.set_acq_line("");
                    }
                    match applied {
                        Some((dt, trial, n)) => {
                            let shift = (dt * 12_000.0).round() as i32;
                            mfsk_app_shared::time_sync::set_acquisition_shift_12k(shift);
                            mfsk_app_shared::time_sync::note_grid_lock(
                                mfsk_app_shared::time_sync::GridLock::Air,
                            );
                            // Not `observe_slot_phase(dt, ...)` — `dt` is
                            // the *pre*-correction residual, and the
                            // one-shot `shift` above is about to erase
                            // it from the grid. Feeding it to the EMA
                            // seeded the cross-slot filter with a value
                            // that no longer applied once the shift
                            // landed, so the filtered estimate spent
                            // several slots decaying off a stale number
                            // instead of tracking the (near-zero)
                            // post-correction residual — caught via
                            // MFSK_CORES3_SIM (filtered=+1.524s against
                            // a same-slot raw median of +0.601s, right
                            // after a cold-acquisition lock).  Clear the
                            // filter instead so it starts fresh from the
                            // next slot's post-correction observation.
                            mfsk_app_shared::time_sync::reset_slot_phase();
                            // Same reasoning — the pooled multi-slot
                            // ring is entirely pre-correction evidence
                            // at this point, so clear it too rather than
                            // let it drag a stale median into the first
                            // several post-correction slots.
                            mfsk_app_shared::time_sync::reset_dt_pool();
                            // Persist it so the reboot into FT4 mode
                            // ("QSY to FT4", #356) keeps the lock.
                            if let Some(now_ms) = mfsk_app_shared::time_sync::utc_now_ms() {
                                crate::uac::persist_grid_fix(mfsk_app_shared::grid_fix::GridFix {
                                    offset_us: (dt * 1_000_000.0).round() as i32,
                                    period_s: 15.0,
                                    epoch_at_fix: (now_ms / 1000) as i64,
                                    // The trial decoded `n` messages
                                    // at this phase, which is a far
                                    // better statement of confidence
                                    // than the `r` this used to carry.
                                    // Saturates at 1.0 by
                                    // `LOCK_MIN_DECODES`, so a phase
                                    // good enough to lock persists as
                                    // fully trusted.
                                    confidence: (n as f32
                                        / mfsk_app_shared::grid_state::LOCK_MIN_DECODES as f32)
                                        .min(1.0),
                                });
                            }
                            log::warn!(
                                "  cold-acquisition: grid phase {dt:+.2} s → shift {shift:+} samples \
                                 (trial {trial}/{}, {n} decoded)",
                                phases.len()
                            );
                            log::warn!(
                                "  cold-acquisition: capture began {start_s:.3} s into its slot \
                                 (counted in the phase)"
                            );
                        }
                        None => log::warn!(
                            "  cold-acquisition: none of {} candidate phases decoded — will retry",
                            phases.len()
                        ),
                    }
                }
            }
        }

        let mut had_response_this_slot = false;
        if let Ok(mut ui) = UI.lock() {
            for r in results.iter() {
                if let Some(text) = unpack77(r.message77()) {
                    let mut msg: heapless::String<22> = heapless::String::new();
                    let take = text.len().min(msg.capacity());
                    let _ = msg.push_str(&text[..take]);
                    const FP_SPEC_SHIFT: u32 = 12;
                    let cell_scale = (1u32 << FP_SPEC_SHIFT) as f32;
                    let calibrated_snr =
                        mfsk_core::ft8::decode_block::xsnr2_db_simple(&spec.spec, r, cell_scale);
                    let snr_i8 = calibrated_snr.round().clamp(-128.0, 127.0) as i8;
                    let row = DecodedRow {
                        df_hz: r.freq_hz.round().clamp(0.0, 65_535.0) as u16,
                        snr_db: snr_i8,
                        hard_errors: r.hard_errors.min(255) as u8,
                        msg,
                        slot_seq,
                        first_seq: slot_seq,
                    };
                    ui.push_decode(row);
                    log::info!(
                        "{:4.0}Hz {:+5.1}dB (raw={:+5.1}) {}",
                        r.freq_hz,
                        calibrated_snr,
                        r.snr_db,
                        text
                    );
                    qso.set_rx_snr(snr_i8);
                    let parity_lock_ok = mfsk_app_shared::parity::framing_settled_for_parity_lock();
                    if qso
                        .process_message(&text, wav_idx as u32, parity_lock_ok)
                        .is_some()
                    {
                        had_response_this_slot = true;
                    }
                }
            }
        }

        // **This period's transmission is decided here** — before
        // key-up, from what decoded before key-up. That is the bound's
        // whole purpose.
        if qso.state == QsoState::Idle {
            qso.call_cq(None);
        }
        let intent = qso.next_tx();
        push_tx_line(&qso, intent.as_ref());

        // **What key-up cut, finished on the idle time after it.**
        //
        // The reply for *this* period has just been decided, which is
        // what the key-up bound protects. Everything the bound stopped
        // is still worth having — on a CQ-first portable station it is
        // the queue the next period's call is chosen from — and the
        // decode task is about to block on `spec_q` for ~13 s with the
        // slot's audio still valid. So it runs there, yielding the
        // moment the next SpecBundle lands.
        //
        // These rows reach the panel and the DT statistics. They are
        // deliberately **not** fed to `QsoManager`: its intent for this
        // period is already out, and whether a caller decoded after
        // key-up should enter the state machine a period late is a
        // policy question, not a side effect of where the decode
        // finished.
        let mut late_response = false;
        // **The carry-over runs on slack, and only on slack.**
        //
        // It is stage-3 work, so it goes through `dsp_worker` on
        // APP_CPU — the core `stage1_inc` lives on. Priority 6 lets
        // stage1_inc preempt it, but the two share the PSRAM bandwidth
        // and the FFT, and a spectrogram that has to fight for those
        // finishes late. "Until the next SpecBundle arrives" therefore
        // meant eleven seconds of every fifteen with that contention
        // standing.
        //
        // Measured on a radio, 2026-09-19: `tail_win` 1.3-2.1 s against
        // a geometric 0.93, `hint_err` −0.6..−1.2 s — stage1_inc a
        // second behind the audio sink — and the next bundle then
        // arriving inside the 500 ms floor, which dropped the slot.
        // Every other slot on a band running 8-13 stations.
        //
        // Three gates, all of them "is there slack":
        //
        // * this slot did not already run past key-up,
        // * stage1_inc is keeping up (`hint_err` inside
        //   `CARRY_OVER_MAX_LAG_US` — it is the measurement that says
        //   whether the last slot's work hurt), and
        // * a hard cap on the slice, so even a healthy pipeline gets
        //   the core back long before the next bundle is due.
        const CARRY_OVER_MAX_MS: i64 = 2_000;
        const CARRY_OVER_MAX_LAG_US: i64 = 300_000;
        let pipeline_lag = slot_end_hint_us.map_or(0, |h| (slotend - h).abs());
        let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
        let carry_deadline = (now_us + CARRY_OVER_MAX_MS * 1_000)
            .min(slotend + 15_000_000 - 1_500_000);
        let carry_ok = post_slotend <= dual_core::FT8_KEY_UP_AFTER_SLOT_END_US
            && pipeline_lag <= CARRY_OVER_MAX_LAG_US
            && now_us < carry_deadline;
        if !carry_ok && (!leftover.is_empty() || !failed_coarse.is_empty()) {
            log::info!(
                "SLOT[{wav_idx}] carry-over skipped — {} candidates held back                  (post_slotend={post_slotend}us, pipeline lag={}us)",
                leftover.len() + failed_coarse.len(),
                pipeline_lag,
            );
        }
        if carry_ok && (!leftover.is_empty() || !failed_coarse.is_empty()) {
            let n_left = leftover.len() + failed_coarse.len();
            let late = dual_core::continue_leftovers(
                spec_q,
                slot.audio(),
                leftover,
                failed_coarse,
                &cfg,
                &results,
                carry_deadline,
            );
            if !late.is_empty() {
                log::info!(
                    "SLOT[{wav_idx}] src={source} past key-up: {}/{n_left} carried candidates decoded on the idle tail",
                    late.len(),
                );
                if let Ok(mut ui) = UI.lock() {
                    for r in late.iter() {
                        if let Some(text) = unpack77(r.message77()) {
                            let mut msg: heapless::String<22> = heapless::String::new();
                            let take = text.len().min(msg.capacity());
                            let _ = msg.push_str(&text[..take]);
                            const FP_SPEC_SHIFT: u32 = 12;
                            let cell_scale = (1u32 << FP_SPEC_SHIFT) as f32;
                            let calibrated_snr = mfsk_core::ft8::decode_block::xsnr2_db_simple(
                                &spec.spec,
                                r,
                                cell_scale,
                            );
                            let snr_i8 = calibrated_snr.round().clamp(-128.0, 127.0) as i8;
                            ui.push_decode(DecodedRow {
                                df_hz: r.freq_hz.round().clamp(0.0, 65_535.0) as u16,
                                snr_db: snr_i8,
                                hard_errors: r.hard_errors.min(255) as u8,
                                msg,
                                slot_seq,
                                first_seq: slot_seq,
                            });
                            log::info!(
                                "{:4.0}Hz {:+5.1}dB (raw={:+5.1}) {} [late]",
                                r.freq_hz,
                                calibrated_snr,
                                r.snr_db,
                                text
                            );
                            qso.set_rx_snr(snr_i8);
                            let parity_lock_ok =
                                mfsk_app_shared::parity::framing_settled_for_parity_lock();
                            if qso
                                .process_message(&text, wav_idx as u32, parity_lock_ok)
                                .is_some()
                            {
                                had_response_this_slot = true;
                                late_response = true;
                            }
                        }
                    }
                }
            }
        }

        // **The retry count waits for the whole slot.** A partner whose
        // reply decodes 200 ms after key-up answered — counting that
        // period as unanswered spends a retry, and at the limit
        // `on_period_end` resets the QSO outright, throwing away a
        // contact whose reply is on the screen. This period's
        // transmission is already chosen either way (above), so the
        // only thing that moves is the bookkeeping: at the retry limit
        // the reset now lands one period later, which costs one repeat
        // and saves the QSOs the old order abandoned.
        if !had_response_this_slot && qso.state != QsoState::Idle {
            let _ = qso.on_period_end();
        }
        if late_response {
            // `next_tx` is a pure read; this only refreshes what the
            // panel shows for the *next* period.
            push_tx_line(&qso, qso.next_tx().as_ref());
        }
    }
}

fn push_tx_line(qso: &QsoManager, intent: Option<&qso::TxIntent>) {
    let line = qso::format_tx_line(qso, intent);
    log::info!("[QSO] {}", line.as_str());
    if let Ok(mut ui) = UI.lock() {
        ui.set_tx_line(line.as_str());
    }
}

fn wf_drain(wf_q: esp_idf_svc::sys::QueueHandle_t) -> ! {
    let n: u32 = 0;
    loop {
        let tick = pipeline::recv_box::<pipeline::WfTick>(wf_q);
        if let Ok(mut ui) = UI.lock() {
            ui.push_waterfall(tick.row);
        }
        let _ = n;
    }
}
