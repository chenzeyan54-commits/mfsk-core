//! When to call the slot grid locked, and when to go get a new one.
//!
//! Split out of `m5stack-cores3-app`'s decode pipeline so it can be
//! tested on the host. Everything here is a pure function of the
//! per-slot decode count — no clock, no audio, no hardware — and the
//! two bugs it exists to prevent were both reachable from a short
//! sequence of counts, which is exactly what a unit test is for. They
//! cost several reflash-and-watch cycles to find on the board instead.
//!
//! The model, in one line: **the grid is set once and held; a run of
//! slots that are not decoding properly is what sends us back for a
//! new one** (#356).

/// Decodes a slot must produce before the grid counts as locked.
///
/// Three, because one is demonstrably reachable on a grid that is a
/// full second wrong: at a 1.2 s offset `MFSK_CORES3_SIM` decoded
/// exactly one station for 13 slots straight, against 8 for the same
/// audio aligned. Two would still be inside the noise of that; three
/// is the smallest count that says the window is genuinely on the band
/// rather than clipping one strong signal's edge.
pub const LOCK_MIN_DECODES: usize = 3;

/// How far off the grid may be and still be lockable, in seconds.
///
/// **A lock used to be a decode count and nothing else**, and a count
/// is not a phase: on a live band a grid 0.68 s out decoded 6 stations
/// on its first slot, locked on that, and then decoded 0-1 for as long
/// as it was left alone (2026-09-19, CoreS3 with the RTC as its only
/// clock). The decoder searches ±1.0 s and works well over about
/// ±0.4 s, so "some stations decoded" says the grid is inside the
/// search window, not that it is anywhere near the middle of it.
///
/// 0.3 s leaves the useful plateau intact while refusing a grid that
/// is demonstrably off-centre. A slot that decodes but misses this bar
/// counts as under par, so the acquire trigger accumulates and the air
/// gets a chance to place the grid properly.
pub const LOCK_MAX_PHASE_S: f32 = 0.3;

/// Under-par slots **that carried a signal** before a cold
/// acquisition, from a standing start (nothing has ever locked).
///
/// One. The count used to be three, standing in for "is the band quiet
/// or is the grid lost?"; [`GridState::observe_slot`] now measures
/// that directly and a signal-less slot no longer counts, so the run
/// no longer has to be long to mean anything. A single slot with a
/// station in it that would not decode is the whole of the evidence,
/// and on a mode the operator selected by hand it is evidence they
/// have already accepted.
pub const ACQUIRE_TRIGGER_SLOTS: u32 = 1;

/// The same count once a lock has produced decodes. Higher, and still
/// higher for a reason the signal test does not cover: after a lock,
/// slots that carry signal and decode nothing are more often fading or
/// a band full of stations this receiver cannot reach than a grid that
/// was demonstrably working going bad. Six of them is 90 s, against
/// re-acquiring's 25 s capture plus whatever the grid would have
/// decoded meanwhile.
pub const REACQUIRE_TRIGGER_SLOTS: u32 = 6;

/// What the caller should do with the grid after a slot's decodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridAction {
    /// Nothing to do.
    Hold,
    /// This slot cleared [`LOCK_MIN_DECODES`] and the grid was not
    /// locked before — raise `GridLock::Air`.
    Lock { n_dec: usize },
    /// Long enough without decoding properly. Arm the capture ring and
    /// run an acquisition. `relock` is true when a working grid is
    /// being given up, which the caller reports differently and which
    /// also drops the lock state.
    Acquire { slots: u32, relock: bool },
}

/// Lock state and the under-par run that leads back to acquisition.
#[derive(Debug, Default, Clone, Copy)]
pub struct GridState {
    /// Highest decode count seen since the current lock. `0` means
    /// nothing has locked yet.
    best_n: usize,
    /// Consecutive slots that were not decoding properly.
    lost_slots: u32,
    /// An acquisition is already in flight; the trigger stays quiet
    /// until the caller reports its outcome.
    acquiring: bool,
}

impl GridState {
    pub fn new() -> Self {
        Self::default()
    }

    /// A grid that is known to be unplaced: acquire now, not after
    /// [`ACQUIRE_TRIGGER_SLOTS`] slots of proving it.
    ///
    /// The trigger exists for a grid that *was* working and stopped —
    /// three empty slots is how you tell a quiet band from a lost lock.
    /// At a cold start with the air as the phase source there is nothing
    /// to tell apart: the clock has not placed the grid and the only
    /// thing that can is a capture. Waiting three slots for permission
    /// is 45 s of a receiver that already knows what it has to do, and
    /// was reported from the bench as exactly that.
    pub fn new_unplaced() -> Self {
        Self {
            lost_slots: ACQUIRE_TRIGGER_SLOTS,
            ..Self::default()
        }
    }

    pub fn is_locked(&self) -> bool {
        self.best_n > 0
    }

    pub fn best_n(&self) -> usize {
        self.best_n
    }

    pub fn lost_slots(&self) -> u32 {
        self.lost_slots
    }

    /// Fold in one slot's decode count.
    pub fn observe(&mut self, n_dec: usize) -> GridAction {
        self.observe_with_phase(n_dec, None)
    }

    /// [`Self::observe`] with the slot's measured phase error, when the
    /// caller has one (`time_sync::slot_dt_offset`). `None` keeps the
    /// old count-only behaviour, which is what every caller without a
    /// DT median has.
    pub fn observe_with_phase(&mut self, n_dec: usize, phase_err_s: Option<f32>) -> GridAction {
        self.observe_slot(n_dec, phase_err_s, true)
    }

    /// [`Self::observe_with_phase`] told whether the slot carried a
    /// signal the decoder could not read.
    ///
    /// **This is what the slot count was standing in for.** Waiting
    /// [`ACQUIRE_TRIGGER_SLOTS`] empty slots answered "is the band
    /// quiet, or is the grid lost?" — a question worth asking, because
    /// an acquisition on a quiet band is 40 s that cannot succeed
    /// (measured twice on a radio 2026-09-19: "none of 5 candidate
    /// phases decoded"). But the slot count is a proxy, and the direct
    /// measurement is already in hand: coarse sync's top score, which
    /// sits at the noise floor on a quiet band and runs tens to low
    /// hundreds when a station is there and the grid is missing it.
    ///
    /// So a slot with no signal no longer counts toward the trigger at
    /// all — it is not evidence about the grid either way — and one
    /// with signal counts immediately. An operator who chose
    /// `TIME: AIR DT` from the CONFIG page has already said the phase
    /// is suspect; making them wait three slots for a receiver to
    /// re-derive that is answering a question they answered.
    pub fn observe_slot(
        &mut self,
        n_dec: usize,
        phase_err_s: Option<f32>,
        had_signal: bool,
    ) -> GridAction {
        let was_locked = self.is_locked();
        // A phase this far out is not a grid worth holding, however
        // many stations came through it. See [`LOCK_MAX_PHASE_S`].
        let phase_ok = phase_err_s.is_none_or(|e| e.abs() <= LOCK_MAX_PHASE_S);

        // **One decode is not a lock.** A grid a full second out still
        // decodes the odd station, and the first cut of lock-and-hold
        // treated that as good enough: it set `best_n`, and the trigger
        // — `n_dec == 0` at the time — could then never fire again, so
        // the receiver sat at 1-of-8 indefinitely with nothing able to
        // correct it.
        let locking = n_dec >= LOCK_MIN_DECODES && !was_locked && phase_ok;
        if n_dec >= LOCK_MIN_DECODES && phase_ok {
            self.best_n = self.best_n.max(n_dec);
        }

        // Before a lock, "not decoding properly" — not "not decoding at
        // all". After one, the test stays at zero: a few empty slots
        // read as a quiet band sooner than as a grid that was
        // demonstrably working going bad.
        let below_par = if was_locked {
            n_dec == 0
        } else {
            // Decoding *and* off-centre is still under par: without
            // this the run resets on every slot that decodes, the
            // trigger never accumulates, and a grid that cannot be
            // locked can never be re-acquired either.
            n_dec < LOCK_MIN_DECODES || !phase_ok
        };
        // A slot with nothing in it neither adds to the run nor clears
        // it: it says nothing about the grid, and acquiring from it
        // would fail for the same reason it decoded nothing.
        if below_par {
            if had_signal {
                self.lost_slots += 1;
            }
        } else {
            self.lost_slots = 0;
        }

        if locking {
            return GridAction::Lock { n_dec };
        }

        let trigger = if was_locked {
            REACQUIRE_TRIGGER_SLOTS
        } else {
            ACQUIRE_TRIGGER_SLOTS
        };
        if self.lost_slots >= trigger && !self.acquiring {
            self.acquiring = true;
            let slots = self.lost_slots;
            if was_locked {
                // Give up the lock while the capture runs, so the state
                // matches reality and the panel stops claiming a grid
                // the decoder can no longer demonstrate.
                self.best_n = 0;
            }
            return GridAction::Acquire {
                slots,
                relock: was_locked,
            };
        }
        GridAction::Hold
    }

    /// Report that the in-flight acquisition finished. `applied` is
    /// true when a phase cleared the confidence gate and was used, in
    /// which case the under-par run starts over — the grid just moved,
    /// so the slots that led here say nothing about the new one.
    pub fn acquisition_done(&mut self, applied: bool) {
        self.acquiring = false;
        if applied {
            self.lost_slots = 0;
        }
    }

    pub fn is_acquiring(&self) -> bool {
        self.acquiring
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bug this module was extracted for. A grid ~1 s out decodes
    /// one station per slot forever; the receiver has to notice.
    #[test]
    fn an_unplaced_grid_acquires_on_its_first_empty_slot() {
        let mut g = GridState::new_unplaced();
        assert!(matches!(g.observe(0), GridAction::Acquire { .. }));
    }

    #[test]
    fn an_unplaced_grid_still_locks_if_the_first_slot_is_good() {
        // Nothing is forced: a first slot that decodes well, on a
        // centred grid, locks and no capture runs.
        let mut g = GridState::new_unplaced();
        assert_eq!(
            g.observe_with_phase(6, Some(0.02)),
            GridAction::Lock { n_dec: 6 }
        );
    }

    #[test]
    fn a_grid_that_decodes_but_is_off_centre_neither_locks_nor_holds() {
        // The 2026-09-19 case: six stations through a grid 0.68 s out.
        let mut g = GridState::new();
        assert!(!g.is_locked(), "an off-centre grid must not lock");
        // Under par even though the slot decoded, so the air gets its
        // chance — and with the trigger at one, on that slot.
        assert!(matches!(
            g.observe_with_phase(6, Some(-0.68)),
            GridAction::Acquire { .. }
        ));
    }

    #[test]
    fn the_same_count_locks_once_the_phase_is_centred() {
        let mut g = GridState::new();
        assert_eq!(
            g.observe_with_phase(6, Some(-0.05)),
            GridAction::Lock { n_dec: 6 }
        );
        assert!(g.is_locked());
    }

    #[test]
    fn one_decode_a_slot_is_not_a_lock_and_still_reaches_acquisition() {
        let mut g = GridState::new();
        assert!(!g.is_locked(), "one decode a slot must not lock the grid");
        // One under-par slot is now the trigger from a standing start —
        // a slot that decoded at all carried a signal by definition.
        assert_eq!(
            g.observe(1),
            GridAction::Acquire {
                slots: ACQUIRE_TRIGGER_SLOTS,
                relock: false
            }
        );
    }

    /// The same sequence under the original rule (`n_dec == 0`) never
    /// triggered, which is what pinned the receiver at 1-of-8. Guard
    /// the property directly: no run of under-par slots may be silent.
    #[test]
    fn an_under_par_run_always_terminates_in_acquisition() {
        for n in 0..LOCK_MIN_DECODES {
            let mut g = GridState::new();
            let mut fired = false;
            for _ in 0..ACQUIRE_TRIGGER_SLOTS * 4 {
                if matches!(g.observe(n), GridAction::Acquire { .. }) {
                    fired = true;
                    break;
                }
            }
            assert!(fired, "n_dec={n} never reached acquisition");
        }
    }

    #[test]
    fn a_real_lock_holds_and_does_not_re_acquire_on_a_quiet_slot() {
        let mut g = GridState::new();
        assert_eq!(g.observe(8), GridAction::Lock { n_dec: 8 });
        assert!(g.is_locked());
        // Well short of the lock threshold, but a locked grid rides it
        // out rather than throwing away something that works.
        for _ in 0..20 {
            assert_eq!(g.observe(1), GridAction::Hold);
        }
        assert!(g.is_locked());
    }

    #[test]
    fn a_locked_grid_that_goes_silent_re_acquires_and_drops_the_lock() {
        let mut g = GridState::new();
        g.observe(8);
        for _ in 0..REACQUIRE_TRIGGER_SLOTS - 1 {
            assert_eq!(g.observe(0), GridAction::Hold);
        }
        assert_eq!(
            g.observe(0),
            GridAction::Acquire {
                slots: REACQUIRE_TRIGGER_SLOTS,
                relock: true
            }
        );
        assert!(!g.is_locked(), "the lock is given up while re-acquiring");
    }

    /// A locked grid tolerates more silence than a cold start, so a
    /// quiet band does not cost a 25 s capture every three slots.
    #[test]
    fn a_lock_buys_a_longer_leash() {
        let mut cold = GridState::new();
        let mut warm = GridState::new();
        warm.observe(8);
        let mut cold_at = None;
        let mut warm_at = None;
        for i in 1..=REACQUIRE_TRIGGER_SLOTS {
            if matches!(cold.observe(0), GridAction::Acquire { .. }) && cold_at.is_none() {
                cold_at = Some(i);
            }
            if matches!(warm.observe(0), GridAction::Acquire { .. }) && warm_at.is_none() {
                warm_at = Some(i);
            }
        }
        assert_eq!(cold_at, Some(ACQUIRE_TRIGGER_SLOTS));
        assert_eq!(warm_at, Some(REACQUIRE_TRIGGER_SLOTS));
        assert!(warm_at > cold_at);
    }

    /// While a capture is in flight the trigger stays quiet — one
    /// acquisition at a time, however long the run gets.
    #[test]
    fn no_second_acquisition_while_one_is_in_flight() {
        let mut g = GridState::new();
        for _ in 0..ACQUIRE_TRIGGER_SLOTS {
            g.observe(0);
        }
        assert!(g.is_acquiring());
        for _ in 0..10 {
            assert_eq!(g.observe(0), GridAction::Hold);
        }
        // An inconclusive attempt leaves the run standing, so the next
        // slot can trigger the retry immediately.
        g.acquisition_done(false);
        assert_eq!(
            g.observe(0),
            GridAction::Acquire {
                slots: ACQUIRE_TRIGGER_SLOTS + 11,
                relock: false
            }
        );
    }

    /// A phase that was applied moved the grid, so the slots that led
    /// there say nothing about the new one — the run starts over.
    ///
    /// With [`ACQUIRE_TRIGGER_SLOTS`] at one the new phase is condemned
    /// by the next signal-bearing slot that will not decode, and that
    /// is deliberate: what keeps a failed phase from thrashing is not a
    /// slot count but the 25 s of ring the next acquisition has to
    /// refill before it can run at all.
    #[test]
    fn an_applied_acquisition_resets_the_run() {
        let mut g = GridState::new();
        for _ in 0..ACQUIRE_TRIGGER_SLOTS {
            g.observe(0);
        }
        g.acquisition_done(true);
        assert_eq!(g.lost_slots(), 0);
        assert!(matches!(g.observe(0), GridAction::Acquire { .. }));
    }

    /// A slot with nothing in it is not evidence about the grid.
    #[test]
    fn a_signal_less_slot_does_not_reach_acquisition() {
        let mut g = GridState::new();
        for _ in 0..10 {
            assert_eq!(
                g.observe_slot(0, None, false),
                GridAction::Hold,
                "a quiet band must not trigger a capture that cannot succeed"
            );
        }
        assert_eq!(g.lost_slots(), 0);
        // One slot with a station in it that would not decode is.
        assert!(matches!(
            g.observe_slot(0, None, true),
            GridAction::Acquire { .. }
        ));
    }

    /// The measured recovery, as a sequence: 1-of-8 for a few slots,
    /// acquisition, a bad phase that decodes nothing, a second
    /// acquisition, then a grid that works.
    #[test]
    fn the_hardware_recovery_sequence() {
        let mut g = GridState::new();
        assert!(matches!(g.observe(1), GridAction::Acquire { .. }));
        g.acquisition_done(true); // -5.75 s, applied, and wrong
        assert!(matches!(g.observe(0), GridAction::Acquire { .. }));
        g.acquisition_done(true); // +6.60 s, applied, close enough
        assert_eq!(g.observe(5), GridAction::Lock { n_dec: 5 });
        assert!(g.is_locked());
    }
}
