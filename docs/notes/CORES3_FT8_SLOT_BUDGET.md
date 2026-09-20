# The CoreS3 FT8 slot: what it spends, and what actually limits it

Written 2026-09-20 after a day of on-air experiments that each
answered a narrow question and two of which were reverted. The
experiments were not wrong; the trouble was that they were chosen one
at a time, without a model saying which resource each one acts on, and
several of them predicted effects smaller than the sample could
resolve. This is the model, the measurements it rests on, and the
consequences for what to try next.

Everything here is from **278 on-air slots on 7041 kHz**, five builds,
2026-09-20, logged over UDP
(`embedded-poc/m5stack-cores3-app/logs/udp_*_2026-09-20.log`).

## 1. What a slot is

```text
 t = 0.00 s   slot boundary. The previous period's audio is complete.
              WSJT-X opens its transmit window HERE: the reply is taken
              from the auto-sequencer and PTT asserted in the same pass
              (see EMBEDDED.md, "The transmit period, as WSJT-X defines
              it"). **This is the reply deadline.**
 t = 0.50 s   this station's transmit audio would start. `txDelay` and
              the rig's turnaround are spent inside this 0.5 s, not
              added to it.
```

Within the period that ends at that boundary, the decoder gets:

| resource | what sets it | measured |
|---|---|---|
| pre-boundary compute | emit point → `tail_win − coarse` | 810 ms at pair 87 |
| post-boundary compute | `FT8_KEY_UP_AFTER_SLOT_END_US` | 500 ms, of which ~270 used |
| pass-1 candidates | `PASS1_LIMIT` | **30, saturated every slot** |
| stage-3 slots | `max_cand` | **15, used in full every slot** |
| per candidate | BP + refine | ~72 ms |

## 2. What it actually does with them

```text
  pass-1  30.0  →  refine  15.0  →  decode  8.42        hit rate 56 %
                                    of which in time     7.91  (94 %)
```

Three facts follow, and the third is the one that matters.

**The deadline does not bind.** `cut > 0` on **2 of 278 slots**. Stage 3
is never stopped by running out of time; it finishes what it claimed.

**The boundary binds a little.** 6.1 % of decodes land after it, so they
are on the screen and in the next period's queue but cannot be answered
this period. At the shipped emit point that is 0.52 decodes a slot.

**The refine budget is 44 % waste.** 6.58 of the 15 stage-3 slots
produce nothing, every slot, in every arm — the figure is 51-58 % hit
rate across five builds and does not move with anything tried today.
**That is ten times the boundary loss and it is the largest single
number in the table.**

## 3. So the receiver is selection-limited, not time-limited

The decoder is not short of time; it is short of *good candidates to
spend time on*. Coarse sync hands it 30, it can afford 15, and nearly
half of those 15 are wrong.

This reframes every lever tried today:

| lever | acts on | measured | verdict |
|---|---|---|---|
| emit point 87 → 85 | pre-boundary time | late 0.70 → 0.04 (6.3 σ), dec −0.23 (0.7 σ) | real, but buys *when*, not *how many* |
| block-2 gate | selection, at the margin | 2 of 30 pass-1 candidates on a fixture, 1 inside the top 15 | acts on the right resource, ~0.2-0.4 expected |
| `share_cand_budget` | which 15, early vs late | +0.65/+0.76 on `qso1`/`qso2` (host) | acts on the right resource; needs `defer` to be large |
| raising `max_cand` | more of the 30 | 15→20→25 gave 8→7→8, +170 ms (board, 2026-09-05) | flat on one fixture, untested where it counts |
| `valid_rows` clamp | nothing | bit-identical; reverted | the hazard it targeted did not exist |

**The levers interact, which is why each looked marginal alone.**
Emitting earlier is what makes a larger candidate set answerable at
all; a larger candidate set is what makes `share_cand_budget`'s
reallocation matter; and emitting earlier is itself what *creates* the
`defer` (1.2 → 3.9) that `share_cand_budget` exists to serve. Tested
one at a time, each pays a cost in its own arm and banks its gain in
another lever's.

## 4. The unmeasured number that decides the most

**What is the hit rate of pass-1 ranks 16-30?** Thirty candidates are
found and fifteen are examined, every slot. If the unexamined half
converts at 20 %, there are three more decodes a slot available to a
larger or better-chosen budget. If it converts at 2 %, then coarse sync
is the ceiling and no amount of budget reallocation reaches it.

Nothing in this repository has measured it on anything but
`qso3_busy` at one phase, and today established that fixture is a poor
proxy: it predicted the emit move would cost 24 % in per-candidate time
where the radio showed an 8 % *improvement*, and it has 6-7 decodes a
slot where the band has 8.4 with a maximum of 13.

It is measurable on the host mirror with no radio time: run the board
pipeline at `max_cand` 15 and 30 over the same phases and compare.

## 5. What a sample can resolve

Today's per-slot spread, over 280 slots: `dec` sd 1.94, `intime` sd
1.73. For a symmetric A-B at 3 σ:

| slots per arm | detectable in `dec` | in `intime` | radio time |
|---:|---:|---:|---:|
| 45 | 1.23 | 1.10 | 22 min |
| 100 | 0.82 | 0.74 | 50 min |
| 200 | 0.58 | 0.52 | 100 min |
| 400 | 0.41 | 0.37 | 200 min |

**A 45-slot arm can only see an effect of about one decode a slot.**
The emit A-B-A's `intime` gain of +0.44 was never going to reach
significance at n = 44, and neither would the block-2 gate's expected
0.2-0.4. Both were run anyway. The rule this table exists to enforce:

> Before an on-air arm, state the effect the model predicts and read
> the detectable threshold off this table. If the prediction is below
> it, the experiment is not worth the radio time — measure the
> mechanism instead.

The block-2 gate's mechanism, for instance, is one log field away:
report how many pass-1 candidates exceed the lag ceiling and the best
rank among them. `gate2=0` most slots ends the question with no arm at
all; `gate2=1@<15` most slots gives the expected magnitude, and then
the table above says how long an arm has to be.

## 6. Order of work this implies

1. **Measure the pass-1 16-30 hit rate on the mirror.** Host only. It
   decides whether selection work has a ceiling worth chasing.
2. **Instrument rather than A-B** for anything whose predicted effect
   is under ~0.8 decodes: the block-2 gate's `gate2=`, and
   `share_cand_budget`'s reallocation count.
3. **Test the package, not the parts.** If (1) says the unexamined
   candidates convert, the combination to measure is emit 85 +
   `share_cand_budget` together, against 87 + neither — because that is
   the pairing the model says is coherent. One long arm, not three
   short ones.
4. **Leave the emit point at 87** until (3), and leave the gate off.
   Both are wired and default-off; neither costs anything sitting
   there.

## 7. What was reverted today, and why it is recorded

- A `valid_rows` clamp threaded through `coarse_sync_inner`,
  `SpecBundle` and both halves of the split, built on a code comment's
  explanation that a lag past the row bound "correlates against zeros".
  A zero adds nothing to a sum; the clamp was bit-identical and was
  reverted. The real mechanism is lost *evidence* (two Costas blocks
  instead of three), not inflated scores.
- Widening `EMBEDDED_SYNC_LAG_S` to ±1.75 s, reverted 2026-09-19 for
  the same misunderstood reason, correctly.

Both are here because the observation in a comment is evidence and the
explanation beside it may not be. See
`mfsk-core/tests/ft8_coarse_partial_blocks.rs`.
