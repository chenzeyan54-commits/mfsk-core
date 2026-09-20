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

### Measured, 2026-09-20 — and the answer is "nothing is there"

81 phases per point, fixed-point, emit pair 87:

| | `qso3_busy` | `qso1` | `qso2` |
|---|---:|---:|---:|
| `max_cand` 15 (shipped) | 6.53 | 2.10 | 2.10 |
| `max_cand` 20 | 6.63 | 2.10 | 2.10 |
| `max_cand` 25 | 6.63 | 2.11 | 2.12 |
| `max_cand` 30 | 6.63 | **2.78** | **2.86** |

**Pass-1 ranks 16-25 convert at about 0.1 %.** Ten more candidates buy
one hundredth of a decode. The 44 % refine waste is not recoverable by
looking deeper — coarse sync's tail is noise, and the prior board
result (15→20→25 giving 8→7→8) was right for the wrong fixture.

**The jump at 30 is not depth.** `max_cand_late = max_cand −
n_early_refined` is 30 − 27.8 = 2.2 there, so the deferred path
receives a budget for the first time. The proof is that
`share_cand_budget` reaches the same place at `max_cand` 15 — 2.75 and
2.86 against 2.78 and 2.86 — for **half the compute**.

| candidate class | count | decodes bought | per candidate |
|---|---:|---:|---:|
| ready, pass-1 ranks 16-25 | 10 | +0.01 | ~0.1 % |
| deferred | 2.2-3.0 | +0.68-0.76 | **~30 %** |

A deferred candidate is worth about 250 ready ones at those ranks, and
the shipped allocation gives the deferred path **zero**.

### Which settles the emit point too

| | `qso3_busy` | `qso1` | `qso2` |
|---|---:|---:|---:|
| 85, arrival allocation | 6.02 | **0.64** | **0.62** |
| 85, value allocation | 6.31 | 2.74 | 2.86 |

Emitting earlier raises `defer` (2.2 → 7.0 on `qso1`), so moving the
emit point *without* fixing the allocation collapses the recording that
has stations in the deferred set. Fixed, it returns to the level of
87 + value. **So the emit point does not buy decodes; it buys the
fraction that lands in time** (on the air, late 0.70 → 0.04), and it
requires the allocation change as a precondition.

Three caveats, because the numbers above are host fixtures:

1. **The allocation's gains land after the boundary.** Deferred
   candidates are refined on the full slot, which arrives at the
   boundary, so they raise `dec` and not `intime`. They are decodes
   for the screen and for the next period's choice.
2. **`qso1`/`qso2` are more sensitive than the band.** The board's own
   emit-85 arm lost 0.23 decodes where `qso1` loses 70 %, because on
   this band the decodable stations are mostly in the ready set.
3. **The board's `defer` is 1.2-1.8**, between `qso3_busy`'s 1.0 (no
   gain) and `qso1`/`qso2`'s 2.2-3.0 (+0.7). Expect +0.2 to +0.5.

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

1. ~~Measure the pass-1 16-30 hit rate on the mirror.~~ **Done
   2026-09-20: ranks 16-25 convert at ~0.1 %, and the value is in the
   deferred candidates instead — see §4.** Depth is a dead end;
   allocation is not.
2. **Instrument rather than A-B** for anything whose predicted effect
   is under ~0.8 decodes: the block-2 gate's `gate2=`, and
   `share_cand_budget`'s reallocation count.
3. **Turn `share_cand_budget` on.** The mechanism is now confirmed
   rather than assumed — it buys exactly what doubling `max_cand`
   buys, for no extra work, and degenerates to today's behaviour when
   `defer` is 0. The board measurement that kept it off would need
   ~200 slots an arm to resolve +0.3, which is more radio time than
   the risk justifies for a reallocation that cannot cost work.
4. **Then the emit point becomes a latency decision, not a recall
   one.** With the allocation fixed, 85 costs no decodes and moves
   0.7 a slot from after the reply boundary to before it. Revisit it
   there, and leave the block-2 gate off until its `gate2=`
   instrumentation says it ever fires on the air.

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

## 8. Robustness: the gap between ±0.88 s and a 25 s acquisition

Everything above is about yield. This section is about the failure the
receiver actually has in the field, which is not yield at all.

```text
grid error ≤ 0.88 s   the per-slot search absorbs it
grid error > 0.88 s   ???
                      → 3 under-par slots → 25 s capture → 10-15 s of
                        compute → ~40 s with the band dark
```

There is no middle. On a radio on 2026-09-19 the grid sat 1.65 s out,
every slot decoded nothing, and the receiver spent **two minutes and
two acquisitions** getting back — while the stations were in the audio
the whole time.

### Why the obvious fix does not work

Widen the search. It was tried, to ±1.75 s, and reverted: candidates
appeared at −1.64 / +1.72 / +1.08 with scores of 20-30, `ready` fell to
14, and nothing decoded. §7 has the mechanism — a lag past the row
bound leaves the candidate scored on two Costas blocks instead of
three, the ratio is not penalised for the missing evidence, and
`PASS1_LIMIT` is 30, so junk crowds the shortlist.

And the width is bounded anyway:

```text
lag_max(P) = (2P − 163) × 0.08     P = 87 → 0.88 s,  P = 92 → 1.68 s
```

**±2.5 s is not reachable on the embedded time grid at any emit
point.** The host does not reach it either — `bounded_sync_lag_steps`
clamps to 62 steps and block 2's last symbol lands at 387 of 371 rows,
so upstream at ±2.5 s is *also* running on a truncated block 2 at the
extremes. "±2.5 s" means "two blocks are acceptable at the edges".

**So the streaming head start and the search width are the same
resource** — rows of spectrogram — and at a fixed emit point, decoding
early and searching wide are in direct competition.

### The split that resolves it

The same separation that fixed the emit/late problem: **decoding fast
and knowing where the grid is are different jobs, and only one of them
needs the answer before key-up.**

| | window | gated | runs on | purpose |
|---|---|---|---|---|
| decode search | ±0.88 s | yes | the bundle, before the boundary | this period's reply |
| **grid probe** | **±2.5 s (clamps to ±2.48)** | **no** | the same bundle, after the boundary | where the grid is |

The probe may accept two-block candidates precisely because it does
not decode. It is looking for the **circular median dt of a cluster**,
and an outlier that scores high on two blocks does not move a median.
Phantoms are free here in a way they are never free in a decode.

It needs no new spectrogram, which is what makes it affordable: the
bundle already carries 174 rows and the allsum is lag-independent, so
the probe is the same data with more lag bins.

```text
n_lag = 27  (±1.0 s)   coarse 103 ms measured on the board
n_lag = 63  (±2.48 s)  ~240 ms estimated, same band, same cores
```

**And it costs nothing in steady state**, because it only runs when the
grid is unproven — the condition that today arms a 25 s capture. The
trade it offers is one slot and ~240 ms against forty seconds of dark
band.

### Increments

1. **Measure and report only.** Run the probe, log its dt estimate and
   the agreement among its candidates, act on nothing. On the air this
   says whether a wide search on a partly-filled bundle produces a
   number worth trusting — which is exactly the question the ±1.75 s
   experiment failed to ask before wiring its output into the search.
2. Act on it: a one-shot grid shift, the way cold acquisition applies
   one, when the probe and the under-par run agree.
3. Only then consider whether acquisition's 25 s capture is still the
   right answer for errors past 1.68 s, or whether it becomes the rare
   fallback it was always meant to be.
