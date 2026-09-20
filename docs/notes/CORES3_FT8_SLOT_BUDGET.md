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

**The refine budget is 44 % unproductive.** 6.58 of the 15 stage-3
slots produce nothing, every slot, in every arm — the hit rate is
51-58 % across five builds and does not move with anything tried.
**That is ten times the boundary loss and it is the largest single
number in the table.**

### …and it is the sensitivity floor, not a recoverable loss

Called "waste" here at first, and it is not. Three levers were
measured against it and all three came back empty:

- **depth** — pass-1 ranks 16-25 convert at ~0.1 % (§4)
- **allocation** — `share_cand_budget` gains ~0 on a centred grid,
  because this band's deferred candidates hold no stations (§4)
- **duplicates** — `how_much_of_the_refine_budget_is_the_same_carrier_twice`
  finds **0 of 615** refined candidates to be a second look at a
  carrier already in the set, across 41 phases. Coarse sync already
  dedupes; the `dedupe+sort` step in its own profile line is that.

So the 15 are 15 distinct carriers and 6.6 of them do not decode,
which leaves two populations: noise peaks coarse sync cannot tell from
signal, and real stations too weak for a decoder with **no OSD, no SIC
and no AP**. `m5stack-cores3-app`'s own note has already priced the
second one — "it decodes 7 on `qso3_busy` where host JTDX gets ~18;
that gap is the cost of the leanness, and it is the right trade for
battery-budgeted field operation. **Not a bug to chase.**"

**Treat the 44 % as that trade, not as an opportunity.** Everything
that would move it — OSD, SIC, AP, a better coarse discriminator — is
either deliberately excluded or is a decoder project rather than a
budget one.

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

### Built, measured, and abandoned — 2026-09-20

Increment 1 was "report and act on nothing", and it reported enough to
end the line of work the same afternoon.

**First cut, medoid + `r`.** On a radio with a healthy NTP grid
(decode median +0.04 s) eight consecutive slots read −0.85, +0.91,
+0.28, +0.36, +0.04, +0.04, +1.64, +0.04 with `r` between 0.87 and
0.98 throughout. Three right out of eight, and `r` cannot tell which
three. That much was already written down: `acquire_slot_phase` marks
itself superseded for exactly this, "`r` up to 1.00 while being ~7 s
wrong", and says to select on score mass instead.

**Second cut, `circular_dt_clusters` mass.** On the same band the
gross outliers vanished — eleven slots, every reading within 0.29 s of
the decode median, `sd` 0.159, at **157 ms** a slot. Which proves only
that it is not lying when the grid is *already fine*.

**The question it exists for, swept on the host**
(`mfsk-core/tests/ft8_wide_grid_probe.rs`, offsets −2.4..+2.4 s in
0.2 s steps, three recordings):

| | inside ±0.88 s | **beyond ±0.88 s** |
|---|---|---|
| `qso3_busy` | 9/9 | **10/16** |
| `qso1` | 9/9 | **5/16** |
| `qso2` | 5/9 | **4/16** |

**19 of 48 where it matters.** It is right where it is redundant and
wrong where it is needed, and the errors are systematic rather than
noisy: on `qso1`/`qso2` a grid 1.4 s early reads back as 0.2 s early —
the probe pulls toward the middle of its own window instead of
following the displacement.

`dom` does not rescue a single reading either. Means of 5.2 on hits
against 3.5 on misses for `qso3_busy`, and **2.9 on hits against 3.7
on misses** for `qso2` — the wrong way round. There is no per-slot
statistic here that a consumer could gate on.

Running the whole band instead of `allsum_head`'s lower half was
tried, on the theory that half the stations meant half the mass:
9/16, 4/16, 4/16. Slightly worse, for double the time.

**Why it cannot be fixed by a better statistic.** Negative lag
truncates Costas block 0 from −0.48 s (`jstrt` = 6) and positive lag
truncates block 2 from +0.88 s, so **every candidate in the region of
interest is a partial-evidence score** and the real stations lose
their advantage precisely where the probe needs them to keep it. It is
the same fact that killed the ±1.75 s widen (§7), and the summary
statistic was never the problem. `ft8::acquire` uses three tiles over
25 s for this reason, and even so its first cluster is "usable 23
times in 40" and it resolves the rest by *trying a decode*.

The wiring is reverted; the sweep stays as the record.

### Where robustness goes instead

The gap is real and the cheap in-slot probe does not close it. What is
left, in order of how much of the 40 s of dark band each removes:

1. ~~Make acquisition shorter.~~ **Measured 2026-09-20: shortening is
   a wash, and the same measurement found something better.** See
   below.
2. **`share_cand_budget` as drift insurance** (§4). It gains ~0 on a
   centred grid, and the board's decodes reach +0.84 s against a
   deferred edge at +0.86 s — so it is what keeps stations from being
   dropped outright as the grid walks toward that edge.
3. **Keep the grid from drifting at all**, which is where the RTC and
   NTP work of this week went and is the only one of the three that
   has already paid.

## 9. Acquisition: the capture length is not the lever, the order is

`tests/ft8_acquire_capture_length.rs`, 40 capture phases per point,
three recordings, scored the way the board accepts — the true phase
within ±1.0 s of one of the five clusters it would try, since a trial
corrects its centre by the median DT of what it decoded.

| tiling | capture | qso3 | qso1 | qso2 | total | E[outage] |
|---|---:|---:|---:|---:|---:|---:|
| 3 × ±2.5 s @ 5 s (shipped) | 25.0 s | 40/40 | 34/40 | 32/40 | **88 %** | 42 s |
| 2 × ±3.75 s @ 7.5 s | 22.5 s | 34/40 | 34/40 | 28/40 | 80 % | 41 s |
| 2 × ±4.5 s @ 6 s | 21.0 s | 37/40 | 30/40 | 27/40 | 78 % | 41 s |
| 2 × ±5.0 s @ 5 s | 20.0 s | 36/40 | 26/40 | 18/40 | 67 % | 44 s |
| 1 × ±6.24 s | 15.0 s | 32/40 | 28/40 | 20/40 | 67 % | 39 s |

The shipped tiling reaching 40/40 on `qso3_busy` is the instrument
agreeing with the record — `acquire_slot_phases`' own doc says "one of
the first five every time" for that recording.

**Every shortening loses as much success as it saves time.** Expected
outage — time to a *successful* acquisition, retrying on failure —
sits at 39-44 s across the whole table. Capture length is not the
lever.

### What the same table found instead

**One tile over the first 15 s already succeeds two thirds of the
time**, and those 15 s are collected before the shipped acquisition
has finished listening. So the question is not how long to capture but
what order to do the work in: try one tile as soon as a slot exists,
and keep capturing only if it misses.

That is worth something only if the three-tile stage succeeds on the
cases the one-tile stage failed. Measured:

| | 1 tile | 3 tiles | **3 given 1 missed** | E[outage] |
|---|---:|---:|---:|---:|
| `qso3_busy` | 32/40 | 40/40 | **8/8** | 24 s vs 37 s |
| `qso1` | 28/40 | 34/40 | **9/12** | 27 s vs 44 s |
| `qso2` | 20/40 | 32/40 | **14/20** | 34 s vs 46 s |

**70-100 % of the first stage's misses are rescued, and the reason is
geometric rather than lucky.** One tile at ±6.24 s covers 12.48 s of
the 15 s period, so 2.52 s — 17 %, about 6.7 of 40 phases — is outside
its reach by construction. `qso3_busy` misses exactly 8. The second
stage's tiles at 5 s and 10 s are precisely what covers that hole, so
the rescue is a property of the tiling and not of the recording.

**Expected dark band falls 26-39 %** for the same total work, ordered
so the common case exits early.

Caveat on the absolute numbers: the stage times (20 s for one tile,
38 s for the full sequence, 37 s shipped) are read off the code's own
measurements — "three tiled searches at 543-635 ms each and up to five
full-slot decodes at ~1.1 s, 10-15 s in total" — and not measured for
an implementation that does not exist yet. The **ratio** is what the
table supports.

### What it would take

Not a parameter. `arm_acquisition` fills a ring to
`ACQUIRE_CAPTURE_SAMPLES` and `decode_pipeline` waits for the whole
thing; a progressive acquisition needs the ring readable at 15 s,
extendable to 25 s if the first stage misses, and a trial loop that
can run twice against a growing buffer. The `rust_oom` recorded beside
`ACQUIRE_CAPTURE_SAMPLES` is the standing warning about what a second
buffer costs.
