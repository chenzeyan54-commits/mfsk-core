## CLAUDE.md (lines 187-195, corrected)

**A-priori decoding is a general option that got coupled to sniper by
accident.** The AP engine broke out of its candidate loop on
`if has_ap`, so a hint — not a narrow search — was what made it
single-target; and it ran a *parallel, shallower* per-candidate ladder
(OSD depth-2 only, no Top-K rescue) that cost most of the decodes — 4
against 11 on the FT4 golden, measured. Both are gone: AP is now a rung
on `process_candidate_basic`'s own ladder, reaching FT4 and every
FST4 sub-mode, and `msg::pipeline_ap` is 96 lines of hypothesis
generation with no engine of its own. Full writeup in `docs/notes/DESIGN_RATIONALE.md` §3.

---

## docs/reference/LIBRARY.md (lines 636-641, corrected)

**A-priori decoding is a general option, not a sniper feature.** AP is
a rung on `process_candidate_basic`'s own ladder, reaching FT4 and
every FST4 sub-mode; `msg::pipeline_ap` is hypothesis generation with
no engine of its own. It used to be coupled to the sniper by accident
and that cost most of the decodes — the measurement is in
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md).

---

## docs/reference/LIBRARY.ja.md (lines 656-661, corrected)

**事前情報デコード (AP) は sniper の機能ではなく一般の選択肢である。**
AP は `process_candidate_basic` 自身の ladder の一段であり、FT4・
FST4 全サブモードに届く。`msg::pipeline_ap` は仮説生成だけで自前の
エンジンを持たない。かつて偶然 sniper と結合しており、それがデコードの
大半を失わせていた — 実測は
[`DESIGN_RATIONALE.md`](../notes/DESIGN_RATIONALE.md) にある。

---

## docs/notes/DESIGN_RATIONALE.md (lines 100-106, corrected)

**The engine is now gone.** AP is a rung at the end of
`engine::pipeline::process_candidate_basic`'s own ladder — everything
above it has already run and failed, so it can only add decodes — and
it therefore reaches FT4 and every FST4 sub-mode.
`msg::pipeline_ap` is what remains: `ap_passes` (the hypothesis set,
WSJT-X's `iaptype` equivalents) and `ap_bits_for`, 96 lines with no
engine of its own.