# mfsk-wasm-bench

A minimal `wasm-bindgen` harness for measuring the effect of
`-C target-feature=+simd128` on FT8 decode under
`wasm32-unknown-unknown` (issue #208). Not a real API surface, not
published, not a workspace member — see the root `Cargo.toml`'s
`exclude` list for why.

This is a manual/periodic investigative tool, not a CI gate: no
`wasm32` target exists in this repo's CI today, and a timing check
would be flaky on shared runners. Re-run it by hand when re-measuring
the `+simd128` effect or profiling a candidate dense-kernel
vectorization (Part 2 of #208).

## Build

Requires the `wasm32-unknown-unknown` rustup target, `wasm-pack`, and
Node.js. Optionally `wasm-objdump` (from the [WABT] toolkit) to confirm
vectorization took — `build.sh` runs this automatically and prints the
count.

```sh
rustup target add wasm32-unknown-unknown
cargo install wasm-pack

./build.sh                 # +simd128 (the checked-in .cargo/config.toml default)
./build.sh --no-simd128    # scalar baseline, for the before/after comparison
./build.sh --profiling     # keep wasm name-section symbols, for `node --prof` (Stage C)
```

`--no-simd128` works by setting `RUSTFLAGS=""` for that build, which
wins over `.cargo/config.toml`'s `target.wasm32-unknown-unknown.rustflags`
per Cargo's documented precedence (env var > target-triple config).
Verified empirically 2026-07-28: `+simd128` build emits ~8700
`v128`/`f32x4`/`i32x4` instructions in the compiled `.wasm`, the
`--no-simd128` build emits 0. `build.sh` prints this count on every
build — don't trust a comparison run without checking it.

[WABT]: https://github.com/WebAssembly/wabt

## Run

```sh
node bench.mjs [runs]
```

Defaults to `runs=7` and the repo's own committed
`embedded-poc/assets/qso3_busy.wav` fixture (also
`ft8_qso3_full_parity_recall.rs`'s golden WAV — known-correct decode,
so you can eyeball recall alongside timing, not just watch the number
move). Prints one warmup call's decode output (message|freq_hz|dt_sec
per line) followed by `runs` steady-state timings and their median.

To reproduce the exact numbers from issue #208 / `docs/notes/BENCHMARKS.md`'s
WASM section, point at the WAVs used there — `sim_busy_band.wav` /
`sim_extreme_hard.wav`, `ft8sim`-generated stress WAVs from a sibling
`webft8` checkout, not committed to this repo:

```sh
MFSK_BENCH_WAV=/path/to/sim_busy_band.wav node bench.mjs
```

## Decode budget (`DecodeRequest::budget`)

```sh
MFSK_BENCH_BUDGET_MS=5,10,20,40 node bench.mjs
```

Runs `decode_wav_budget(audio, ms)` at each budget and prints the
station count alongside the `BudgetReport`. This is the browser case the
option exists for: a tab that must keep a frame cannot afford the worst
slot's decode.

Measured here on `qso3_busy.wav` (Node, `+simd128`, no rayon):

| budget | stations | wall clock | report |
|---:|---:|---:|---|
| 5 ms | 0 | 13.4 ms | `exhausted, ran=0, skipped=17, cut_at_sync=21` |
| 10 ms | 0 | 13.0 ms | `exhausted, ran=0, skipped=17, cut_at_sync=21` |
| 15 ms | 8 | 15.3 ms | `exhausted, ran=8, skipped=9, cut_at_sync=17` |
| 20 ms | 14 | 23.8 ms | `exhausted, ran=15, skipped=2, cut_at_sync=7` |
| 40 ms | 14 | 27.6 ms | not exhausted |

Two things to read off it. The **sync triage sweep is a floor** — it is
never budget-gated, so anything under ~13 ms returns nothing *and still
costs 13 ms*; `max_cand` is the knob for that, not this one. And the
schedule works: at 20 ms it declines 2 of 17 candidates and loses no
stations, because `cut_at_sync=7` says the two it dropped were barely
above the triage gate. A `cut_at_sync` near 21 would mean the opposite.

## Example before/after

```sh
./build.sh --no-simd128 && node bench.mjs
./build.sh              && node bench.mjs
```

Compare the two `median:` lines. Decode output (the message lines)
should be byte-identical between the two runs — that's the recall
invariant any dense-kernel vectorization work (Stage D of the #208
plan) must preserve; a difference there is a correctness bug, not
noise.

## Profiling (Stage C)

```sh
./build.sh --profiling
node --prof bench.mjs 200      # more iterations = better sample resolution
node --prof-process isolate-*-v8.log > profile.txt
```

`--profiling` alone isn't enough to get real function names out of
`--prof-process` (you'd otherwise see `wasm-function[N]`) — two things
have to both be true: rustc needs to emit name/debug info in the first
place (`build.sh --profiling` sets `CARGO_PROFILE_RELEASE_DEBUG=2` for
this), and `wasm-opt` needs to not strip the wasm `name` custom section
afterwards (`Cargo.toml`'s `[package.metadata.wasm-pack.profile.profiling]`
passes `-g` for this). Both are already wired up — just use
`--profiling`, don't reach for `--dev` (unoptimized, not representative
of the real hot-loop shape) to get symbols.

Clean up `isolate-*.log` / `profile.txt` after — they're gitignored but
sizeable and not useful past the investigation that produced them.
