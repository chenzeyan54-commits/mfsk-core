# FT8 感度ベンチマーク — 環境セットアップ

クリーンチェックアウトから FT8 の AWGN/フェージング SNR スイープ
(`tests/ft8_sweep.rs`) を再現する手順。[`FT4_BENCHMARK.ja.md`](FT4_BENCHMARK.ja.md)
・[`FST4_BENCHMARK.ja.md`](FST4_BENCHMARK.ja.md) と同じ **WSJT-X Fortran
ソース → simN シミュレータ → WAV コーパス → `cargo test --ignored`**
というパイプラインを `ft8sim` に適用したもの。

これは既存 CI の「ft8 characterization」スイート
（`ft8_decode_block_snr_sweep` 等、`.github/workflows/ci.yml` 参照）とは
別物 — あちらは自前の LCG ノイズ生成器で信号を合成しており、WSJT-X 本家
のground truthとは無関係。`ft8sim` は WSJT-X 自身の Fortran シミュレータ
（`fst4sim`/`ft4sim` と同じ系統）なので、そこから生成したコーパスは
自己無矛盾チェックではなく真の Watterson フェージングリファレンスになる。

## 1. 前提パッケージ / 2. `ft8sim` をビルド

パッケージ・手順とも [`FT4_BENCHMARK.ja.md`](FT4_BENCHMARK.ja.md) の
1-2 節と同一 — `ft8sim` は FT4 と全く同じ共有 lib サブツリー
（LDPC(174,91)、CRC-14、`sgran` 乱数シードを共有）を再利用する:

```sh
sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
scripts/build_ft8sim.sh [/path/to/WSJT-X] [out-dir]
# デフォルト: WSJT-X-dir = ../WSJT-X, out-dir = target/ft8sim/
```

動作確認:

```sh
target/ft8sim/ft8sim "CQ JL1NIE PM95" 1500 0.0 0.0 0.0 1 -15
ls 000000_*.wav
```

（`ft4sim` と同じ 7 引数 CLI: `message f0 DT fdop del nfiles snr` —
T/R 周期引数はなし、FT8 にもサブモードはない。）

## 3. WAV コーパスの生成

```sh
scripts/gen_ft8_sweep_wavs.sh [ft8sim-path] [out-dir]
# デフォルト: ft8sim-path = target/ft8sim/ft8sim
#           out-dir     = embedded-poc/assets/ft8_sweep/
```

同じく 4 チャネル × グリッド × `TRIALS` 構成。デフォルトの `SNRS`
グリッド（-5〜-26 dB）は FT8 の公称 WSJT-X AWGN 閾値である約
**-20〜-21 dB**（2500 Hz 基準帯域幅）を挟む。

## 4. スイープの実行

```sh
MFSK_FT8_SWEEP_DIR=embedded-poc/assets/ft8_sweep \
  cargo test --test ft8_sweep --release \
  --features ft8,fft-rustfft,parallel,uvpacket \
  -- --ignored --nocapture
```

### 実測値 2026-07-18（このコーパス/シード）

線形補間による 50% クロス点:

| チャネル | 50% クロス点(概算) |
|---|---:|
| AWGN | ≈ -20.4 dB |
| CCIR good | ≈ -20.0 dB |
| CCIR moderate | ≈ -18.3 dB |
| CCIR poor | ≈ -18.2 dB |

AWGN と CCIR good は公称値 -20/-21 dB の約 1 dB 以内に収まっており、
FT4 が公称値 -17.5 dB に対して見せた約 2 dB のギャップ
（`FT4_BENCHMARK.ja.md` 参照）よりかなり良い一致。これは FT8 の
本番 `decode_frame` が（FT4/FST4 が使う汎用 `engine::pipeline` 経路
ではなく）#48 統合後の WSJT-X 忠実パイプライン `decode_block` を
通っていることと整合する。CCIR moderate/poor はより大きい約
2〜2.5 dB のギャップを見せており、未調査 — フェージング下の感度が
優先課題になった場合、`FST4_BENCHMARK.ja.md` 6 節の「直す前に診断する」
アプローチの候補になる。

## 5. `DecodeStrictness` probe はここにはない

FT4/FST4 のスイープと異なり、今回は strictness 較正 probe を含めて
いない。FT8 の本番経路は `process_candidate`（`pub` ではない）を、
FT4/FST4 が共有する未較正コピー（issue #72）とは別の、既に較正済みの
`ft8::decode::DecodeStrictness`（`src/ft8/decode.rs` の
"Calibrated from real WAV bench 2026-04-07" というコメント参照）と共に
呼んでいる。外部テストからこれを振る public なフックがない。今回の
スイープが FT8 に初めてもたらすのは、新しい較正対象ではなく、既存の
較正値を検証するための体系的な Watterson フェージングコーパスである。

## 6. 旧 CI 「ft8 characterization」スイート — 2026-07-18 削除

`.github/workflows/ci.yml` には push-only の matrix entry
`ft8 characterization` があり、`ft8_coarse_sync_concurrent`、
`ft8_decode_block_coarse_diag`、`ft8_decode_block_depth_sweep`、
`ft8_decode_block_pass1_sweep`、`ft8_decode_block_snr_sweep`、
`ft8_qso3_coarse_sync_probe`（`ft8/**` push 毎に約10分）を含んでいた。
6件全部を監査した上で、matrix entry ごと削除:

- 6件全部が `println!` のみの診断で **assertion が一切ない** —
  数値が何であれ絶対に fail しないので、かかっていた時間に見合う
  regression 検知能力がゼロだった。
- `ft8_coarse_sync_concurrent` は `fixed-point` feature 必須だが、CI の
  `full` feature set には含まれていない — この cleanup 以前から
  静かに 0 テスト実行だった。
- `ft8_decode_block_snr_sweep` が実際に自前 LCG ノイズ生成器で AWGN を
  合成していたもの（フェージングモデルなし）で、外部の ground truth
  なしに自分たちの decoder 2 つを比較するだけ — 「golden でない
  simulator で検証している」という指摘に最も直接該当する。
- 残り 4 件は実録音（`REAL_QSO_WAVS`、`embedded-poc/assets/` に
  チェックイン済み）を使っていたので合成データではなかったが、
  それでも assertion はなかった。

FT8 の recall regression 検知は既に別の場所でカバーされており、
代替は不要だった: `ft8_qso3_apoff_recall` / `ft8_qso3_apon_recall` は
hard-assertion で常時実行（`default` suite、`#[ignore]` されていない）
— これが実際に recall regression を検知するテスト。

別の `ft8 recall` matrix tier（`ft8_decode_block_real_qso`、
`ft8_reference_suite_recall`）も同じ問題を抱えていることが判明し、
同日中に対応済み: `ft8_reference_suite_recall`（PASS1_LIMIT/max_cand
の組込チューニング用 config sweep、情報提供のみ）は削除。
`ft8_decode_block_real_qso` は hard-assertion floor テストに変換
（embedded ship config vs host `decode_frame` の truth を
`qso1`/`qso2`/`qso3_busy` で比較 — `qso1`/`qso2` は WSJT-X golden が
なく他に一切テストされていなかった）、`#[ignore]` も外したので
matrix tier 自体を削除 — 今は `default` で実行される。

## 7. CCIR moderate/poor のフェージング差 — 診断して解消 (issue #72 follow-up, 2026-07-18)

§4 で未解決として挙げた項目を、`FT4_BENCHMARK.md` §8〜§12 が FT4 の AWGN
差を潰したときと同じ「直す前に診断する」規律で追った。その調査が残した
2つの教訓込みで — **仮説は実際に狭域スイープを回して裏付けるまで信用しない**、
そして**診断コードが本物のデコード経路と同じゲートを適用していることを
確かめてから「救えた」と言う**。

**まず否定したもの（土俵が違い、比較として無効）**: `process_candidate` の
未使用な `EqMode::Local`（本番の呼び出し側は全て `EqMode::Off` を直書き）が
フェージング下で効くのではないかと考え、`decode_frame`/`process_candidate`
に `eq_mode` の公開フックが無いため `decode_sniper_eq`（`target_freq ± 250 Hz`）
を代用にして外から振った。**レビュー後に訂正**: sniper モードはハードウェアの
roofing filter に合わせるための機構であり、`EqMode::Local` は BPF スカート
による歪みに特化して調整されたものである。WSJT-X との比較にも、
「等化一般がフェージングに効くか」の検証にも、どちらの代用にもならない。
（記録として、この無効な実験では `EqMode::Local` は試した全セルで CCIR
recall を悪化させた。BPF 向けの等化器を別種の歪みに誤用した結果として
筋は通るが、どちら向きの証拠としても採用していない。）

**本当の診断**: `ft8_diag_weak_trials`（`tests/ft8_sweep.rs`、
`ft4_diag_weak_trials` に倣う）を作り、`process_candidate` の前段
（`coarse_sync` → `fine_refine_3stage` → `nsync` ゲート）を、それ自身が
呼んでいる公開部品に対して直接再現した — `process_candidate` と
`process_one_candidate_inner` は `crate::ft8` の外では `pub` ではない。
§4 の交差点付近で落ちている CCIR moderate/poor のトライアルを横断して見ると、
**正解に近い候補はほぼ毎回見つかっており**、`fine_refine_3stage` は真の位置に
着地し、`nsync` もゲートを余裕で通っていた（21 中 10〜21）。粗同期と
fine-refine は健全である。ボトルネックはその下流、LLR/BP/OSD にある。

そこで内部へ入り（`ft8::decode::tests::ft8_diag_internal_osd_trace`、
`src/ft8/decode.rs` — `process_candidate` は private な `fn` で、同モジュール
自身の `#[cfg(test)] mod tests` から `use super::*` で届く）、
`process_one_candidate_inner` / `osd_strategy::try_fallback` に
`MFSK_FT8_OSD_TRACE` で切り替わる一時的なトレースを入れて（現在は削除済み）
段階ごとの `hard_errors` を見た。判明したのは、`OSD_HARDERRORS_MAX = 22`
（`decode_block/osd_strategy.rs`）— WSJT-X の一律 36 に対する mfsk-core 独自の
乖離で、`qso3_busy.wav` 上の CRC まぐれ当たりと判断した3候補を弾くために
締めたもの — が、**CCIR フェージング下で golden なデコードを捨てていた**
ことである。複数の独立した LLR 変種が、送信された正解テキスト
（`"CQ JL1NIE PM95"`、`ft8sim` 由来の既知の ground truth）に
`hard_errors` 20 台後半〜30 台前半で収束しており、その全てが 22 の天井で
拒否されていた。

**対照実験**: `OSD_HARDERRORS_MAX` を WSJT-X 自身の 36 に広げて再計測
（`ft8_snr_sweep`、本物の `decode_frame`）:

| チャネル | -21 | -20 | -19 | -18 | -17 |
|---|---:|---:|---:|---:|---:|
| CCIR moderate | 0%→0% | 10%→10% | 5%→25% | 55%→85% | 65%→85% |
| CCIR poor | 0%→5% | 10%→10% | 15%→35% | 45%→65% | 65%→90% |

スイープ範囲のどこにも退行なし。実録音に対するハード assertion のテスト
（`ft8_qso3_apoff_recall`、`ft8_decode_block_real_qso`）とも突き合わせ:
**バイト単位で同一** — 同じ 7/8 golden ＋ 7 ファントム = 計 14、同じ 4/4、
5/5、12/14+2。この緩和はそれらの候補には一切触れていない。

触れたのは別の集合で、その結果はこの節自身の履歴を書き換える。
`ft8_qso3_jtdx_recall.rs`（JTDX の積極的な 18 件リスト）が 13/18 から
**17/18** になった — 0.6.3 で `OSD_HARDERRORS_MAX = 22` がまさに
「ファントムと見なして排除する」ために導入された当の3候補
（`N1API F2VX 73` e=30、`N1API HA6FQ -23` e=25、`CQ EA2BFM IN83` e=31）を
そのまま回収した（導入前の recall は 16/18 だった）。本デコーダはいま、
その3候補について JTDX が主張するのと**同一のテキスト**に独立に到達して
いる。2つの別々のデコーダが同じ CRC-14 保護メッセージに収束するのは、
偶然に対する強い反証である（本当にランダムなら ~1/16384²）。よってこれらは
ファントムではなく実信号と再分類される。これは issue #150（JTDX-18 の
ground truth 疑義）にも実質的な材料を与える — 未検証だった 4〜5 件のうち
3件に独立の裏付けが付いた。`ft8_qso3_apon_recall.rs` のマルチパス extras の
下限も同様に 4/6 から 0.6.3 以前の 5/6 に戻った（同じ `CQ EA2BFM` の回収、
別のテスト）。

**採用**: `OSD_HARDERRORS_MAX` を WSJT-X の 36 に恒久的に設定（旧 22）、
`osd_strategy.rs` の doc コメントは撤回されたファントム根拠ではなくこの
経緯を記録するよう書き直し、`ft8_qso3_jtdx_recall.rs` の `MIN_JTDX_HITS` を
13→17、`ft8_qso3_apon_recall.rs` の `JTDX_EXTRAS_HARD_FLOOR_MULTIPASS` を
4→5 に引き上げた。非 ignore の全スイートと `-D clippy::perf` は通して green。

**全4チャネルの再スイープ**（`ft8_snr_sweep`、本物の `decode_frame`、
`-5`〜`-26` dB の全グリッド、§4 と同じコーパス／seed）— 50% 交差点、
線形補間:

| チャネル | §4（前） | 今回（後） | Δ |
|---|---:|---:|---:|
| AWGN | ≈ -20.4 dB | ≈ -20.8 dB | +0.4 dB |
| CCIR good | ≈ -20.0 dB | ≈ -20.6 dB | +0.6 dB |
| CCIR moderate | ≈ -18.3 dB | ≈ -18.6 dB | +0.3 dB |
| CCIR poor | ≈ -18.2 dB | ≈ -18.5 dB | +0.3 dB |

グリッドのどこにも退行なし（雑音フロアでは 0% のまま、強信号では 100% の
まま — 拒否専用ゲートを緩めたのだから単調であるべきで、そのとおり）。
交差点の移動量は、この節の前半に出てくる固定 SNR セルでの recall の跳ね方
（例: CCIR moderate -18 dB で 55%→85%）が示唆するより控えめである —
この領域では recall-vs-SNR 曲線が急峻なので、同じパーセントの跳ねが
50% 点では小さい dB 差にしか写らない。AWGN の ≈-20.8 dB は WSJT-X 公称の
-20〜-21 dB の**内側**に入った（以前は端）。CCIR good が4つの中で最も
得をした（+0.6 dB）が、§4 ではフェージング由来の OSD 天井圧力が最も
小さかったチャネルである — これは修正の機構（フェージング特有ではなく、
境界上のデコードを一般に回収する）と整合し、フェージング限定の効果では
ないことを示す。CCIR moderate/poor の +0.3 dB は本物だが、「85%/90%」と
いう定性的な recall の数字が単独で示唆するほど大きくはない — 固定 SNR セル
の数字と 50% 交差点の数字はどちらも正しく、答えている問いが違うだけである
（特定の動作点での recall か、モード横断比較に使う閾値の定義か）。

## 8. SIC LPF 窓のバグ — JTDX 17/18 → 18/18 で解消 (issue #180, 2026-07-25)

§7 の `OSD_HARDERRORS_MAX` 緩和の続き。あれで `ft8_qso3_jtdx_recall.rs` の
JTDX-18 recall は 13/18 → 17/18 になり、取りこぼしはちょうど1件残った:
`WA2FZW DL5AXX RR73` @ 2545.88 Hz。当時は JTDX の偽陽性だろうと分類した
（`coarse_sync` はその周波数付近に候補を見つけるが、メッセージを復元しない）。

**その分類は誤りだった。** issue #180 の調査（もともとは*別の*取りこぼし
`CQ DX DL8YHR JO41` @ 2606.25 Hz、同じく `qso3_busy.wav` を追っていた）が
本物の WSJT-X `jt9 -d3` ビルドで ground truth を取ったところ、jt9 自身が
この WAV で `WA2FZW DL5AXX RR73` を復号する — JTDX の産物ではなく実信号
だった。jt9 のディスク復号は段階的／チェックポイント式である（平坦な単一
パスではない）: 3つのチェックポイントで音声の先頭部分を順に長くしながら
復号し、次の難しい段階の前にそこまでの確定デコードを減算する — DL8YHR は
そうして早期に減算された13信号（`WA2FZW` を含む）が取り除かれて初めて
復号される。

同じ信号に対して mfsk-core の SIC が `subtractft8.f90` より多くの残差を
残す理由を追い（同期/LLR/BP/OSD のバグではない — 決定的な試験として、
jt9 が吐いた SIC 後の残差バッファを mfsk-core の無改造デコード鎖に
そのまま読ませたところ DL8YHR がそのまま復号された）、
`subtract_tones_lpf` の LPF カーネルに行き着いた: `normalized_kernel` の
cos² 窓が、引数を `NFILT`（`= 2*lpf_half`）ではなく `lpf_half` で割って
おり、テーパーの引数範囲が `[-pi/2, pi/2]` から `[-pi, pi]` へ倍になって
いた。`cos²` が単調なテーパー（中心で 1 → 端で 0）になるのは前者の範囲
だけである。倍の範囲では 1/4 点で 0 まで落ちたあと、**真の端で 1 — 全重み —
まで戻る**（FT8 の `lpf_half=2000` なら、現在のサンプルから `lpf_half`
サンプル＝166 ms 離れた位置）。数値で確認:

| オフセット（サンプル） | 出荷時のカーネル | 正しいカーネル |
|---|---|---|
| 0（中心） | 1.000 | 1.000 |
| 1000 | 0.000 | 0.500 |
| 2000（端） | **1.000** | **0.000** |

つまり、これが FT8/FT4 の正規 SIC 経路になった v0.6.2 以降の全ての
`subtract_tones_lpf` 呼び出し — 本ドキュメント §1〜§7 のうち SIC に依存する
`decode_frame_subtract` 系を通る数値すべてを含む — が、166 ms 前の古い
チャネルサンプルに現在のサンプルと同じ重みを与える、ひどく歪んだ
「ローパス」で動いていた。QSB/チャネル推定を平滑化するどころか積極的に
壊していたことになる。修正は式1行（`lpf_half` ではなく `2*lpf_half` で
割る）。同じバグ・同じ修正が FT4 にも当てはまる（`subtractft4.f90` は
同一の窓の式を使っている）。

**結果**: `ft8_qso3_jtdx_recall.rs` が **17/18 → 18/18** — `WA2FZW DL5AXX
RR73` が復号されるようになり、新規ファントムゼロ、WSJT-X 8件 golden にも
退行なし（依然 7/8、`K1BZM DK8NE` が唯一の欠落 — 下の issue #182 参照）、
AP-on マルチパスの JTDX-extras 下限にも退行なし（当時は依然 5/6、同じ
残り1件）。非 ignore の全スイートは通して green。

**追記 (issue #182, 2026-07-26)**: `K1BZM DK8NE` の欠落は、この節が当初
指していた「AP リストの幅」仮説とは異なり、**AP リストの問題ではなかった**。
真因は `osd_decode_npre1`（この候補の `q=11` に対する WSJT-X の OSD
`ndeep=2` ディスパッチ）に、BP で精錬された `bp_llr_zsum` ではなく生の
チャネル LLR を食わせていたことである。本家の `decode174_91.f90` ドライバは
常に BP 後の LLR を OSD に渡す。既に計算済みの BP 精錬 LLR を OSD の
呼び出し側まで通す（BP 前のものを再計算・再利用するのをやめる）ことで
解消した: AP-on マルチパスの JTDX-extras 下限は **6/6** になり、
ship-config の `ft8_qso3_jtdx_recall.rs` の 18 件チェック（これも同じ OSD
呼び出し側を通る）も `K1BZM DK8NE` を AP ヒント無しで直接回収するように
なった。調査の全容は `CHANGELOG.md` を参照。

**適用範囲の確認 — 動くべきなのは帯域内 SIC のシナリオだけであり、§4/§7 の
`ft8_snr_sweep` は意図的に再実行していない。** あのスイープは1トライアル
あたり単一の目標信号を合成し同一チャネル干渉が無いので、
`subtract_tones_lpf` はそこでは一度も呼ばれない — 修正がそれらの数値に
影響する経路が存在しないため、再実行は割に合わないと判断した。これは
修正自身のコールグラフからの推論であって、§7 の表の再計測ではない。
傍証はある: WebFT8 の独立した `ft8-bench` シミュレータ群（WebFT8 リポジトリの
`docs/bench.md`、本リポジトリではない）は同日に全実行されており、帯域内
干渉の無いシナリオ（単一目標＋AWGN/BPF のみ）は前後でバイト単位に一致し、
帯域内 SIC のあるシナリオは全て動いた — この議論が予測するのと同じ境界が、
`ft8sim` とは別のコーパス／ハーネスで示されたことになる。この区別が特定の
判断で問題になるなら、このノートに頼らず §7 のスイープを直接回し直すこと。

**mfsk-core の issue/PR**: [#180](https://github.com/jl1nie/mfsk-core/issues/180)（調査）、[#178](https://github.com/jl1nie/mfsk-core/pull/178)（修正、マージ済み）。

## 9. AWGN/CCIR スイープ再計測、`DecodeDepth` 再設計はスイープに無影響と確認 (issue #182 follow-up, 2026-07-26)

§4 の「2026-07-18 測定」の表は、それ以降一度も回し直されていなかった —
§8 は再計測ではなく適用範囲の推論で済ませている。今回は直接回した。
きっかけは `DecodeDepth` の enum→struct 再設計と、無関係な
`auto_ap_strategy` の削除（同日に両方入った、`CHANGELOG.md` 参照）で、
どちらもこのスイープの数値に触れていないことを確認するためである。

**構造上、影響し得ないことを確認**: `ft8_snr_sweep` の `decode_wav_ft8` は
`decode_frame` → `decode_frame_inner` を呼ぶ。これは
`decode_block::decode_block_multipass`（`auto_ap_strategy::run` が持って
いた唯一の呼び出し元）とは別実装であり、`decode_frame_inner` は削除の
前後を通じて一度もそれを呼んでいない。再設計 PR 全体の `git diff` で検証済み:
`ft8/decode.rs`、`ft8/decode_block/*`、FT8 専用テスト、FFI/組込みグルーの
FT8 部分以外のファイルは1つも触られていない — 他プロトコルのスイープ経路は
そもそも存在しないので動きようがない。

**再計測**（`ft8_snr_sweep`、`--ignored --nocapture`、§4 と同じ
`-5`〜`-26` dB グリッド／セルあたり 20 トライアルのコーパス）:

| チャネル | 2026-07-18 | 2026-07-26 | Δ |
|---|---:|---:|---:|
| AWGN | ≈ -20.4 dB | ≈ -21.4 dB | -1.0 dB（高感度側） |
| CCIR good | ≈ -20.0 dB | ≈ -20.8 dB | -0.8 dB |
| CCIR moderate | ≈ -18.3 dB | ≈ -18.9 dB | -0.6 dB |
| CCIR poor | ≈ -18.2 dB | ≈ -19.0 dB | -0.8 dB |

4チャネルとも同じ向き（より負＝必要 SNR が低い＝良い）へ 0.6〜1.0 dB と
揃って動いた。`DecodeDepth` 再設計ではなく（上で無影響を確認済み）、
2つの日付のあいだに入った FT8 の感度修正すべての累積効果である
（OSD の `bp_llr_zsum` シード、§7 の `OSD_HARDERRORS_MAX` 緩和、§8 の
SIC LPF 窓修正、その他 `CHANGELOG.md` が追っているもの）。この表に
まとめ直されていなかっただけである。単一実行であり複数回の平均ではない —
0.5 dB 未満の差はセルあたり 20 トライアルのサンプリング雑音の内と見なす
こと（`BENCHMARKS.md` からリンクされている「疎な SNR サンプリング」の
教訓を参照）。ここで信号として読むべきは、4チャネルが揃って同じ向きに
動いたことであって、個々のセルではない。

この表（`BENCHMARKS.md` 側の写しではない。あちらは既に今日に近い中間値
— AWGN ≈-20.8、CCIR good ≈-20.6、CCIR moderate ≈-18.6、CCIR poor ≈-18.5
— を持っていた）が、このパス以降は古いものとして扱うべき表である。
`BENCHMARKS.md` の FT8 節は今日の数値に直接更新した。

**このパスで追加したもの**: `tests/ft8_qso3_full_parity_recall.rs`。
`ft8_qso3_apoff_recall.rs` の ship-config 7/8 下限とは別の新しい回帰で、
**ホスト研究構成**（`DecodeDepth::FULL`、`sync_min=0.8`、`max_cand=60`。
`ft8_qso3_jtdx_recall.rs` が既に使っていた既定と同じ）が WSJT-X の 8件
golden を完全に取ること（**8/8**、ship-config の `DecodeDepth::EMBEDDED`
では OSD がコンパイルから外れるため構造的に到達できない
`K1BZM DK8NE -10` を含む）を assert する。実測 **~139〜148 ms**
（シングル／マルチスレッドとも — この候補数では並列化があまり効かない）で、
本物の `jt9 -8 -d3` のファイル全体復号 ~1.1 s に対して ~7〜8 倍速く、
recall は完全に同等。「ホストがこの WAV で WSJT-X と厳密に一致するのに
どれだけ速いか」に対する追跡可能な答えが、これで常設の回帰テストになった。

## 10. CCIR moderate/poor の「WSJT-X 公称値なし」を本物の `jt9 -8 -d3` ground truth で置き換え (2026-07-26)

§4 のスイープ表（および `BENCHMARKS.md` の写し）は、CCIR good/moderate/poor
の「WSJT-X 公称値」欄に常に「—」を置いていた — WSJT-X は AWGN の公式閾値は
公表するが、フェージングモデル別の数値は公表しない。しかし `ft8sim`
（このスイープのコーパスを生成しているのと同じ WSJT-X 純正シミュレータ）は
本物の CCIR フェージング WAV を作れるし、本物の `jt9` バイナリがローカルに
あった（`~/wsjtx-build/jt9`。まず `qso3_busy.wav` で健全性確認 — 22/22
デコード、`FT8_BENCHMARK.md` 自身の過去の調査にある既知良好な参照と
バイト単位で一致）。よって「—」を恒久的な空欄にしておく理由は無かった。
`jt9 -8 -d 3` を AWGN/CCIR コーパス全体に直接かけた（3チャネル × 13 SNR点
× 20 トライアル = 780 ファイル、8並列で実時間 ~62 秒。ヒットの定義は
jt9 の標準出力に `JL1NIE` が現れること — Rust 側スイープ自身の
「既知メッセージ1つ」方式に合わせた）:

| チャネル | mfsk-core (§9) | 本物の `jt9 -8 -d3` | Δ (jt9 − mfsk-core) |
|---|---:|---:|---:|
| AWGN | ≈ -21.4 dB | ≈ -21.2 dB | mfsk-core が +0.2 dB 先行 |
| CCIR good | ≈ -20.8 dB | ≈ -20.75 dB | ほぼ同等 |
| CCIR moderate | ≈ -18.9 dB | ≈ -19.5 dB | **jt9 が +0.6 dB（差）** |
| CCIR poor | ≈ -19.0 dB | ≈ -19.7 dB | **jt9 が +0.7 dB（差）** |

AWGN と CCIR good は §9 の枠組みが既に前提していたことを裏付ける — 本物の
WSJT-X と同等かわずかに先行。**CCIR moderate/poor は別の話である**:
重めの Watterson フェージング下に、これまで文書化されていなかった
実在の ~0.6〜0.7 dB の感度差があり、「公称値なし」ではなく直接の ground
truth で裏付けられた。これは §7 が既に解消した `OSD_HARDERRORS_MAX` の
フェージング recall 問題とは別物である（あちらは `qso3_busy.wav` 上の
golden デコードの天井の話で、このスイープの「きれいな信号＋フェージング
チャネル」の 50% 交差点測定とは軸が違う）。これは本当に新しい、未解決の
差である。

**まだ真因は特定していない。** 本ドキュメント自身の「直す前に診断する」
規律（§6 の教訓、および `FST4_BENCHMARK.md` §6 の共通の手順書）に沿った、
次パスの候補方向: Watterson チャネルモデルの `fdop`/`del` はフェージングの
きつさに比例する（`ft8_sweep.rs` のモジュール doc によれば
`ccir_moderate` = 0.5 Hz/1.0 ms、`ccir_poor` = 1.0 Hz/2.0 ms）ので、
mfsk-core と本物の jt9 の違いは**チャネルの脱相関**と特に相互作用している
可能性が高い。最初に見るべき妥当な場所は `fine_refine_3stage` の
コヒーレント Costas ブロック合成（issue #182、`BENCHMARKS.md` に節あり）で、
これは参照窓を通じた位相安定性を仮定している。重いフェージングはその仮定を
部分的に崩し得るが、本ドキュメントはそこを確認していない。FT4 の同種の
コヒーレント合成修正で既に観測されている「フェージングチャネルは AWGN ほど
得をしない」パターンと似ている（`FT4_BENCHMARK.md` §9）。このパスでは
これ以上追っていない — 説明のつかない「—」を残すのではなく、次の具体的な
手がかりとしてここに記す。[#190](https://github.com/jl1nie/mfsk-core/issues/190)
として追跡 — **§11 で真因特定・解消済み**。上記の fine-sync の線は
行き止まりで、真の原因ではなかった。

セル別の生カウント、jt9 -8 -d3、セルあたり 20 トライアル:

| SNR | CCIR good | CCIR moderate | CCIR poor |
|---:|---:|---:|---:|
| -15 dB | 19/20 | 20/20 | 20/20 |
| -17 dB | 19/20 | 17/20 | 18/20 |
| -18 dB | 19/20 | 18/20 | 16/20 |
| -19 dB | 19/20 | 11/20 | 15/20 |
| -20 dB | 16/20 | 9/20 | 8/20 |
| -21 dB | 8/20 | 3/20 | 2/20 |
| -22 dB | 3/20 | 1/20 | 0/20 |

## 11. issue #190 の真因: 数値的な差ではなく `jt9` CLI の暗黙の CQ-AP (2026-07-26)

§10 が挙げた fine-sync の線（フェージング下でコヒーレント Costas 合成が
劣化する）は自然な次の容疑者だったが、直接追跡して否定された。ローカルの
`jt9` ビルド（`/home/minoru/src/WSJT-X/lib/ft8/ft8b.f90`、issue #180 の
`DL8YHR_PROBE` の前例に倣い `f1≈1500 Hz` で絞った `ISSUE190_PROBE` の
デバッグ出力）に計測を入れ、Stage A/B/C の `ibest`/`delfbest`/`xdt` と
結果の `nsync` を出力させ、jt9 は成功し mfsk-core の `decode_frame` は
失敗する3つのトライアル `ccir_moderate_m19_{01,05,14}.wav` に対して
走らせた（§10 の再現手順にあるトライアル別 CSV 差分で特定）。

**`nsync` は jt9 と mfsk-core でほぼ完全に一致した**（jt9: 14/16/18 に対し、
同じ3トライアルで mfsk-core を独立にトレースしても 14/16/18）—
fine-sync／コヒーレント合成の線は行き止まりであって原因ではない。実際の
分岐は1段あとに現れた: jt9 自身の `ipass` ループ（`ft8b.f90:311-471`）は
まず4つのブラインド LLR 変種で BP/OSD を試し（`ipass=1..4`）、
**3トライアルとも4つ全部が失敗**している — これは mfsk-core 側の
4変種全滅と完全に一致する — そして `ipass=5`、`iaptype=1`
（"CQ ??? ???"、固定の 29 ビット `mcq` パターンから作る WSJT-X の a-priori
仮説。オペレータ由来の mycall/hiscall を必要としない）で初めて成功する。
`nharderrors` は 27, 29, 29 で、ブラインド BP/OSD の天井は超えているが
`decode174_91` の AP 受理範囲（`nharderrors ≤ 36`）の内側である。

`-c`/`-x` で mycall を設定していないのに、なぜ全トライアルでこのパスが
発火したのか。事実は2つある: `lib/jt9.f90:302` がスタンドアロン CLI で
`shared_data%params%lft8apon=.true.` を直書きしており（GUI 側の
`FT8AP` 既定 false とは独立）、`ft8b.f90` の `naptypes(0,1:4)=(1,2,0,0)` が、
AP が有効なら idle 状態（`nQSOProgress=0`）の**すべての**候補に対して
`iaptype=1` を試す — "CQ" のビットパターンはコンパイル時定数であって
呼び出し側が与える仮説ではないので、外部ヒントは要らない。つまり本プロジェクトの
WSJT-X 同等性の作業で使われてきた `jt9 -8 -d3` の呼び出しは（このスイープの
参照測定自身を含めて）**すべて暗黙に無料の CQ-AP パスを含んでいた**。
そしてスイープコーパスのメッセージは `CQ JL1NIE PM95`
（`scripts/gen_ft8_sweep_wavs.sh`）、すなわちその無料パスが狙う当のメッセージ型
である。

mfsk-core には等価な機構が既にあった —
`decode_block::process_candidates::process_one_candidate_inner` の Step 4
AP ループには既に "Pass 12: blind-CQ (WSJT-X iaptype 1)" が含まれていた —
が、`if accepted.is_none() && let Some(ap) = ap_hint` でゲートされており、
**呼び出し側**が `ApHint` を渡したときにしか走らなかった。`decode_frame`
（このスイープの `decode_wav_ft8` が呼ぶ、ヒント無しのブラインド入口で、
大半の呼び出し側が使うもの）は一度も渡さないので、素のブラインド復号では
pass 12 が発火しない — jt9 の `ipass=1..4` の失敗をそのまま再現し、
jt9 の `ipass=5` 相当に到達しないまま終わっていた。

これは `c38092f` で削除された `auto_ap_strategy` モジュール（0.8.0 の
`DecodeDepth` 再設計、issue #182 follow-up）とは別の話である。あちらは
同一スロットで復号済みの他のコールサインから AP の *iaptype-2*（`mycall`）を
自前で種付けするもので、その doc コメント自身が「WSJT-X の実能力を超える
mfsk-core 独自の拡張であって移植ではない」と明記していた。その削除
（`qso3_busy.wav` で測定、recall 変化ゼロ）は正しく、この発見の影響も
受けない。ここでの欠落は**無条件の** CQ 仮説（iaptype-1）であり、mycall への
依存が一切ない実在の WSJT-X の挙動である — コードは既にあって正しく、
ゲートが一層だけ保守的すぎた。

**修正**（`process_candidates.rs` Step 4）: pass 12（ブラインド CQ）は
`accepted.is_none()` **かつ** `sync_quality (nsync) ≥ BLIND_CQ_MIN_NSYNC (12)`
のときに `ap_hint` と無関係に走るようになった。ヒント依存のパス（5〜11）だけが
`ap_hint.is_some()` のゲートに残る。ホスト専用（`fft-rustfft`）なので組込みは
無影響。偽陽性のリスクは既存のパス毎 `validate` クロージャで抑えられたまま
である（ハードエラー上限、CRC、plausibility、そして `call1="CQ"` なので
復号テキストが文字列 "CQ" を literally 含むこと）。

**nsync≥12 の下限はコストゲートであり、ゲート無し版を測ってから足した。**
ゲート無しだと `decode_frame_subtract_staged`（`qso3_busy.wav` の golden
テストが通るマルチパス SIC ドライバ）が、同期品質に関係なく失敗した候補を
すべて pass 12 に送っていた — あのファイル1つで 188 候補、うち 140 が
nsync 7〜9 で、（あのファイルで実際に観測されたどの nsync 値でも）このパス
経由のデコードは1件も出なかった。実際の回収は終始 nsync 14〜18 を要して
いる。あのファイルの staged-SIC 経路の実時間は 0.7 s → 1.5 s になり
（**同じファイルでの本物の `jt9 -8 -d3` の ~1.15 s より遅い** — この修正の
前は mfsk-core が jt9 より ~40% 速かった）、このリリースで
`auto_ap_strategy` を削除する動機になったのと同じ形のコストの驚きを
繰り返した。nsync≥12 でゲートすると（実際の回収が要する 14〜18 より十分下、
雑音支配の 7〜9 のかたまりより十分上）、あのファイルの Step-4 候補数は
188→34 に落ち、実時間は ~0.85 s に戻る — 再び jt9 より速い — しかも
どちらのゲート設定でも **golden テストの recall 変化はゼロ**である。

**再計測**（同じ 780 ファイルのコーパス、§10 と同じ補間方法。ゲート版）:

| チャネル | 修正前 | 修正後（ゲート版） | 本物の `jt9 -8 -d3` | Δ（修正後 − jt9） |
|---|---:|---:|---:|---:|
| AWGN | ≈ -21.4 dB | ≈ -21.6 dB | ≈ -21.2 dB | mfsk-core が +0.4 dB 先行 |
| CCIR good | ≈ -20.8 dB | ≈ -21.1 dB | ≈ -20.75 dB | mfsk-core が +0.35 dB 先行 |
| CCIR moderate | ≈ -18.9 dB | ≈ -20.0 dB | ≈ -19.5 dB | mfsk-core が +0.5 dB 先行 |
| CCIR poor | ≈ -19.0 dB | ≈ -19.7 dB | ≈ -19.7 dB | ほぼ同等 |

CCIR moderate/poor の 0.6〜0.7 dB の差は解消した: moderate は jt9 より
明確に先行し、poor は劣後ではなく同等になった（ゲートはゲート無し版が
得るはずだった感度をいくらか返上している — CCIR poor はフェージングが
重い分、実際の候補がゲートの削る nsync 10〜13 帯により多く落ちる。
混雑バンドのマルチパス復号で jt9 より速いままでいるための、受け入れ可能な
取引と判断した。上のコストの議論を参照）。

セル別の生カウント、mfsk-core 修正後（ゲート版）、セルあたり 20 トライアル
（上の §10 の jt9 の表と対比）:

| SNR | CCIR good | CCIR moderate | CCIR poor |
|---:|---:|---:|---:|
| -15 dB | 19/20 | 20/20 | 20/20 |
| -17 dB | 19/20 | 16/20 | 20/20 |
| -18 dB | 19/20 | 17/20 | 16/20 |
| -19 dB | 19/20 | 15/20 | 12/20 |
| -20 dB | 16/20 | 10/20 | 9/20 |
| -21 dB | 11/20 | 5/20 | 4/20 |
| -22 dB | 2/20 | 0/20 | 0/20 |

**コスト、単一パス**（`ft8_qso3_full_parity_recall.rs`、`qso3_busy.wav`、
`DecodeDepth::FULL`、`max_cand=60`）: §9 の ~139〜141 ms → ~165〜175 ms
（+~20%。この小さめの単一パス走査では nsync ゲートを下回る候補が少ない）。
本物の `jt9 -8 -d3` のファイル全体 ~1.1〜1.2 s に対して依然 ~6〜7 倍速い。

**コスト、マルチパス**（`ft8_qso3_staged_sic_check.rs` の
`decode_frame_subtract_staged`、`max_cand=200`、実時間を単独計測）:
~700 ms → ~850 ms（+~20%）— 同一ファイルでの jt9 の ~1.15 s に対して
余裕を持って収まっており、上で測ったゲート無し版とは違う。

既存の golden テストに recall 退行なし（WSJT-X AP-off 7/8、JTDX 18/18、
full-parity 8/8、staged-SIC 18/18、AP-on JTDX-extras 6/6）。どちらのゲート
設定でもそうである — 新しいパスの `validate` クロージャは復号テキストが
"CQ" を含むことを要求するので、CQ 形式の候補を助けることしかできず、
指定局宛 QSO の golden エントリに偽陽性を持ち込むことはない。

issue [#190](https://github.com/jl1nie/mfsk-core/issues/190) クローズ。

## 12. 0.11.0 タグ前の tier-C 再計測 — §4 の数値は最大 1.8 dB 古い (2026-09-20)

**§4 の表は 2026-07-18 時点の測定であり、現在の値ではない。**
同じコーパス・同じ seed で 0.11.0 のタグ前に回した tier-C
（`scripts/run-sensitivity-sweeps.sh ft8`、チャネルあたり 180 トライアル）:

| チャネル | §4 (2026-07-18) | 2026-09-20 | 差 |
|---|---|---|---|
| AWGN | ≈ −20.4 dB | **−21.60 dB** | −1.2 dB |
| CCIR good | ≈ −20.0 dB | **−21.11 dB** | −1.1 dB |
| CCIR moderate | ≈ −18.3 dB | **−20.00 dB** | **−1.7 dB** |
| CCIR poor | ≈ −18.2 dB | **−19.67 dB** | −1.5 dB |

すべて**高感度側**への移動である。AWGN の −21.60 dB は WSJT-X の公称値
−20〜−21 dB を上回っている。

**どのコミットが効いたかは、この計測では特定していない。** 2つの日付の
あいだに coarse-sync のラグ窓が ±2.5 s へ広がり（真因は `PASS1_LIMIT`、
issue #280）、`fine_sync_12k` が入り、FT4/FST4 の a-priori 修正が入った。
最後のものは FT4 自身の AWGN 曲線を 1.1 dB 動かしたと記録されているので、
FT8 のフェージングチャネルで同程度動くのは不自然ではない。切り分けには
スイープ上での bisect が要り、それはリリース前の確認ではなく計測campaign
である。

機械照合される写しは `docs/notes/sweep-baseline.json` にあり、同じ run で
更新済み。`BENCHMARKS.md` の FT8 行も同じ4値を持っている。

この §12 は、直前の §11（2026-07-26、修正後 AWGN ≈ −21.6 dB）ではなく
§4 と比較している — 日本語版の読者が最初に目にする表が §4 だからである。
§9→§11 の中間の経緯はそれぞれの節を参照のこと。
