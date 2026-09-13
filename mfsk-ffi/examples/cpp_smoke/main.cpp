// SPDX-License-Identifier: GPL-3.0-or-later
//
// End-to-end C++ driver for the rs-ft8n FFI — encodes a known test
// message for every supported protocol, feeds the synthesised PCM
// back through the matching decoder handle, and verifies the decoded
// text round-trips correctly. Doubles as smoke test for the ABI
// (NULL handling, last-error, samples / message-list lifetimes) and
// as proof that each protocol is actually wired up in the C ABI.
//
// Build: run `./build.sh`.

#include "mfsk.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

namespace {

// Tally of failed sub-tests — reported at the end so one broken
// protocol doesn't hide the status of the others.
int g_failures = 0;

void fail(const char* proto, const char* detail) {
    std::fprintf(stderr, "  FAIL [%s] %s\n", proto, detail);
    g_failures++;
}

// Helper: does any decoded message text contain `needle` (case-sensitive)?
// `text` is a fixed inline buffer (issue #205), always NUL-terminated —
// no null check needed, unlike the old heap-`CString`-pointer shape.
bool any_contains(const MfskResultList& list, const char* needle) {
    for (size_t i = 0; i < list.len; ++i) {
        const MfskResult& m = list.items[i];
        if (std::strstr(m.text, needle) != nullptr) {
            return true;
        }
    }
    return false;
}

void print_decodes(const char* proto, const MfskResultList& list) {
    std::printf("  [%s] %zu decode(s):\n", proto, list.len);
    for (size_t i = 0; i < list.len; ++i) {
        const MfskResult& m = list.items[i];
        std::printf("    freq=%7.2f dt=%+.3f snr=%+.1f err=%u pass=%u text='%s'\n",
                    m.freq_hz, m.dt_sec, m.snr_db,
                    m.hard_errors, m.pass,
                    m.text);
    }
}

// ── v2 decode session ───────────────────────────────────────────────
//
// Written the way a consumer would: init params from the mode, open a
// session, decode into memory the caller owns. Nothing here frees a
// pointer the library allocated, which is the whole point — that
// category is what makes Kotlin and Swift wrappers leak when an
// exception unwinds past the free.
void test_session_decode() {
    std::printf("\n— v2 decode session: params → open → rows into caller memory\n");

    MfskSamples pcm{};
    if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("session", "mfsk_encode_ft8 failed");
        return;
    }
    std::vector<int16_t> audio(pcm.len);
    for (size_t i = 0; i < pcm.len; ++i) {
        audio[i] = static_cast<int16_t>(pcm.samples[i] * 32767.0f);
    }
    mfsk_samples_free(&pcm);

    MfskDecodeParams p;
    std::memset(&p, 0, sizeof p);
    p.size = sizeof p;
    if (mfsk_decode_params_init(MFSK_MODE_FT8, &p) != MFSK_STATUS_OK) {
        fail("session", "mfsk_decode_params_init failed");
        return;
    }
    std::printf("  FT8 defaults: band [%.0f, %.0f] sync_min %.2f max_cand %u\n",
                p.freq_min_hz, p.freq_max_hz, p.sync_min, p.max_cand);

    MfskStatus st = MFSK_STATUS_INTERNAL;
    MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, &p, &st);
    if (s == nullptr || st != MFSK_STATUS_OK) {
        fail("session", mfsk_last_error());
        return;
    }

    MfskDecode rows[8];
    std::memset(rows, 0, sizeof rows);
    for (auto& r : rows) r.size = sizeof r;
    size_t n = 0;
    if (mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000,
                                nullptr, rows, 8, &n) != MFSK_STATUS_OK) {
        fail("session", mfsk_session_last_error(s));
        mfsk_session_close(s);
        return;
    }
    std::printf("  %zu decode(s):\n", n);
    bool found = false;
    for (size_t i = 0; i < n; ++i) {
        std::printf("    mode=%d freq=%7.2f dt=%+.3f snr=%+.1f cv=%.3f "
                    "info=%u pass=%u text='%s'\n",
                    static_cast<int>(rows[i].mode), rows[i].freq_hz, rows[i].dt_sec,
                    rows[i].snr_db, rows[i].sync_cv, rows[i].info_bits,
                    rows[i].pass, rows[i].text);
        if (std::strstr(rows[i].text, "JA1ABC") != nullptr) found = true;
        if (rows[i].mode != MFSK_MODE_FT8) {
            fail("session", "row reports the wrong mode");
        }
        if (rows[i].info_bits != 91) {
            fail("session", "FT8 is LDPC(174,91); info_bits should be 91");
        }
    }
    if (!found) {
        fail("session", "did not decode the signal it was given");
    }

    // FEC bits come from the session, not from a pointer in the row.
    size_t need = 0;
    if (mfsk_session_copy_info(s, 0, nullptr, 0, &need) != MFSK_STATUS_INVALID_ARG ||
        need != 91) {
        fail("session", "copy_info should report the size it needs");
    } else {
        std::vector<uint8_t> bits(need);
        size_t got = 0;
        if (mfsk_session_copy_info(s, 0, bits.data(), bits.size(), &got) != MFSK_STATUS_OK ||
            got != need) {
            fail("session", "copy_info failed with a correctly sized buffer");
        }
    }

    // A short buffer reports the count needed rather than truncating.
    size_t needed = 0;
    MfskDecode one;
    std::memset(&one, 0, sizeof one);
    one.size = sizeof one;
    if (mfsk_session_decode_i16(s, audio.data(), audio.size(), 12000,
                                nullptr, &one, 0, &needed) != MFSK_STATUS_INVALID_ARG) {
        fail("session", "a zero-capacity buffer should report INVALID_ARG");
    } else if (needed != n) {
        fail("session", "*out_len should be the count needed");
    }

    mfsk_session_close(s);

    // Asking a mode for something it does not have fails at open, with
    // a message — not silently at decode, which is what the pre-v2
    // options handle did with six of its eleven fields.
    MfskDecodeParams bad;
    std::memset(&bad, 0, sizeof bad);
    bad.size = sizeof bad;
    mfsk_decode_params_init(MFSK_MODE_FT4, &bad);
    bad.sic_early = true;
    MfskStatus badst = MFSK_STATUS_OK;
    if (mfsk_session_open(MFSK_MODE_FT4, &bad, &badst) != nullptr ||
        badst != MFSK_STATUS_UNSUPPORTED) {
        fail("session", "sic_early on FT4 should be refused at open");
    } else {
        std::printf("  refused sic_early on FT4: %s\n", mfsk_last_error());
    }

    // A mode with no decode handle says so, naming the bit to check.
    MfskStatus wst = MFSK_STATUS_OK;
    if (mfsk_session_open(MFSK_MODE_WSPR, nullptr, &wst) != nullptr ||
        wst != MFSK_STATUS_UNSUPPORTED) {
        fail("session", "WSPR has no decode handle and should refuse");
    }

    // Every mode that claims the handle must open one.
    const uint32_t total = mfsk_mode_count();
    int opened = 0;
    for (uint32_t i = 0; i < total; ++i) {
        MfskMode m;
        if (mfsk_mode_at(i, &m) != MFSK_STATUS_OK) continue;
        if ((mfsk_mode_caps(m) & MFSK_CAP_DECODE_HANDLE) == 0) continue;
        MfskStatus ost = MFSK_STATUS_INTERNAL;
        MfskDecodeSession* sess = mfsk_session_open(m, nullptr, &ost);
        if (sess == nullptr || ost != MFSK_STATUS_OK) {
            fail(mfsk_mode_name(m), "claims MFSK_CAP_DECODE_HANDLE but will not open");
        } else {
            opened++;
            mfsk_session_close(sess);
        }
    }
    std::printf("  opened a session for all %d handle-driving mode(s)\n", opened);

    std::printf("  OK\n");
}

// ── Mode introspection (FFI v2 slice 1) ─────────────────────────────
//
// The point of this surface is that a C consumer stops hardcoding a
// capability matrix, so the test has to be written the way a consumer
// would: enumerate what the build has, ask each mode what it supports,
// and act on the answer. Anything asserted from a list written here
// would be testing this file, not the library.
void test_mode_introspection() {
    std::printf("\n— Mode introspection\n");

    const uint32_t abi = mfsk_abi_version();
    std::printf("  abi version: %u\n", abi);
    if (abi < 2) {
        fail("introspect", "mfsk_abi_version() predates the introspection surface");
        return;
    }

    const uint32_t n = mfsk_mode_count();
    if (n == 0) {
        fail("introspect", "this build claims to support no modes at all");
        return;
    }
    std::printf("  %u mode(s) in this build\n", n);

    // Walk every mode the way a UI populating a picker would.
    int with_handle = 0, fst4_submodes = 0, snipers = 0;
    uint32_t widest_fft = 0;
    char widest_name[16] = {0};

    for (uint32_t i = 0; i < n; ++i) {
        MfskMode m;
        if (mfsk_mode_at(i, &m) != MFSK_STATUS_OK) {
            fail("introspect", "mfsk_mode_at failed inside 0..count");
            return;
        }

        MfskModeInfo info;
        std::memset(&info, 0, sizeof info);
        info.size = sizeof info;
        if (mfsk_mode_info(m, &info) != MFSK_STATUS_OK) {
            fail("introspect", "mfsk_mode_info failed for an enumerated mode");
            return;
        }

        // The name must round-trip through the string form, which is
        // what a config file or a CLI flag will carry.
        const char* name = mfsk_mode_name(m);
        if (name == nullptr || std::strcmp(name, info.name) != 0) {
            fail("introspect", "mfsk_mode_name disagrees with MfskModeInfo::name");
            return;
        }
        MfskMode back;
        if (mfsk_mode_from_name(name, &back) != MFSK_STATUS_OK || back != m) {
            fail(name, "did not round-trip through mfsk_mode_from_name");
            return;
        }

        if (info.caps & MFSK_CAP_DECODE_HANDLE) {
            with_handle++;
            // Anything the decode handle drives must publish a usable
            // default search, or a caller has nothing to start from.
            MfskDecodeDefaults d;
            std::memset(&d, 0, sizeof d);
            d.size = sizeof d;
            if (mfsk_mode_defaults(m, &d) != MFSK_STATUS_OK) {
                fail(name, "drives the decode handle but publishes no defaults");
                return;
            }
            if (!(d.freq_max_hz > d.freq_min_hz) || d.max_cand == 0) {
                fail(name, "publishes an unusable default search");
                return;
            }
            // The trap this field exists for: FT4's threshold is on a
            // different scale from everyone else's.
            if (d.sync_scale == MFSK_SYNC_SCALE_BASELINE_NORMALISED && !(d.sync_min > 1.0f)) {
                fail(name, "baseline-normalised sync_min is at or below the noise floor");
                return;
            }
            if (info.decode_fft1_size == 0) {
                fail(name, "drives the decode handle but reports no slot transform size");
                return;
            }
            if (info.decode_fft1_size > widest_fft) {
                widest_fft = info.decode_fft1_size;
                std::snprintf(widest_name, sizeof widest_name, "%s", name);
            }
        }

        if (info.caps & MFSK_CAP_SNIPER) {
            snipers++;
            if (m != MFSK_MODE_FT8) {
                fail(name, "advertises a sniper, which is an FT8-only mode");
                return;
            }
        }
        if (std::strncmp(name, "FST4-", 5) == 0) {
            fst4_submodes++;
        }
    }

    // The equivalence this whole redesign is named for: the pre-v2 ABI
    // could address exactly one FST4 sub-mode (`Fst4s60 = 5`), so the
    // other four were unreachable from C for decode and encode alike.
    if (fst4_submodes != 5) {
        fail("introspect", "expected all five FST4 sub-modes to be addressable");
        return;
    }
    if (with_handle < 7) {
        fail("introspect", "FT8 + FT4 + five FST4 should all drive the decode handle");
        return;
    }
    if (snipers != 1) {
        fail("introspect", "exactly one mode should claim the sniper");
        return;
    }
    std::printf("  %d mode(s) drive the decode handle, %d FST4 sub-mode(s) addressable\n",
                with_handle, fst4_submodes);
    std::printf("  largest slot transform: %s at %u points\n", widest_name, widest_fft);

    // A mode this build lacks and a name that is not a mode must be
    // distinguishable — a typo is not the same problem as a missing
    // feature, and today a caller cannot tell.
    MfskMode dummy;
    if (mfsk_mode_from_name("FT9", &dummy) != MFSK_STATUS_INVALID_ARG) {
        fail("introspect", "a nonsense mode name should be INVALID_ARG");
    }
    if (mfsk_mode_at(n, &dummy) != MFSK_STATUS_INVALID_ARG) {
        fail("introspect", "one past the end should fail rather than wrap");
    }
    if (mfsk_mode_info(MFSK_MODE_FT8, nullptr) != MFSK_STATUS_INVALID_ARG) {
        fail("introspect", "a NULL out pointer should be rejected");
    }

    // Size versioning: an older caller declares a smaller struct and
    // must get only its prefix written. Emulated by declaring a size
    // that stops before the geometry fields.
    {
        unsigned char buf[sizeof(MfskModeInfo)];
        std::memset(buf, 0xAA, sizeof buf);
        const uint32_t shortSize = offsetof(MfskModeInfo, ntones);
        std::memcpy(buf, &shortSize, sizeof shortSize);
        if (mfsk_mode_info(MFSK_MODE_FT8, reinterpret_cast<MfskModeInfo*>(buf)) != MFSK_STATUS_OK) {
            fail("introspect", "size-versioned call with an older header failed");
        } else {
            uint32_t written = 0;
            std::memcpy(&written, buf, sizeof written);
            if (written != shortSize) {
                fail("introspect", "size was not rewritten to what was actually written");
            }
            for (size_t i = shortSize; i < sizeof buf; ++i) {
                if (buf[i] != 0xAA) {
                    fail("introspect", "wrote past the caller's declared struct size");
                    break;
                }
            }
        }
    }

    std::printf("  OK\n");
}

// ── FT8 ──────────────────────────────────────────────────────────────
void test_ft8() {
    std::printf("— FT8 roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("FT8", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("FT8", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("FT8", list);
        if (!any_contains(list, "JA1ABC") || !any_contains(list, "PM95")) {
            fail("FT8", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── FT8 streaming (mfsk_decode_i16_streaming, issue #246 follow-up) ───
//
// A real C callback invoked from actual C++-compiled code, through the
// generated header — the one thing the Rust-side `tests/streaming_ffi.rs`
// suite can't exercise (it calls the same crate's own functions
// directly, never crossing an actual compiler/ABI boundary the way a
// separate C++ translation unit does).
extern "C" void streaming_collect(const MfskResult* result, void* user_data) {
    auto* out = static_cast<std::vector<std::string>*>(user_data);
    out->emplace_back(result->text);
}

void test_ft8_streaming() {
    std::printf("— FT8 streaming: encode 'CQ JA1ABC PM95' at 1650 Hz → mfsk_decode_i16_streaming\n");
    MfskSamples pcm{};
    if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1650.0f, &pcm) != MFSK_STATUS_OK) {
        fail("FT8-streaming", mfsk_last_error());
        return;
    }
    std::vector<int16_t> i16_samples(pcm.len);
    for (size_t i = 0; i < pcm.len; ++i) {
        float s = pcm.samples[i] * 32767.0f;
        if (s > 32767.0f) s = 32767.0f;
        if (s < -32768.0f) s = -32768.0f;
        i16_samples[i] = static_cast<int16_t>(s);
    }

    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
    MfskResultList list{};
    std::vector<std::string> streamed;
    const MfskStatus st = mfsk_decode_i16_streaming(
        dec, i16_samples.data(), i16_samples.size(), 12000, nullptr,
        streaming_collect, &streamed, &list);
    if (st != MFSK_STATUS_OK) {
        fail("FT8-streaming", mfsk_last_error() ? mfsk_last_error() : "decode failed");
    } else {
        print_decodes("FT8-streaming (batch list)", list);
        std::printf("  streamed via callback: %zu\n", streamed.size());
        bool streamed_ok = false;
        for (const auto& s : streamed) {
            if (s.find("JA1ABC") != std::string::npos) streamed_ok = true;
        }
        if (!streamed_ok) {
            fail("FT8-streaming", "callback never delivered JA1ABC");
        }
        if (!any_contains(list, "JA1ABC") || !any_contains(list, "PM95")) {
            fail("FT8-streaming", "batch list missing expected callsign/grid");
        }
        if (streamed.size() != list.len) {
            fail("FT8-streaming", "streamed count != batch list count for one clean candidate");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── FFI builder-parity setters (issue #162 follow-up) ──────────────────
//
// MfskDecodeOptions hadn't grown a single setter since its creation
// (issue #205) despite its own doc comment anticipating exactly that —
// this proves every mfsk_decode_options_set_* setter actually reaches
// a real decode call from compiled C++, not just that they link.
void test_builder_options() {
    std::printf("— FFI builder-parity: mfsk_decode_options_set_* (strictness/eq_mode/freq_hint/sic_rounds/sic_early/ap_hint)\n");
    MfskSamples pcm{};
    if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("builder-options", mfsk_last_error());
        return;
    }

    MfskDecodeOptions* opts = mfsk_decode_options_new(
        200.0f, 3000.0f, 2.0f, 50, MFSK_DECODE_DEPTH_BP_ALL_OSD);
    if (opts == nullptr) {
        fail("builder-options", "mfsk_decode_options_new returned null");
        mfsk_samples_free(&pcm);
        return;
    }
    if (mfsk_decode_options_set_strictness(opts, MFSK_STRICTNESS_DEEP) != MFSK_STATUS_OK) {
        fail("builder-options", "set_strictness failed");
    }
    if (mfsk_decode_options_set_eq_mode(opts, MFSK_EQ_MODE_LOCAL) != MFSK_STATUS_OK) {
        fail("builder-options", "set_eq_mode failed");
    }
    if (mfsk_decode_options_set_freq_hint(opts, 1500.0f) != MFSK_STATUS_OK) {
        fail("builder-options", "set_freq_hint failed");
    }
    // sic_early overwrites whatever sic_rounds set, matching
    // DecodeRequest's own .sic_rounds(_).sic_early() overwrite
    // semantics — call both to prove the "last one wins" contract
    // doesn't error either way, not just that one setter works alone.
    if (mfsk_decode_options_set_sic_rounds(opts, 2) != MFSK_STATUS_OK) {
        fail("builder-options", "set_sic_rounds failed");
    }
    if (mfsk_decode_options_set_sic_early(opts) != MFSK_STATUS_OK) {
        fail("builder-options", "set_sic_early failed");
    }
    // Wide-band AP hint — string marshalling across the C boundary is
    // the most novel part of this setter family, worth its own
    // real-compiler proof. grid/report left NULL (call1/call2 only).
    if (mfsk_decode_options_set_ap_hint(opts, "JA1ABC", "CQ", nullptr, nullptr) != MFSK_STATUS_OK) {
        fail("builder-options", "set_ap_hint failed");
    }

    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, opts, &list);
    if (st != MFSK_STATUS_OK) {
        fail("builder-options", mfsk_last_error() ? mfsk_last_error() : "decode failed");
    } else {
        print_decodes("builder-options", list);
        if (!any_contains(list, "JA1ABC")) {
            fail("builder-options", "setters broke a clean decode");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_decode_options_free(opts);
    mfsk_samples_free(&pcm);
}

// ── Sniper mode (issue #249) ─────────────────────────────────────────
// The single-frequency entry point, driven the way a C caller actually
// would: aim at a frequency a sked/spot already named, on FT4, with an
// AP hint — which is no longer what this entry point is for.
// `mfsk_decode_options_set_ap_hint` now reaches the wide-band decoder
// for every protocol, and the sniper is FT8-only, so this checks both:
// FT4's sniper reports the mode unsupported, and the same hint decodes
// through the ordinary entry point.
void test_sniper() {
    std::printf("— FFI sniper: FT8 is the only mode that has one; FT4/WSPR must refuse\n");
    MfskSamples pcm{};
    if (mfsk_encode_ft4("CQ", "JA1ABC", "PM95", 1200.0f, &pcm) != MFSK_STATUS_OK) {
        fail("sniper", mfsk_last_error());
        return;
    }
    std::vector<int16_t> audio(pcm.len);
    for (size_t i = 0; i < pcm.len; ++i) {
        audio[i] = static_cast<int16_t>(pcm.samples[i] * 32767.0f);
    }
    mfsk_samples_free(&pcm);

    MfskDecodeOptions* opts = mfsk_decode_options_new(
        200.0f, 3000.0f, 1.2f, 8, MFSK_DECODE_DEPTH_BP_ALL_OSD);
    if (opts == nullptr) {
        fail("sniper", "mfsk_decode_options_new returned null");
        return;
    }
    if (mfsk_decode_options_set_ap_hint(opts, "JA1ABC", "CQ", nullptr, nullptr) != MFSK_STATUS_OK) {
        fail("sniper", "set_ap_hint failed");
    }

    // FT4's sniper is gone: narrow-band single-target search is the
    // receive half of an analogue roofing filter, a DX-chasing mode
    // that a contest protocol has no use for. It must say so rather
    // than decode something.
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT4);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_i16_sniper(
        dec, audio.data(), audio.size(), 12000, 1200.0f, opts, &list);
    if (st != MFSK_STATUS_UNKNOWN_PROTOCOL) {
        fail("sniper", "FT4 sniper should return MFSK_STATUS_UNKNOWN_PROTOCOL");
    }
    mfsk_result_list_free(&list);

    // The AP hint this entry point existed to reach now works on the
    // ordinary wide-band decode, for every protocol.
    MfskResultList wide{};
    const MfskStatus wst = mfsk_decode_i16(
        dec, audio.data(), audio.size(), 12000, opts, &wide);
    if (wst != MFSK_STATUS_OK) {
        fail("sniper", mfsk_last_error() ? mfsk_last_error() : "wide-band AP decode failed");
    } else {
        print_decodes("FT4 wide-band + AP hint", wide);
        if (!any_contains(wide, "JA1ABC")) {
            fail("sniper", "wide-band decode with an AP hint did not find the signal");
        }
    }
    mfsk_result_list_free(&wide);

    // And the mode that *does* have a sniper must still work through
    // it. Nothing else covers this entry point from C now that the
    // FT4 arm is a refusal, so without this the only compiled-C
    // exercise of the sniper would be two error paths.
    {
        MfskSamples ft8pcm{};
        if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f, &ft8pcm) != MFSK_STATUS_OK) {
            fail("sniper", "mfsk_encode_ft8 failed");
        } else {
            std::vector<int16_t> ft8audio(ft8pcm.len);
            for (size_t i = 0; i < ft8pcm.len; ++i) {
                ft8audio[i] = static_cast<int16_t>(ft8pcm.samples[i] * 32767.0f);
            }
            mfsk_samples_free(&ft8pcm);

            MfskDecoder* ft8dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
            MfskResultList hit{};
            const MfskStatus fst = mfsk_decode_i16_sniper(
                ft8dec, ft8audio.data(), ft8audio.size(), 12000, 1500.0f, nullptr, &hit);
            if (fst != MFSK_STATUS_OK) {
                fail("sniper", mfsk_last_error() ? mfsk_last_error() : "FT8 sniper failed");
            } else {
                print_decodes("FT8 sniper", hit);
                if (!any_contains(hit, "JA1ABC")) {
                    fail("sniper", "FT8 sniper did not find the signal it was aimed at");
                }
            }
            mfsk_result_list_free(&hit);
            mfsk_decoder_free(ft8dec);
        }
    }

    // A protocol with no single-frequency mode must say so rather than
    // decode something else.
    MfskDecoder* wspr = mfsk_decoder_new(MFSK_PROTOCOL_WSPR);
    MfskResultList unused{};
    const MfskStatus bad = mfsk_decode_i16_sniper(
        wspr, audio.data(), audio.size(), 12000, 1200.0f, nullptr, &unused);
    if (bad != MFSK_STATUS_UNKNOWN_PROTOCOL) {
        fail("sniper", "WSPR sniper should return MFSK_STATUS_UNKNOWN_PROTOCOL");
    }
    mfsk_decoder_free(wspr);

    mfsk_decoder_free(dec);
    mfsk_decode_options_free(opts);
}

// ── FT4 ──────────────────────────────────────────────────────────────
void test_ft4() {
    std::printf("— FT4 roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_ft4("CQ", "JA1ABC", "PM95", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("FT4", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT4);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("FT4", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("FT4", list);
        if (!any_contains(list, "JA1ABC") || !any_contains(list, "PM95")) {
            fail("FT4", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── WSPR ─────────────────────────────────────────────────────────────
void test_wspr() {
    std::printf("— WSPR roundtrip: encode 'K1ABC FN42 37' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_wspr("K1ABC", "FN42", 37, 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("WSPR", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_WSPR);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("WSPR", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("WSPR", list);
        if (!any_contains(list, "K1ABC") || !any_contains(list, "FN42")) {
            fail("WSPR", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── JT9 ──────────────────────────────────────────────────────────────
void test_jt9() {
    std::printf("— JT9 roundtrip: encode 'CQ K1ABC FN42' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_jt9("CQ", "K1ABC", "FN42", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("JT9", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_JT9);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("JT9", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("JT9", list);
        if (!any_contains(list, "K1ABC") || !any_contains(list, "FN42")) {
            fail("JT9", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── FST4-60A ─────────────────────────────────────────────────────────
// Gated behind the RUN_FST4_ROUNDTRIP environment variable because the
// 60-s slot + outer 786 432-pt FFT makes this multi-second and not
// every developer wants to wait on it every build.
void test_fst4() {
    if (!std::getenv("RUN_FST4_ROUNDTRIP")) {
        std::printf("— FST4-60A roundtrip: skipped (set RUN_FST4_ROUNDTRIP=1)\n");
        return;
    }
    std::printf("— FST4-60A roundtrip: encode 'CQ JA1ABC PM95' at 1500 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_fst4s60("CQ", "JA1ABC", "PM95", 1500.0f, &pcm) != MFSK_STATUS_OK) {
        fail("FST4", mfsk_last_error());
        return;
    }
    // Pad up to a full 60-s slot with 1 s of leading silence so the
    // outer FFT has the window decode_frame expects.
    constexpr size_t kSlot = 60 * 12000;
    std::vector<float> slot(kSlot, 0.0f);
    const size_t offset = 12000;
    const size_t copy_len = std::min(pcm.len, kSlot - offset);
    std::memcpy(slot.data() + offset, pcm.samples, copy_len * sizeof(float));

    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FST4S60);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, slot.data(), slot.size(), 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("FST4", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("FST4", list);
        if (!any_contains(list, "JA1ABC") || !any_contains(list, "PM95")) {
            fail("FST4", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── JT65 ─────────────────────────────────────────────────────────────
void test_jt65() {
    std::printf("— JT65 roundtrip: encode 'CQ K1ABC FN42' at 1270 Hz → decode\n");
    MfskSamples pcm{};
    if (mfsk_encode_jt65("CQ", "K1ABC", "FN42", 1270.0f, &pcm) != MFSK_STATUS_OK) {
        fail("JT65", mfsk_last_error());
        return;
    }
    MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_JT65);
    MfskResultList list{};
    const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
    if (st != MFSK_STATUS_OK) {
        fail("JT65", mfsk_last_error() ? mfsk_last_error() : "decode_f32 failed");
    } else {
        print_decodes("JT65", list);
        if (!any_contains(list, "K1ABC") || !any_contains(list, "FN42")) {
            fail("JT65", "expected callsign / grid not recovered");
        }
    }
    mfsk_result_list_free(&list);
    mfsk_decoder_free(dec);
    mfsk_samples_free(&pcm);
}

// ── Multi-thread stress ─────────────────────────────────────────────
//
// The C API documents `MfskDecoder` as "not Sync — one handle per
// thread". These tests back that up with a real multi-threaded
// driver to catch any accidental shared mutable state in the Rust
// backends (thread_local slots, global FFT planners, etc.) that
// would break under concurrent use.
void test_threads_one_handle_per_thread() {
    std::printf("— threads × 1 handle each: 8 parallel FT8 decodes\n");
    constexpr int kThreads = 8;
    std::atomic<int> ok_count{0};
    std::atomic<int> fail_count{0};
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([&ok_count, &fail_count, t]() {
            MfskSamples pcm{};
            if (mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f + t * 20.0f, &pcm) != MFSK_STATUS_OK) {
                fail_count++;
                return;
            }
            MfskDecoder* dec = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
            MfskResultList list{};
            const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
            bool ok = (st == MFSK_STATUS_OK) && any_contains(list, "JA1ABC");
            if (ok) ok_count++; else fail_count++;
            mfsk_result_list_free(&list);
            mfsk_decoder_free(dec);
            mfsk_samples_free(&pcm);
        });
    }
    for (auto& th : ts) th.join();
    std::printf("  → %d/%d OK, %d fail\n",
                ok_count.load(), kThreads, fail_count.load());
    if (ok_count.load() != kThreads) {
        fail("threads", "one-handle-per-thread concurrent decode failed");
    }
}

// Even stronger: one handle shared across threads. Spec says "not
// Sync"; this test verifies the *current* implementation in fact
// stays sound under sharing (no internal mutable state on DecoderInner).
// If this ever starts failing, tighten the spec AND fix the cause.
void test_threads_shared_handle() {
    std::printf("— threads × 1 shared handle: 8 parallel FT8 decodes\n");
    constexpr int kThreads = 8;
    std::atomic<int> ok_count{0};
    std::atomic<int> fail_count{0};
    MfskDecoder* shared = mfsk_decoder_new(MFSK_PROTOCOL_FT8);
    std::vector<std::thread> ts;
    for (int t = 0; t < kThreads; ++t) {
        ts.emplace_back([shared, &ok_count, &fail_count, t]() {
            MfskSamples pcm{};
            if (mfsk_encode_ft8("CQ", "K1ABC", "FN42", 1500.0f + t * 20.0f, &pcm) != MFSK_STATUS_OK) {
                fail_count++;
                return;
            }
            MfskResultList list{};
            const MfskStatus st = mfsk_decode_f32(shared, pcm.samples, pcm.len, 12000, nullptr, &list);
            bool ok = (st == MFSK_STATUS_OK) && any_contains(list, "K1ABC");
            if (ok) ok_count++; else fail_count++;
            mfsk_result_list_free(&list);
            mfsk_samples_free(&pcm);
        });
    }
    for (auto& th : ts) th.join();
    mfsk_decoder_free(shared);
    std::printf("  → %d/%d OK, %d fail\n",
                ok_count.load(), kThreads, fail_count.load());
    if (ok_count.load() != kThreads) {
        fail("threads-shared", "shared-handle concurrent decode failed");
    }
}

// Mixed-protocol threads: ensures per-thread thread_local state
// (like mfsk_last_error) in the Rust side doesn't cross-contaminate.
void test_threads_mixed_protocols() {
    std::printf("— threads × mixed protocols (FT8 + FT4 + WSPR concurrently)\n");
    std::atomic<int> ok_count{0};
    std::atomic<int> fail_count{0};
    auto run_proto = [&](MfskProtocol proto, auto encode_fn, const char* needle) {
        MfskSamples pcm{};
        if (encode_fn(&pcm) != MFSK_STATUS_OK) { fail_count++; return; }
        MfskDecoder* dec = mfsk_decoder_new(proto);
        MfskResultList list{};
        const MfskStatus st = mfsk_decode_f32(dec, pcm.samples, pcm.len, 12000, nullptr, &list);
        if (st == MFSK_STATUS_OK && any_contains(list, needle)) ok_count++;
        else fail_count++;
        mfsk_result_list_free(&list);
        mfsk_decoder_free(dec);
        mfsk_samples_free(&pcm);
    };
    std::thread t_ft8([&] {
        run_proto(MFSK_PROTOCOL_FT8,
                  [](MfskSamples* p) { return mfsk_encode_ft8("CQ", "JA1ABC", "PM95", 1500.0f, p); },
                  "JA1ABC");
    });
    std::thread t_ft4([&] {
        run_proto(MFSK_PROTOCOL_FT4,
                  [](MfskSamples* p) { return mfsk_encode_ft4("CQ", "W1AW", "FN31", 1500.0f, p); },
                  "W1AW");
    });
    std::thread t_wspr([&] {
        run_proto(MFSK_PROTOCOL_WSPR,
                  [](MfskSamples* p) { return mfsk_encode_wspr("K1ABC", "FN42", 37, 1500.0f, p); },
                  "K1ABC");
    });
    t_ft8.join();
    t_ft4.join();
    t_wspr.join();
    std::printf("  → %d/3 OK, %d fail\n", ok_count.load(), fail_count.load());
    if (ok_count.load() != 3) {
        fail("threads-mixed", "mixed-protocol concurrent decode failed");
    }
}

// ── Negative paths ──────────────────────────────────────────────────
void test_null_handling() {
    std::printf("— NULL / invalid-arg handling\n");
    // NULL decoder
    MfskResultList list{};
    MfskStatus st = mfsk_decode_f32(nullptr, nullptr, 0, 12000, nullptr, &list);
    if (st != MFSK_STATUS_INVALID_ARG) {
        fail("null", "expected INVALID_ARG for null decoder");
    }
    // Free NULL pointers — must not crash.
    mfsk_decoder_free(nullptr);
    mfsk_result_list_free(nullptr);
    mfsk_samples_free(nullptr);
    // Unknown callsign at encode time → InvalidArg + meaningful error.
    MfskSamples bogus{};
    st = mfsk_encode_ft8("XXX", "Y2Z", "FN42", 1500.0f, &bogus);
    if (st == MFSK_STATUS_OK) {
        fail("null", "expected pack77 failure for bogus callsigns");
        mfsk_samples_free(&bogus);
    }
}

} // namespace

int main() {
    const uint32_t ver = mfsk_version();
    std::printf("mfsk-ffi version: %u.%u.%u\n",
                (ver >> 16) & 0xff,
                (ver >> 8) & 0xff,
                ver & 0xff);

    test_mode_introspection();
    test_session_decode();
    test_ft8();
    test_ft8_streaming();
    test_builder_options();
    test_sniper();
    test_ft4();
    test_fst4();
    test_wspr();
    test_jt9();
    test_jt65();
    test_threads_one_handle_per_thread();
    test_threads_shared_handle();
    test_threads_mixed_protocols();
    test_null_handling();

    if (g_failures == 0) {
        std::printf("\nALL OK\n");
        return 0;
    }
    std::fprintf(stderr, "\n%d FAILURE(S)\n", g_failures);
    return 1;
}
