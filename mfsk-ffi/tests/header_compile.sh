#!/usr/bin/env bash
# Compile each generated header as the *only* thing in a translation unit,
# once as C11 and once as C++17, with warnings fatal.
#
# Why this exists: both cbindgen.toml files set `cpp_compat = true` and
# nothing verified it. The C++ smoke driver includes `mfsk.h` after its own
# headers, so a header that only compiles because something else was
# included first would pass CI. Any hand-written text in a cbindgen
# `[export] header` block — export macros, `#define`s — is not type-checked
# by cbindgen at all, so this is the only thing that would catch a typo in
# it.
#
# The two headers are compiled separately on purpose: they are documented
# as not co-includable in one translation unit (`mfsk-ffi-abi/src/lib.rs`
# — C, unlike C++, forbids two textually-identical struct definitions).
# Retiring that restriction is a later change; this script pins the current
# contract rather than assuming the future one.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

status=0
for header in \
    "$REPO_ROOT/mfsk-ffi/include/mfsk.h" \
    "$REPO_ROOT/mfsk-ffi-ft8/include/mfsk_ft8.h"
do
    [ -f "$header" ] || { echo "missing header: $header"; status=1; continue; }
    name="$(basename "$header")"

    printf '#include "%s"\n' "$header" > "$TMP/tu.c"
    printf '#include "%s"\nint main(void) { return 0; }\n' "$header" > "$TMP/tu.cpp"

    if cc -std=c11 -Wall -Wextra -Werror -pedantic -c "$TMP/tu.c" -o "$TMP/tu.o"; then
        echo "ok   $name  (C11)"
    else
        echo "FAIL $name  (C11)"
        status=1
    fi

    if c++ -std=c++17 -Wall -Wextra -Werror -c "$TMP/tu.cpp" -o "$TMP/tu_cpp.o"; then
        echo "ok   $name  (C++17)"
    else
        echo "FAIL $name  (C++17)"
        status=1
    fi
done

exit "$status"
