# Platform differences between the two machines these scripts run on:
# WSL/Linux (usbipd passthrough, GNU coreutils) and macOS (native USB,
# BSD userland). Sourced by flash-monitor.sh and capture.sh so the
# divergences live in one place rather than drifting between them.
#
# Nothing here is ESP-specific — it is entirely "GNU said X, BSD says
# Y", plus where the board's serial node shows up.

# The board's serial node, by platform.
#
# WSL/Linux: usbipd hands the device to the WSL kernel, which enumerates
# the S3's USB-Serial-JTAG as CDC-ACM → /dev/ttyACM0.
#
# macOS: the same CDC-ACM interface appears as /dev/cu.usbmodem<serial>
# with no driver and no passthrough layer — the CoreS3 wires USB-C
# straight to the ESP32-S3's native USB (unlike the Core2, which has a
# CH9102 bridge). The serial suffix differs per board, so glob for it
# and take the first. `cu.*` not `tty.*`: opening `tty.*` blocks on DCD,
# which is not asserted here.
mfsk_default_port() {
    case "$(uname -s)" in
        Darwin)
            local p
            for p in /dev/cu.usbmodem*; do
                [ -e "$p" ] && { printf '%s\n' "$p"; return 0; }
            done
            # Nothing plugged in. Emit the shape anyway so the caller's
            # "port missing" message names something recognisable.
            printf '/dev/cu.usbmodem*\n'
            ;;
        *) printf '/dev/ttyACM0\n' ;;
    esac
}

# True when the usbipd attach dance applies at all. macOS and native
# Linux both address the board directly; only WSL has a device that is
# enumerated on Windows and must be handed across.
mfsk_is_wsl() {
    [ -n "${WSL_DISTRO_NAME:-}" ] || grep -qi microsoft /proc/version 2>/dev/null
}

# Run a command under a pty, capturing the transcript, and stop it after
# DURATION seconds.
#
#   mfsk_run_pty <duration_s> <logfile> <cmd> [args...]
#
# Two BSD/GNU splits, both load-bearing:
#
# 1. `script`. GNU takes the command as one string after -c
#    (`script -qfc "CMD" FILE`); BSD takes the file first and the
#    command as ordinary argv (`script -q -F FILE CMD args...`). The
#    BSD form is the better one — no quoting of the inner command — but
#    GNU's `script` does not accept it.
# 2. `timeout`. Not in the BSD userland at all, and coreutils is not
#    assumed to be installed, so fall back to a bash watchdog. The
#    watchdog TERMs the `script` process, which is what `timeout
#    --foreground` does: the signal has to reach the pty owner, not
#    this shell, or espflash is left holding the port.
#
# stdin from /dev/null so the monitor never blocks on input.
mfsk_run_pty() {
    local duration=$1 log=$2; shift 2

    local timeout_bin=""
    if command -v timeout >/dev/null 2>&1; then
        timeout_bin=timeout
    elif command -v gtimeout >/dev/null 2>&1; then
        timeout_bin=gtimeout
    fi

    if [ "$(uname -s)" = Darwin ]; then
        if [ -n "$timeout_bin" ]; then
            "$timeout_bin" --foreground "$duration" \
                script -q -F "$log" "$@" </dev/null || true
        else
            script -q -F "$log" "$@" </dev/null &
            mfsk__watchdog $! "$duration"
        fi
    else
        # GNU: the inner command has to be re-quoted into one string.
        local quoted
        printf -v quoted '%q ' "$@"
        if [ -n "$timeout_bin" ]; then
            "$timeout_bin" --foreground "$duration" \
                script -qfc "$quoted" "$log" </dev/null || true
        else
            script -qfc "$quoted" "$log" </dev/null &
            mfsk__watchdog $! "$duration"
        fi
    fi
}

# TERM the given pid after N seconds unless it exits first; then reap
# the sleeper so the shell does not sit on it. KILL after a 5 s grace
# period, because a monitor that ignores TERM would otherwise hang the
# capture it was meant to bound.
mfsk__watchdog() {
    local pid=$1 duration=$2 sleeper
    ( sleep "$duration"; kill -TERM "$pid" 2>/dev/null
      sleep 5;           kill -KILL "$pid" 2>/dev/null ) &
    sleeper=$!
    wait "$pid" 2>/dev/null || true
    kill -TERM "$sleeper" 2>/dev/null || true
    wait "$sleeper" 2>/dev/null || true
}

# Tidy a pty transcript: strip the CR the pty injects, and the `^D\b\b`
# BSD `script` writes when its stdin is already at EOF (which ours is —
# `</dev/null`, so the monitor never blocks on input). GNU `script -c`
# emits neither; both are cosmetic, but the `^D` lands on the log's
# first line, which is where a reader looks for the espflash banner.
#
# Not `sed -i`: GNU takes the backup suffix as optional, BSD requires it
# as a separate argument, so `sed -i 's/\r$//' f` on BSD consumes the
# script as the suffix and fails. Write-and-move works on both and needs
# no branch.
mfsk_strip_cr() {
    local f=$1 tmp
    tmp=$(mktemp "${TMPDIR:-/tmp}/mfsk-log.XXXXXX") || return 1
    LC_ALL=C sed -e 's/\r*$//' -e "s/\^D$(printf '\b\b')//g" "$f" > "$tmp" \
        && mv "$tmp" "$f"
}
