#!/usr/bin/env bash
# run.sh - builds the real XIAO program (v4.5_XIAO_ESP32) for the PC, once for each XIAO (ESP32C3, ESP32S3), and the
# harness, then runs every scenario. Arguments go to the harness (--only <text>, --kit <key>, --list).
# Part of the XIAO test harness (folder Tests/XIAO_ESP32 of the Boards repository).
# Needs g++ with 32-bit support (Ubuntu: apt install g++-multilib): the ESP32 is a 32-bit chip, so int and long
# are 32 bits here too. SANITIZE=0 builds without the address / undefined-behaviour checks (faster, less found).
set -euo pipefail
cd "$(dirname "$0")"
HERE=$(pwd)
REPO=$(cd ../.. && pwd)
SKETCH="$REPO/Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32"
BUILD=${BUILD:-build}
mkdir -p "$BUILD"

FLAGS=(-m32 -std=gnu++17 -funsigned-char -g -O1 -Wall -Wextra -Wno-unused-parameter)
if [ "${SANITIZE:-1}" = "1" ]; then FLAGS+=(-fsanitize=address,undefined -fno-omit-frame-pointer); fi

# One C++ file, as the Arduino IDE builds a sketch (this one has a single .ino; its .h files are included by it)
{
    echo '#include <Arduino.h>'
    echo "#line 1 \"$SKETCH/v4.5_XIAO_ESP32.ino\""
    cat "$SKETCH/v4.5_XIAO_ESP32.ino"
} > "$BUILD/xiao.cpp"

# the program, one shared library per XIAO: loaded again at every restart, so its variables start fresh
lib() {
    local out=$1; shift
    g++ "${FLAGS[@]}" -fPIC -shared -fno-gnu-unique -I"$HERE/fake" -I"$SKETCH" "$@" "$BUILD/xiao.cpp" -o "$out"
}
pids=()
lib "$BUILD/xiao_c3.so" & pids+=($!)
lib "$BUILD/xiao_s3.so" -DXIAO_KIT_S3 & pids+=($!)
g++ "${FLAGS[@]}" -rdynamic -I"$HERE/fake" harness/*.cpp fake/fake_core.cpp -ldl -o "$BUILD/harness" & pids+=($!)
failed=0
for p in "${pids[@]}"; do wait "$p" || failed=1; done
if [ "$failed" = 1 ]; then echo "build failed" >&2; exit 2; fi

export XIAO_LIB_DIR="$BUILD"
export ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0}
export UBSAN_OPTIONS=${UBSAN_OPTIONS:-halt_on_error=1:print_stacktrace=1}
export BOARDS_REPO="$REPO"
export HARNESS_COMMIT=${HARNESS_COMMIT:-$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || true)}
exec "$BUILD/harness" --known known-bugs.txt --report "$BUILD/report.md" "$@"
