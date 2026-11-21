#!/usr/bin/env bash
# run.sh - builds the real Sprayer_ESP32 and Seeder_ESP32 programs for the PC (one library per board) and the
# harness, then runs every scenario. Arguments go to the harness (--only <text>, --board <key>, --list).
# Part of the module test harness (folder Tests/ESP32_Modules of the Boards repository).
# Needs g++ with 32-bit support (Ubuntu: apt install g++-multilib): the ESP32 is a 32-bit chip, so int and long
# are 32 bits here too. SANITIZE=0 builds without the address / undefined-behaviour checks (faster, less found).
set -euo pipefail
cd "$(dirname "$0")"
HERE=$(pwd)
REPO=$(cd ../.. && pwd)
BUILD=${BUILD:-build}
mkdir -p "$BUILD"

FLAGS=(-m32 -std=gnu++17 -funsigned-char -g -O1 -Wall -Wextra -Wno-unused-parameter)
if [ "${SANITIZE:-1}" = "1" ]; then FLAGS+=(-fsanitize=address,undefined -fno-omit-frame-pointer); fi

# One C++ file per program, as the Arduino IDE builds a sketch: the main .ino first, then the others by name
sketch() {
    local dir=$1 main=$2 out=$3
    {
        echo '#include <Arduino.h>'
        echo "#line 1 \"$dir/$main.ino\""
        cat "$dir/$main.ino"
        for f in $(ls "$dir"/*.ino | sort); do
            [ "$(basename "$f")" = "$main.ino" ] && continue
            echo "#line 1 \"$f\""
            cat "$f"
        done
    } > "$out"
}
sketch "$REPO/Sprayer/Sprayer_ESP32" Sprayer_ESP32 "$BUILD/sprayer.cpp"
sketch "$REPO/Seeder/Seeder_ESP32" Seeder_ESP32 "$BUILD/seeder.cpp"

# the firmware, one shared library per board (BOARD 1 / 2 as in each program)
lib() {
    local src=$1 inc=$2 board=$3 out=$4
    g++ "${FLAGS[@]}" -fPIC -shared -fno-gnu-unique -I"$HERE/fake" -I"$inc" -DBOARD="$board" "$src" -o "$out"
}
pids=()
lib "$BUILD/sprayer.cpp" "$REPO/Sprayer/Sprayer_ESP32" 1 "$BUILD/sprayer_1.so" & pids+=($!)
lib "$BUILD/sprayer.cpp" "$REPO/Sprayer/Sprayer_ESP32" 2 "$BUILD/sprayer_2.so" & pids+=($!)
lib "$BUILD/seeder.cpp" "$REPO/Seeder/Seeder_ESP32" 1 "$BUILD/seeder_1.so" & pids+=($!)
lib "$BUILD/seeder.cpp" "$REPO/Seeder/Seeder_ESP32" 2 "$BUILD/seeder_2.so" & pids+=($!)
g++ "${FLAGS[@]}" -rdynamic -I"$HERE/fake" harness/*.cpp fake/fake_core.cpp -ldl -o "$BUILD/harness" & pids+=($!)
failed=0
for p in "${pids[@]}"; do wait "$p" || failed=1; done
if [ "$failed" = 1 ]; then echo "build failed" >&2; exit 2; fi

export MODULE_LIB_DIR="$BUILD"
export ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0}
export UBSAN_OPTIONS=${UBSAN_OPTIONS:-halt_on_error=1:print_stacktrace=1}
export BOARDS_REPO="$REPO"
export HARNESS_COMMIT=${HARNESS_COMMIT:-$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || true)}
exec "$BUILD/harness" --known known-bugs.txt --report "$BUILD/report.md" "$@"
