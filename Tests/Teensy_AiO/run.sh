#!/usr/bin/env bash
# run.sh - builds the real Teensy firmware (Firmware/v4.5_AiO_WEB) for the PC and the harness, then runs every
# scenario. Arguments go to the harness (--only <text>, --kit <key>, --jobs <n>, --list).
# Part of the Teensy test harness (folder Tests/Teensy_AiO of the Boards repository).
# Written 2026 by Claude (Anthropic) for renatorgr. The history of changes is in the README changelog and in the git log.
# Needs g++ with 32-bit support (Ubuntu: apt install g++-multilib) and python3: the Teensy is a 32-bit chip, so int
# and long are 32 bits here too. SANITIZE=0 builds without the address / undefined-behaviour checks (faster, less found).
# Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail
cd "$(dirname "$0")"
HERE=$(pwd)
REPO=$(cd ../.. && pwd)
SKETCH=${SKETCH:-$REPO/Firmware/v4.5_AiO_WEB}
BUILD=${BUILD:-build}
mkdir -p "$BUILD/src"

FLAGS=(-m32 -std=gnu++17 -funsigned-char -g -O1 -Wall -Wextra -Wno-unused-parameter)
# (shift-base: a negative number shifted left, as in "(int8_t)b << 8", is defined by GCC, the Teensy's compiler too)
if [ "${SANITIZE:-1}" = "1" ]; then FLAGS+=(-fsanitize=address,undefined -fno-sanitize=shift-base -fno-omit-frame-pointer); fi

# The sketch as the Arduino IDE builds it: one C++ file with prototypes, plus its .cpp files (tools/prep.py)
python3 tools/prep.py "$SKETCH" "$BUILD/src"
NAME=$(basename "$SKETCH")

# The running firmware: the repository's .hex, put in the pretend flash at power-on (harness/bench.cpp loadHex)
HEX="$SKETCH/$NAME.ino.hex"
[ -f "$HEX" ] || HEX="$SKETCH/$NAME.hex"

# the firmware as one shared library (loaded again at every restart of the pretend Teensy)
FW=(-fPIC -shared -fno-gnu-unique -Wno-unused-function -I"$HERE/fake" -I"$BUILD/src" "-DOTA_IMAGE_LEN=(fake_teensy_image_len())")
pids=()
g++ "${FLAGS[@]}" "${FW[@]}" "$BUILD/src/$NAME.cpp" "$BUILD/src/BNO08x_AOG.cpp" "$BUILD/src/zADS1115.cpp" -o "$BUILD/aio.so" & pids+=($!)
g++ "${FLAGS[@]}" -rdynamic -I"$HERE/fake" harness/*.cpp fake/fake_core.cpp -ldl -o "$BUILD/harness" & pids+=($!)
failed=0
for p in "${pids[@]}"; do wait "$p" || failed=1; done
if [ "$failed" = 1 ]; then echo "build failed" >&2; exit 2; fi

export TEENSY_LIB_DIR="$BUILD"
export ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0}
export UBSAN_OPTIONS=${UBSAN_OPTIONS:-halt_on_error=1:print_stacktrace=1}
export BOARDS_REPO="$REPO"
export TEENSY_HEX="$HEX"
export HARNESS_COMMIT=${HARNESS_COMMIT:-$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || true)}
exec "$BUILD/harness" --known known-bugs.txt --report "$BUILD/report.md" "$@"
