#!/bin/sh
# Build the firmware and require behavioural equivalence to the vendor image (the ground truth) via
# the differential emulator. $1 overrides the optimisation level (default: the CMake default, -O2).
set -e
T=$(cd "$(dirname "$0")" && pwd)
[ -f "$T/build/build.ninja" ] || cmake -S "$T" -B "$T/build" -G Ninja >/dev/null
[ -n "$1" ] && cmake -S "$T" -B "$T/build" -G Ninja -DFX3_OPT=-$1 >/dev/null
cmake --build "$T/build" >/dev/null 2>&1
FX="$T/../.re/.venv/bin/python"; [ -x "$FX" ] || FX=python3
cd "$T/../tools/fx3emu"
"$FX" -m fx3emu diff --fuzz "${FUZZ:-16}" 2>&1 | tail -1
