#!/bin/sh
# Build the firmware and require behavioural equivalence to the vendor image (the ground truth) via
# the differential emulator. $1 overrides the optimisation level (default: the CMake default, -O0).
# Builds in the top-level build directory (BUILD_DIR, default build/), turning EINSTAR_BUILD_FIRMWARE on there.
set -e
T=$(cd "$(dirname "$0")" && pwd)
B=${BUILD_DIR:-$T/../build}
cmake -S "$T/.." -B "$B" -DEINSTAR_BUILD_FIRMWARE=ON ${1:+-DFX3_OPT=-$1} >/dev/null
cmake --build "$B" --target firmware >/dev/null 2>&1
export FX3_MODERN_ELF="$(cd "$B" && pwd)/firmware/EN/einstar_fx3.elf"
FX="$T/../.re/.venv/bin/python"; [ -x "$FX" ] || FX=python3
cd "$T/../tools/fx3emu"
"$FX" -m fx3emu diff --fuzz "${FUZZ:-16}" 2>&1 | tail -1
