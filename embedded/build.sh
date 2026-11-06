#!/usr/bin/env bash
# Cross-compile the whole-body controller for the STM32H743 (NUCLEO-H743ZI2).
#   embedded/build.sh [build-dir]       (default: embedded/build)
# Needs the Arm GNU toolchain (arm-none-eabi). Set ARM_TOOLCHAIN_DIR if it is
# not in ~/.local/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${1:-$HERE/build}"
cmake -S "$HERE" -B "$BUILD" -DCMAKE_TOOLCHAIN_FILE="$HERE/cmake/arm-none-eabi.cmake" \
      -DCMAKE_BUILD_TYPE=Release > "$BUILD.configure.log" 2>&1 || { cat "$BUILD.configure.log"; exit 1; }
if ! cmake --build "$BUILD" -j"$(nproc)" > "$BUILD.build.log" 2>&1; then
  grep -E "error|Error" "$BUILD.build.log" | head -40
  echo "build failed; full log: $BUILD.build.log"
  exit 1
fi
sed -n '/Memory region/,/^ *AXI/p' "$BUILD.build.log"
grep -A1 -E "^ +text" "$BUILD.build.log"
echo "built $BUILD/wbc_h743.elf (log: $BUILD.build.log)"
