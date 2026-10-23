#!/usr/bin/env bash
# Build (if needed) and launch F1 Hotlap. Extra arguments are passed to the game.
set -euo pipefail

cd "$(dirname "$0")"

if ! command -v cmake >/dev/null; then
    echo "cmake not found - install it first (e.g. sudo pacman -S cmake)" >&2
    exit 1
fi

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build build --parallel >/dev/null

exec ./build/f1_hotlap "$@"
