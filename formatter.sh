#!/usr/bin/env bash
# Usage: formatter.sh [--check]
#   (default) Reformat all C/C++ sources in-place with clang-format-14.
#   --check   Dry-run: exit non-zero if any file would be reformatted.

set -euo pipefail

if [ "${1:-}" = "--check" ]; then
  MODE=(--dry-run --Werror)
else
  MODE=(-i)
fi

find \
    module \
    apps \
    -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \
  -print0 | xargs -0 clang-format-14 "${MODE[@]}"
