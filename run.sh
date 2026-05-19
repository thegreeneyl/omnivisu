#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export GENICAM_GENTL64_PATH="${GENICAM_GENTL64_PATH:-/usr/local/lib/x86_64-linux-gnu/ids-peak/cti}"

make Release -j"$(nproc)"
exec make RunRelease
