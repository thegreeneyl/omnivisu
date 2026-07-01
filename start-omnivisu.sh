#!/usr/bin/env bash
set -e

# Load X11 / XWayland environment for openFrameworks
source "$HOME/of-env.sh"

APP_DIR="$HOME/Documents/of_v0.12.1_linux64_gcc6_release/apps/myApps/omnivisu"
APP_BIN="$APP_DIR/bin/omnivisu"

cd "$APP_DIR"

if [ ! -x "$APP_BIN" ]; then
  echo "Executable not found or not executable:"
  echo "$APP_BIN"
  echo
  echo "Trying to build first..."
  make Release
fi

echo "Starting omnivisu..."
exec "$APP_BIN"
