#!/usr/bin/env bash
set -e

# Load Wayland environment for the native OF build (not of-env.sh / XWayland)
source "$HOME/of-wayland-env.sh"

# Ensure the display output is flipped for omnivisu
source "$HOME/of-flip.sh"

APP_DIR="$HOME/Documents/of_v0.12.1_linux64_gcc6_release_wayland/apps/myApps/omnivisu"
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
