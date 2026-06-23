#!/usr/bin/env bash
#
# Launch the SteamVR runtime in offline, headless mode (no Steam client, no HMD).
# SteamVR must already be installed in this directory — run ./setup.sh first.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME="$SCRIPT_DIR/runtime"   # the SteamVR runtime is installed here by setup.sh

log() { printf '\033[0;34m[steamvr]\033[0m %s\n' "$1"; }
err() { printf '\033[0;31m[steamvr]\033[0m %s\n' "$1" >&2; }

# Verify the runtime is present.
required=(
  "bin/linux64/vrstartup"
  "bin/linux64/vrserver"
  "bin/linux64/vrcompositor"
  "bin/linux64/vrcompositor-launcher"
)
for f in "${required[@]}"; do
  if [[ ! -e "$RUNTIME/$f" ]]; then
    err "Missing $f — SteamVR is not installed. Run ./setup.sh first."
    exit 1
  fi
done

# steamclient.so stub — lets SteamVR start without the Steam client. Built as a minimal
# valid shared object (an empty file would fail dlopen with "file too short").
stub="$HOME/.steam/sdk64/steamclient.so"
mkdir -p "$HOME/.steam/sdk64"
if [[ ! -s "$stub" ]]; then
  echo 'void SteamAPI_Init(void){}' | gcc -shared -fPIC -x c - -o "$stub" 2>/dev/null || touch "$stub"
fi

# Grant the compositor real-time scheduling (cap_sys_nice); needs root the first time.
launcher="$RUNTIME/bin/linux64/vrcompositor-launcher"
if ! getcap "$launcher" 2>/dev/null | grep -q cap_sys_nice; then
  log "Granting cap_sys_nice to vrcompositor-launcher..."
  setcap 'CAP_SYS_NICE=eip' "$launcher" 2>/dev/null \
    || sudo setcap 'CAP_SYS_NICE=eip' "$launcher" 2>/dev/null \
    || err "Could not set cap_sys_nice; the compositor may run at lower priority."
fi

# Environment for an offline, Steam-runtime-free launch.
export STEAMVR_TOOLSDIR="$RUNTIME"
export STEAMVR_VRENV="$RUNTIME/bin/vrenv.sh"
export LD_LIBRARY_PATH="$RUNTIME/bin/linux64:$RUNTIME/bin/linux64/qt/lib:$RUNTIME/bin/linux64/cef${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export VRCOMPOSITOR_LD_LIBRARY_PATH="$LD_LIBRARY_PATH"
export PRESSURE_VESSEL_RUNTIME=""
export STEAM_RUNTIME=""
export STEAMVR_USE_VULKAN=1
export STEAMVR_FORCE_VULKAN=1

log "Starting SteamVR (offline, headless)..."
exec "$RUNTIME/bin/linux64/vrstartup" "$@"
