#!/usr/bin/env bash
#
# Install and configure the SteamVR runtime for offline, headless tracker use.
#
# SteamVR is Valve-proprietary software and is NOT redistributed with this repository.
# This script downloads it from Valve via steamcmd (SteamVR is a free app, id 250820)
# and applies the configuration needed to run it without a headset.
#
# Run this on the HOST: it only fetches/configures the runtime into this (mounted) folder.
# SteamVR itself runs inside the container via launch.sh, which has the runtime libraries
# from the Docker image.
#
# Usage:  ./setup.sh [steam_account_name]
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME="$SCRIPT_DIR/runtime"   # the SteamVR runtime is downloaded here
STEAMVR_APPID=250820

log()  { printf '\033[0;34m[setup]\033[0m %s\n' "$1"; }
warn() { printf '\033[1;33m[setup]\033[0m %s\n' "$1"; }

SUDO=""
[[ $EUID -ne 0 ]] && SUDO="sudo"

# 1. steamcmd, used only to download SteamVR (it needs the i386 architecture).
# The SteamVR runtime libraries live in the Docker image, not here.
install_steamcmd() {
  log "Installing steamcmd..."
  $SUDO dpkg --add-architecture i386 || true
  $SUDO apt-get update
  $SUDO apt-get install -y steamcmd
}

# 2. Download the SteamVR runtime into this directory via steamcmd.
download_steamvr() {
  if [[ -x "$RUNTIME/bin/linux64/vrstartup" ]]; then
    log "SteamVR runtime already present — skipping download."
    return
  fi
  local account="${1:-}"
  if [[ -z "$account" ]]; then
    read -rp "Steam account name (a free account is sufficient): " account
  fi
  log "Downloading SteamVR (app $STEAMVR_APPID) into $RUNTIME ..."
  steamcmd \
    +force_install_dir "$RUNTIME" \
    +login "$account" \
    +app_update "$STEAMVR_APPID" validate \
    +quit
}

# 3. Enable headless operation: run with trackers only, no HMD required.
#    (The steamclient.so stub is created at launch time by launch.sh, inside the container.)
enable_headless() {
  local cfg="$RUNTIME/resources/settings/default.vrsettings"
  if [[ ! -f "$cfg" ]]; then
    warn "default.vrsettings not found; set steamvr.requireHmd=false manually."
    return
  fi
  if python3 - "$cfg" <<'PY'
import json, sys
path = sys.argv[1]
with open(path) as f:
    cfg = json.load(f)
cfg.setdefault("steamvr", {})["requireHmd"] = False
with open(path, "w") as f:
    json.dump(cfg, f, indent=3)
PY
  then
    log "Headless mode enabled (steamvr.requireHmd = false)."
  else
    warn "Could not edit default.vrsettings automatically; set requireHmd=false manually."
  fi
}

main() {
  install_steamcmd
  download_steamvr "${1:-}"
  enable_headless
  echo
  log "Done. Inside the container, start SteamVR with:  ./assets/SteamVR/launch.sh"
}

main "$@"
