#!/usr/bin/env bash
#
# Install the serial-device udev rules so each device gets a stable /dev/<name> symlink.
# Run on the host; re-run after editing 99-robot-serial.rules — it overwrites and reloads.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES="99-robot-serial.rules"
SUDO=""
[[ $EUID -ne 0 ]] && SUDO="sudo"

$SUDO cp "$SCRIPT_DIR/$RULES" "/etc/udev/rules.d/$RULES"
$SUDO udevadm control --reload-rules
$SUDO udevadm trigger --subsystem-match=tty

echo "Installed /etc/udev/rules.d/$RULES."
echo "If a symlink is missing, re-plug that device (or reboot) so it re-enumerates."
ls -l /dev/glove_left /dev/glove_right /dev/hand_left /dev/hand_right 2>/dev/null || true
