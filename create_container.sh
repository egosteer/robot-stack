#!/bin/bash
set -euo pipefail

# if [ $# -lt 2 ]; then
#   echo "Docker Image starter
#   usage: $0 <container_name> <image[:tag]>
#   example: $0 my_container <image>:humble"
#   exit 1
# fi

CONTAINER_NAME="$1"
# Image to run. Defaults to the public image (auto-pulled if absent); override with ROBOT_STACK_IMAGE.
IMAGE="${ROBOT_STACK_IMAGE:-egosteerai/robot-stack:latest}"

# Mount the repo that contains this script, so it works from any host / location.
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# If a container with the same name exists, warn and exit (safer; to restart, manually rm -f first).
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "Container '${CONTAINER_NAME}' already exists. Remove it first:"
  echo "  docker rm -f ${CONTAINER_NAME}"
  exit 1
fi

# Auto-detect and set up the display environment.
# 1) Auto-detect DISPLAY from the X sockets in /tmp/.X11-unix (X0 -> :0, X1 -> :1, ...).
if [ -z "${DISPLAY:-}" ]; then
  echo "Auto-detecting DISPLAY..."
  for sock in /tmp/.X11-unix/X*; do
    [ -e "$sock" ] || continue
    candidate=":${sock##*/X}"
    # If xdpyinfo is available, verify the display actually responds.
    if command -v xdpyinfo >/dev/null 2>&1; then
      DISPLAY="$candidate" xdpyinfo >/dev/null 2>&1 || continue
    fi
    export DISPLAY="$candidate"
    break
  done
  export DISPLAY="${DISPLAY:-:0}"
  echo "Using DISPLAY: $DISPLAY"
fi

# 2) Auto-detect and set up XAUTHORITY
XA="${XAUTHORITY:-}"
if [ -z "$XA" ] || [ ! -f "$XA" ]; then
  # Prefer the user's home directory.
  XA="${HOME}/.Xauthority"
  if [ ! -f "$XA" ]; then
    # Look in the user runtime directory.
    XA=$(find "/run/user/$(id -u)" -maxdepth 3 -type f -name "Xauth*" 2>/dev/null | head -n1 || true)
  fi
  if [ ! -f "$XA" ]; then
    # Look in the system temp directory.
    XA=$(find /tmp -maxdepth 2 -type f -name "Xauth*" -user "$(whoami)" 2>/dev/null | head -n1 || true)
  fi
  if [ ! -f "$XA" ]; then
    # As a last resort, create a new one.
    XA="${HOME}/.Xauthority"
    touch "$XA" 2>/dev/null || XA=""
  fi
fi
export XAUTHORITY="$XA"

# 3) Set up X11 access permissions
if command -v xhost >/dev/null 2>&1; then
  xhost +local: >/dev/null 2>&1 || true
  xhost +SI:localuser:root >/dev/null 2>&1 || true
  xhost +SI:localuser:"$(whoami)" >/dev/null 2>&1 || true
fi

if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ] && command -v xauth >/dev/null 2>&1; then
  xauth list "$DISPLAY" 2>/dev/null | while read line; do
    echo "$line" | xauth -f "$XAUTHORITY" merge - 2>/dev/null || true
  done
fi

# GPU detection
GPU_OPTIONS=()
if command -v nvidia-smi >/dev/null 2>&1; then
  GPU_OPTIONS+=(--gpus all)
fi

# Optional: mount the host robot_sdk into the container (uncomment and fix the path if needed).
# HOST_ROBOT_SDK="/path/to/robot_sdk"
# SDK_MOUNT_OPT=(-v "${HOST_ROBOT_SDK}:/root/robot_sdk")
SDK_MOUNT_OPT=()

# X authority file mount
XAUTH_MOUNT_OPT=()
if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ]; then
  XAUTH_MOUNT_OPT=(-v "$XAUTHORITY":/root/.Xauthority:ro)
fi

# Display-related mounts
EXTRA_DISPLAY_MOUNTS=(
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
)

if [ -d "/dev/dri" ]; then
  EXTRA_DISPLAY_MOUNTS+=(--device /dev/dri)
fi

if [ -n "${WAYLAND_DISPLAY:-}" ] && [ -S "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}" ]; then
  EXTRA_DISPLAY_MOUNTS+=(-v "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/tmp/${WAYLAND_DISPLAY}:rw")
fi

# Start the container
echo "Starting container..."
docker run -d \
  ${GPU_OPTIONS[@]} \
  --privileged \
  --network=host \
  --ipc=host \
  --name "${CONTAINER_NAME}" \
  -w /root/workspace/robot-stack \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e DISPLAY="${DISPLAY}" \
  -e XAUTHORITY=/root/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_GRAPHICSSYSTEM=native \
  -e QT_LOGGING_RULES="qt.qpa.xcb.xcb_error.debug=false" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,display \
  -e WGPU_BACKEND=gl \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}" \
  -e XDG_SESSION_TYPE="${XDG_SESSION_TYPE:-x11}" \
  -e DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
  -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
  -e VENVPYTHONPATH=/usr/bin/python3 \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_LOCALHOST_ONLY=1 \
  -e PROJECT_DIR=/root/workspace/robot-stack \
  ${EXTRA_DISPLAY_MOUNTS[@]} \
  -v /dev:/dev \
  -v "${SCRIPT_DIR}":/root/workspace/robot-stack \
  ${XAUTH_MOUNT_OPT[@]} \
  ${SDK_MOUNT_OPT[@]} \
  "${IMAGE}" \
  bash -lc '
    set -e
    echo "Container started, DISPLAY=$DISPLAY"
    # Disable the firewall.
    # Note:
    # - This container uses --network=host, sharing the host network namespace,
    #   so the operations below effectively disable/clear firewall rules on the host.
    # - If the host has no firewall enabled, this section has no extra effect.
    # Strategy: try to disable ufw/firewalld and flush nftables/iptables (IPv4/IPv6); ignore any command failures.
    {
      if command -v ufw >/dev/null 2>&1; then
        ufw disable || true
      fi
      if command -v firewall-cmd >/dev/null 2>&1; then
        firewall-cmd --state >/dev/null 2>&1 && firewall-cmd --set-default-zone=trusted || true
        firewall-cmd --permanent --set-default-zone=trusted || true
        firewall-cmd --reload || true
      fi
      if command -v nft >/dev/null 2>&1; then
        nft flush ruleset || true
      fi
      if command -v iptables >/dev/null 2>&1; then
        iptables -F || true
        iptables -t nat -F || true
        iptables -t mangle -F || true
        iptables -X || true
      fi
      if command -v ip6tables >/dev/null 2>&1; then
        ip6tables -F || true
        ip6tables -t nat -F || true
        ip6tables -t mangle -F || true
        ip6tables -X || true
      fi
    } || true

    # Enable multicast on loopback (DDS discovery needs it under --network=host + localhost-only).
    ip link set lo multicast on 2>/dev/null || true

    # Source the (mounted, editable) shell env from interactive shells.
    grep -q "robot-stack/robot.bashrc" /root/.bashrc 2>/dev/null \
      || echo "source /root/workspace/robot-stack/robot.bashrc" >> /root/.bashrc

    trap : TERM INT
    sleep infinity & wait
  '

# Verify the container started
sleep 2
if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is running"
else
  echo "Failed to start container '${CONTAINER_NAME}'"
  exit 1
fi

# Open an interactive shell
echo "Opening shell..."
docker exec -it "${CONTAINER_NAME}" bash
