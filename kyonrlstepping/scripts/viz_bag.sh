#!/bin/bash

set -euo pipefail

usage() {
  echo "Usage: $0 <path_to_rosbag> [--legged|--b2w]"
  exit 1
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPLAY_BAG_DIR="$HOME/ibrido_ws/src/AugMPC/aug_mpc/scripts/utilities"
REPLAY_BAG_SCRIPT="$REPLAY_BAG_DIR/replay_bag.sh"
LAUNCH_VIZ_SCRIPT="$SCRIPT_DIR/launch_mpcviz.py"
BAG_PATH="$1"
MODE="${2:-}"
BAG_NAME="$(basename "$BAG_PATH")"

if [[ ! -x "$REPLAY_BAG_SCRIPT" ]]; then
  REPLAY_BAG_SCRIPT="$REPLAY_BAG_DIR/replay_bag.bash"
fi

extract_ns() {
  local bag_name="$1"
  if [[ "$bag_name" =~ ^rosbag_(.*_ID[^_]*)_ ]]; then
    printf '%s\n' "${BASH_REMATCH[1]}"
    return 0
  fi

  echo "Failed to extract namespace from bag name: $bag_name" >&2
  return 1
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ -n "${REPLAY_PID:-}" ]] && kill -0 "$REPLAY_PID" 2>/dev/null; then
    kill "$REPLAY_PID" 2>/dev/null || true
  fi
  if [[ -n "${VIZ_PID:-}" ]] && kill -0 "$VIZ_PID" 2>/dev/null; then
    kill "$VIZ_PID" 2>/dev/null || true
  fi

  wait "${REPLAY_PID:-}" 2>/dev/null || true
  wait "${VIZ_PID:-}" 2>/dev/null || true

  exit "$exit_code"
}

if [[ ! -d "$BAG_PATH" ]]; then
  echo "Rosbag directory not found: $BAG_PATH" >&2
  exit 1
fi

if [[ ! -x "$REPLAY_BAG_SCRIPT" ]]; then
  echo "Replay script not found or not executable: $REPLAY_BAG_SCRIPT" >&2
  exit 1
fi

if [[ ! -f "$LAUNCH_VIZ_SCRIPT" ]]; then
  echo "Visualizer script not found: $LAUNCH_VIZ_SCRIPT" >&2
  exit 1
fi

case "$MODE" in
  "")
    VIZ_ARGS=(--nodes_perc 10 --kyon_real --wheels)
    ;;
  --legged)
    VIZ_ARGS=(--nodes_perc 10 --kyon_real)
    ;;
  --b2w)
    VIZ_ARGS=(--b2w --nodes_perc 10)
    ;;
  *)
    echo "Unknown arg: $MODE" >&2
    usage
    ;;
esac

NS="$(extract_ns "$BAG_NAME")"

echo "Using namespace: $NS"

trap cleanup EXIT INT TERM

"$REPLAY_BAG_SCRIPT" "$BAG_PATH" --no-pause </dev/null &
REPLAY_PID=$!

sleep 1

python3 "$LAUNCH_VIZ_SCRIPT" "${VIZ_ARGS[@]}" --ns "$NS" &
VIZ_PID=$!

wait "$REPLAY_PID"
wait "$VIZ_PID"
