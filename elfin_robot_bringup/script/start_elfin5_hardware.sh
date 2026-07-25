#!/usr/bin/env bash

# Detect the E05 EtherCAT link, load the model, and start the Servo-Off driver.

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WORKSPACE=$(cd "$PROJECT_DIR/../.." && pwd)
RT_ROOT=/sys/fs/cgroup/cpu,cpuacct
RT_GROUP=$RT_ROOT/elfin-hardware
RT_RUNTIME_US=500000
LOCK_FILE=/run/lock/elfin5-hardware.lock
ORIGINAL_ARGS=("$@")
ALLOW_FREEDRIVE=false
ROSLAUNCH_ARGS=()

usage() {
    cat <<'EOF'
Usage: start_elfin5_hardware.sh [--freedrive] [roslaunch name:=value ...]

Without --freedrive, the gravity controller is loaded but real-hardware
freedrive remains locked. --freedrive opens that manager gate for this launch;
the controller still starts only after all runtime checks pass.
EOF
}

while (( $# > 0 )); do
    case "$1" in
        --freedrive) ALLOW_FREEDRIVE=true ;;
        -h|--help) usage; exit 0 ;;
        *) ROSLAUNCH_ARGS+=("$1") ;;
    esac
    shift
done

if (( EUID != 0 )); then
    exec sudo -E "$0" "${ORIGINAL_ARGS[@]}"
fi

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

# Clean dead ROS registrations left by a previously crashed launch.  This does
# not stop or unregister a node that still answers its XML-RPC health check.
if timeout 3 rosnode list >/dev/null 2>&1; then
    cleanup_output="$(printf 'y\n' | timeout 20 rosnode cleanup 2>&1 || true)"
    removed_nodes="$(grep '^Unregistering ' <<<"$cleanup_output" || true)"
    if [[ -n "$removed_nodes" ]]; then
        echo "Removed stale ROS node registrations:"
        sed 's/^Unregistering /  /' <<<"$removed_nodes"
    else
        echo "No stale ROS node registrations found."
    fi
fi

for command_name in chrt flock; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "Required command is missing: $command_name" >&2
        exit 69
    fi
done

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "Another Elfin hardware launch is already running." >&2
    exit 75
fi

interface=$(ELFIN_HARDWARE_LOCK_HELD=1 "$SCRIPT_DIR/detect_elfin_ethercat_interface.sh")
echo "Verified E05 EtherCAT interface: $interface"
echo "Starting the hardware stack with servos disabled at RR priority 10."
if [[ "$ALLOW_FREEDRIVE" == true ]]; then
    echo "Freedrive manager gate: UNLOCKED for this launch (controller remains stopped)."
else
    echo "Freedrive manager gate: LOCKED (use --freedrive for a supervised trial)."
fi

mkdir -p "$RT_GROUP"
if [[ -s "$RT_GROUP/tasks" ]]; then
    echo "The Elfin realtime cgroup is still occupied; run STOP_ELFIN.sh first." >&2
    exit 75
fi
printf '%s\n' "$RT_RUNTIME_US" > "$RT_GROUP/cpu.rt_runtime_us"
printf '%s\n' "$$" > "$RT_GROUP/tasks"

cleanup_rt_group() {
    printf '%s\n' "$$" > "$RT_ROOT/tasks" 2>/dev/null || true
    rmdir "$RT_GROUP" 2>/dev/null || true
}
trap cleanup_rt_group EXIT

chrt --rr 10 roslaunch elfin_robot_bringup elfin5_hardware_bringup.launch \
    elfin_ethernet_name:="$interface" \
    allow_hardware_freedrive:="$ALLOW_FREEDRIVE" \
    "${ROSLAUNCH_ARGS[@]}"
