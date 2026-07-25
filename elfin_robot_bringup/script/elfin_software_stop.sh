#!/usr/bin/env bash

# Strongest stop available in the current ROS stack. This is not a
# safety-rated emergency stop and cannot replace a wired power cut-off.

set -u

ROS_SETUP=/opt/ros/noetic/setup.bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WORKSPACE=$(cd "$PROJECT_DIR/../.." && pwd)
WS_SETUP=$WORKSPACE/devel/setup.bash
DISABLE_SERVICE=/elfin_ros_control/elfin/disable_robot
HARDWARE_NODE=/elfin_ros_control
DRIVER_PATTERN='[/]elfin_ros_control/elfin_hardware_interface'

if [[ -r "$ROS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
fi
if [[ -r "$WS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$WS_SETUP"
fi

echo "[1/2] Requesting Servo Off on all Elfin joint modules..."
service_output="$(timeout 3s rosservice call "$DISABLE_SERVICE" "data: true" 2>&1)"
service_status=$?
echo "$service_output"

if [[ $service_status -eq 0 && "$service_output" == *"success: True"* ]]; then
    echo "Servo Off confirmed by the driver."
else
    echo "Servo Off was not confirmed within 3 seconds; shutting down the driver now." >&2
fi

echo "[2/2] Shutting down the EtherCAT hardware node..."
timeout 3s rosnode kill "$HARDWARE_NODE" >/dev/null 2>&1 || true

# Give the ROS SIGINT handler time to join the 1 kHz loop and run destructors.
for _ in 1 2 3 4 5; do
    if ! pgrep -f "$DRIVER_PATTERN" >/dev/null 2>&1; then
        echo "Elfin hardware driver stopped. Keep the robot power cut-off within reach."
        exit 0
    fi
    sleep 0.2
done

if pgrep -f "$DRIVER_PATTERN" >/dev/null 2>&1; then
    sudo -n pkill -INT -f "$DRIVER_PATTERN" >/dev/null 2>&1 || \
        pkill -INT -f "$DRIVER_PATTERN" >/dev/null 2>&1 || true
fi

for _ in 1 2 3 4 5; do
    if ! pgrep -f "$DRIVER_PATTERN" >/dev/null 2>&1; then
        echo "Elfin hardware driver stopped. Keep the robot power cut-off within reach."
        exit 0
    fi
    sleep 0.2
done

echo "WARNING: the hardware driver is still running." >&2
echo "Use the upstream physical power cut-off immediately." >&2
exit 1
