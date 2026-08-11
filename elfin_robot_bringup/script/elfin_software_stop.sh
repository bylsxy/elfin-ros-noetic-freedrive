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
LAUNCH_PATTERN='[/]roslaunch elfin_robot_bringup elfin5_hardware_bringup.launch'
PID_FILE=/run/elfin5-hardware.pid

hard_timeout() {
    timeout --signal=TERM --kill-after=1s "$@"
}

process_exists() {
    local pid=$1
    kill -0 "$pid" 2>/dev/null || sudo -n kill -0 "$pid" 2>/dev/null
}

if [[ -r "$ROS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
fi
if [[ -r "$WS_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$WS_SETUP"
fi

echo "[1/2] Requesting Servo Off on all Elfin joint modules..."
service_output="$(hard_timeout 3s rosservice call "$DISABLE_SERVICE" "data: true" 2>&1)"
service_status=$?
echo "$service_output"

if [[ $service_status -eq 0 && "$service_output" == *"success: True"* ]]; then
    echo "Servo Off confirmed by the driver."
else
    echo "Servo Off was not confirmed within 3 seconds; shutting down the driver now." >&2
fi

echo "[2/2] Shutting down the EtherCAT hardware node..."
hard_timeout 3s rosnode kill "$HARDWARE_NODE" >/dev/null 2>&1 || true

# Give the ROS SIGINT handler time to join the 1 kHz loop and run destructors.
driver_stopped=false
for _ in 1 2 3 4 5; do
    if ! pgrep -f "$DRIVER_PATTERN" >/dev/null 2>&1; then
        driver_stopped=true
        break
    fi
    sleep 0.2
done

if [[ "$driver_stopped" != true ]] && pgrep -f "$DRIVER_PATTERN" >/dev/null 2>&1; then
    sudo -n pkill -INT -f "$DRIVER_PATTERN" >/dev/null 2>&1 || \
        pkill -INT -f "$DRIVER_PATTERN" >/dev/null 2>&1 || true
fi

for _ in 1 2 3 4 5; do
    if ! pgrep -f "$DRIVER_PATTERN" >/dev/null 2>&1; then
        driver_stopped=true
        break
    fi
    sleep 0.2
done

launch_pids=()
if [[ -r "$PID_FILE" ]]; then
    launch_pid="$(<"$PID_FILE")"
    if [[ "$launch_pid" =~ ^[0-9]+$ ]] && process_exists "$launch_pid"; then
        launch_pids+=("$launch_pid")
    fi
fi
while IFS= read -r launch_pid; do
    [[ -n "$launch_pid" ]] || continue
    if [[ ! " ${launch_pids[*]-} " =~ " $launch_pid " ]]; then
        launch_pids+=("$launch_pid")
    fi
done < <(pgrep -f "$LAUNCH_PATTERN" 2>/dev/null || true)

if (( ${#launch_pids[@]} > 0 )); then
    sudo -n kill -INT "${launch_pids[@]}" >/dev/null 2>&1 || \
        kill -INT "${launch_pids[@]}" >/dev/null 2>&1 || true
    for _ in $(seq 1 30); do
        any_running=false
        for launch_pid in "${launch_pids[@]}"; do
            process_exists "$launch_pid" && any_running=true
        done
        [[ "$any_running" == false ]] && break
        sleep 0.2
    done
fi

launch_running=false
for launch_pid in "${launch_pids[@]}"; do
    process_exists "$launch_pid" && launch_running=true
done

if [[ "$driver_stopped" == true && "$launch_running" == false ]]; then
    sudo -n rm -f "$PID_FILE" >/dev/null 2>&1 || rm -f "$PID_FILE" 2>/dev/null || true
    echo "Elfin hardware stack stopped and its launch lock was released. Keep the robot power cut-off within reach."
    exit 0
fi

echo "WARNING: the Elfin hardware stack did not stop completely." >&2
echo "Use the upstream physical power cut-off immediately." >&2
exit 1
