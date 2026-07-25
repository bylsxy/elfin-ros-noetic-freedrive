#!/usr/bin/env bash

# Start the Elfin5 MoveIt stack and Control Panel in the required order.
# This script deliberately does not start, enable, or move the hardware.

set -euo pipefail

ROS_SETUP=/opt/ros/noetic/setup.bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WORKSPACE=$(cd "$PROJECT_DIR/../.." && pwd)
RVIZ=false
RVIZ_EGL=false
RVIZ_CONFIG="$PROJECT_DIR/elfin5_moveit_config/launch/moveit_low_load.rviz"

usage() {
    cat <<'EOF'
Usage: start_elfin5_panel.sh [--rviz|--rviz-egl|--rviz-original]

Run this only after either:
  * start_elfin5_hardware.sh (real E05, Servo Off), or
  * the Elfin5 Gazebo launch (simulation).

--rviz           Open the low-load RViz configuration (recommended).
--rviz-egl       Use the same configuration through Qt's EGL fallback.
--rviz-original  Open the original, heavier RViz configuration for comparison.
EOF
}

case "${1:-}" in
    "") ;;
    --rviz) RVIZ=true ;;
    --rviz-egl) RVIZ=true; RVIZ_EGL=true ;;
    --rviz-original)
        RVIZ=true
        RVIZ_CONFIG="$PROJECT_DIR/elfin5_moveit_config/launch/moveit.rviz"
        ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 64 ;;
esac

source "$ROS_SETUP"
source "$WORKSPACE/devel/setup.bash"

if ! timeout 3 rosnode list >/dev/null 2>&1; then
    echo "No ROS master is running." >&2
    echo "For the real E05, first run $SCRIPT_DIR/start_elfin5_hardware.sh in another terminal." >&2
    echo "For simulation, first start the Elfin5 Gazebo launch." >&2
    exit 69
fi

# roslaunch crashes can leave names registered in rosmaster even though their
# XML-RPC endpoints no longer exist.  Purge those records before deciding that
# a second stack is running.
cleanup_output="$(printf 'y\n' | timeout 20 rosnode cleanup 2>&1 || true)"
removed_nodes="$(grep '^Unregistering ' <<<"$cleanup_output" || true)"
if [[ -n "$removed_nodes" ]]; then
    echo "Removed stale ROS node registrations:"
    sed 's/^Unregistering /  /' <<<"$removed_nodes"
else
    echo "No stale ROS node registrations found."
fi

if ! rosparam get /robot_description >/dev/null 2>&1; then
    echo "The Elfin5 robot model is not loaded; refusing to start an incomplete Panel stack." >&2
    echo "Start the hardware or Gazebo entry first." >&2
    exit 69
fi

active_nodes="$(rosnode list 2>/dev/null || true)"
for node_name in /move_group /elfin_basic_api /elfin_gui; do
    if grep -Fxq "$node_name" <<<"$active_nodes"; then
        if timeout 3 rosnode ping -c 1 "$node_name" >/dev/null 2>&1; then
            echo "$node_name is already running and responding." >&2
            echo "Return to its original terminal and press Ctrl+C before starting a second Panel stack." >&2
            exit 75
        fi
        echo "Ignoring unreachable stale registration: $node_name"
    fi
done

if ! timeout 4 rostopic echo -n 1 /joint_states >/dev/null 2>&1; then
    echo "No fresh /joint_states message was received." >&2
    echo "The hardware/Gazebo state source is not ready; the Panel was not started." >&2
    exit 69
fi

service_names="$(rosservice list 2>/dev/null || true)"
if grep -Fxq /elfin_ros_control/elfin/enable_robot <<<"$service_names" \
   && timeout 3 rosnode ping -c 1 /elfin_ros_control >/dev/null 2>&1; then
    enable_state="$(timeout 3 rostopic echo -n 1 /elfin_ros_control/elfin/enable_state 2>&1 || true)"
    if grep -Fq 'data: True' <<<"$enable_state"; then
        echo "The real robot is already Servo On; refusing to hot-start the planning stack." >&2
        echo "Use STOP_ELFIN.sh or the existing Panel to Servo Off first." >&2
        exit 77
    fi
    echo "Detected the real E05 hardware stack with Servo Off."
else
    echo "No real-driver enable service detected; starting the Panel for simulation/visualization."
fi

echo "Starting MoveIt, Basic API and Control Panel (RViz: $RVIZ, EGL fallback: $RVIZ_EGL)."
echo "Wait for 'Ready to take commands for planning group elfin_arm'."
exec roslaunch elfin_robot_bringup elfin5_control_panel.launch \
    display:="$RVIZ" rviz_config:="$RVIZ_CONFIG" rviz_egl:="$RVIZ_EGL"
