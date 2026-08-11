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
ENABLE_OCTOMAP=false
PANEL_GUI=true
REPLACE_EXISTING=false
RVIZ_CONFIG="$PROJECT_DIR/elfin5_moveit_config/launch/moveit_low_load.rviz"
GLOBAL_LAUNCH_WRAPPER=${ELFIN_GLOBAL_LAUNCH_WRAPPER:-/home/catas/ros_ws/src/elfin_vision/scripts/run_with_elfin_global_stop.sh}

hard_timeout() {
    timeout --signal=TERM --kill-after=1s "$@"
}

process_alive() {
    local pid=$1
    kill -0 "$pid" 2>/dev/null
}

stop_process_pattern() {
    local pattern=$1
    local label=$2
    local pids=()
    mapfile -t pids < <(pgrep -f "$pattern" 2>/dev/null || true)
    if (( ${#pids[@]} == 0 )); then
        return 0
    fi
    echo "Stopping old $label..."
    kill -TERM "${pids[@]}" 2>/dev/null || true
    for _attempt in $(seq 1 40); do
        local alive=false pid
        for pid in "${pids[@]}"; do
            process_alive "$pid" && alive=true
        done
        [[ "$alive" == false ]] && return 0
        sleep 0.1
    done
    echo "Old $label ignored SIGTERM; escalating that user-space component to SIGKILL." >&2
    kill -KILL "${pids[@]}" 2>/dev/null || true
}

node_is_live() {
    local node_name=$1
    local listed_nodes=$2
    grep -Fxq "$node_name" <<<"$listed_nodes" \
        && hard_timeout 1s rosnode ping -c 1 "$node_name" >/dev/null 2>&1
}

replace_existing_user_stacks() {
    local nodes node enable_state after_state

    echo "Taking over the old vision/Panel/MoveIt stack safely."
    hard_timeout 2s rosservice call \
        /elfin_vision/harvest_coordinator/command \
        "command: 'execute_disable'
target_index: -1
target_point: {x: 0.0, y: 0.0, z: 0.0}
target_frame: ''
target_label: ''
target_point_valid: false" >/dev/null 2>&1 || true
    hard_timeout 2s rosservice call \
        /elfin_vision/cockpit_controller/set_active \
        "data: false" >/dev/null 2>&1 || true
    hard_timeout 2s rosservice call \
        /elfin_vision/citrus_moveit_planner/stop "{}" \
        >/dev/null 2>&1 || true

    # A Panel takeover must not strand the arm in FREE or Servo On while its
    # control UI and MoveIt connection are being replaced.
    hard_timeout 5s rosservice call \
        /elfin_freedrive_manager/set_freedrive \
        "data: false" >/dev/null 2>&1 || true
    enable_state="$(hard_timeout 2s rostopic echo -n 1 \
        /elfin_ros_control/elfin/enable_state 2>/dev/null || true)"
    if grep -Fq 'data: True' <<<"$enable_state"; then
        echo "Requesting Servo Off before replacing the Panel."
        hard_timeout 5s rosservice call \
            /elfin_ros_control/elfin/disable_robot \
            "data: true" >/dev/null 2>&1 || true
        after_state="$(hard_timeout 2s rostopic echo -n 1 \
            /elfin_ros_control/elfin/enable_state 2>/dev/null || true)"
        if ! grep -Fq 'data: False' <<<"$after_state"; then
            echo "Servo Off was not confirmed; refusing to replace the live Panel." >&2
            return 77
        fi
        echo "Servo Off confirmed."
    fi

    # Stop launch owners before killing their respawning GUI children. TERM is
    # intentionally used so the cross-terminal wrapper does not invoke a
    # second global stop; the hardware ROS master remains alive.
    stop_process_pattern \
        '[/]roslaunch elfin_vision (citrus_system|citrus_sensor_runtime)\.launch' \
        'vision launch'
    stop_process_pattern \
        '[/]roslaunch elfin_robot_bringup elfin5_control_panel\.launch' \
        'Panel/MoveIt launch'
    stop_process_pattern \
        '[/]roslaunch elfin5_moveit_config move_group\.launch' \
        'standalone MoveIt recovery launch'
    stop_process_pattern \
        '[/]roslaunch elfin5_moveit_config moveit_rviz\.launch' \
        'standalone RViz launch'

    nodes="$(hard_timeout 3s rosnode list 2>/dev/null || true)"
    for node in /elfin_vision/dashboard \
                /elfin_vision/cockpit_controller \
                /elfin_vision/servo_server \
                /elfin_vision/harvest_coordinator \
                /elfin_vision/citrus_moveit_planner \
                /elfin_vision/environment_cloud_filter \
                /elfin_vision/tool_collision_manager \
                /elfin_vision/citrus_rgbd_node \
                /elfin_vision/elfin_vision/citrus_rgbd_node \
                /elfin_vision/publish_camera_tf \
                /publish_camera_tf \
                /camera/realsense2_camera \
                /camera/realsense2_camera_manager \
                /elfin_vision/camera/realsense2_camera \
                /elfin_vision/camera/realsense2_camera_manager \
                /elfin_gui \
                /elfin_basic_api \
                /move_group; do
        if grep -Fxq "$node" <<<"$nodes"; then
            hard_timeout 3s rosnode kill "$node" >/dev/null 2>&1 || true
        fi
    done
    while IFS= read -r node; do
        [[ -n "$node" ]] || continue
        hard_timeout 3s rosnode kill "$node" >/dev/null 2>&1 || true
    done < <(grep '^/rviz' <<<"$nodes" || true)

    stop_process_pattern '[/]elfin_gui\.py' 'orphaned Panel GUI'
    stop_process_pattern '[/]elfin_basic_api_node' 'orphaned Basic API'
    stop_process_pattern '[/]moveit_ros_move_group/move_group' 'orphaned move_group'

    printf 'y\n' | hard_timeout 10s rosnode cleanup >/dev/null 2>&1 || true
    nodes="$(hard_timeout 3s rosnode list 2>/dev/null || true)"
    for node in /elfin_gui /elfin_basic_api /move_group \
                /elfin_vision/dashboard \
                /elfin_vision/cockpit_controller \
                /elfin_vision/citrus_moveit_planner \
                /elfin_vision/environment_cloud_filter; do
        if node_is_live "$node" "$nodes"; then
            echo "$node survived replacement; refusing to create a duplicate stack." >&2
            return 75
        fi
    done
    echo "Old vision/Panel/MoveIt stack has been removed; hardware remains Servo Off."
}

usage() {
    cat <<'EOF'
Usage: start_elfin5_panel.sh [--replace-existing] [--rviz|--rviz-egl|--rviz-original] [--octomap] [--no-panel-gui]

Run this only after either:
  * start_elfin5_hardware.sh (real E05, Servo Off), or
  * the Elfin5 Gazebo launch (simulation).

--rviz           Open the low-load RViz configuration (recommended).
--rviz-egl       Use the same configuration through Qt's EGL fallback.
--rviz-original  Open the original MotionPlanning configuration for diagnosis;
                 it may crash after creating a MoveGroup connection.
--octomap        Load the RealSense point cloud as MoveIt collision geometry.
                 Use this for environment-aware citrus planning.
--no-panel-gui   Keep MoveIt and the Basic API running without the legacy wx
                 Panel. The separate citrus dashboard/cockpit remains usable.
--replace-existing
                 Revoke vision execution, exit FREE, request Servo Off, and
                 replace old vision/Panel/MoveIt while keeping hardware alive.
EOF
}

while (($#)); do
    case "$1" in
        --rviz) RVIZ=true ;;
        --rviz-egl) RVIZ=true; RVIZ_EGL=true ;;
        --rviz-original)
            RVIZ=true
            RVIZ_CONFIG="$PROJECT_DIR/elfin5_moveit_config/launch/moveit.rviz"
            ;;
        --octomap) ENABLE_OCTOMAP=true ;;
        --no-panel-gui) PANEL_GUI=false ;;
        --replace-existing) REPLACE_EXISTING=true ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 64 ;;
    esac
    shift
done

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}
export ROS_IP=${ROS_IP:-127.0.0.1}
unset ROS_HOSTNAME
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

if [[ "$REPLACE_EXISTING" == true ]]; then
    replace_existing_user_stacks || exit $?
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

echo "Starting MoveIt, Basic API and Control Panel (legacy GUI: $PANEL_GUI, RViz: $RVIZ, EGL fallback: $RVIZ_EGL, OctoMap: $ENABLE_OCTOMAP)."
echo "Wait for 'Ready to take commands for planning group elfin_arm'."
echo "Ctrl+C in this terminal performs a complete stop: GUI, vision, Panel and Servo Off."
if [[ ! -x "$GLOBAL_LAUNCH_WRAPPER" ]]; then
    echo "Global-stop launch wrapper is missing: $GLOBAL_LAUNCH_WRAPPER" >&2
    exit 69
fi
exec "$GLOBAL_LAUNCH_WRAPPER" "Panel/MoveIt terminal" \
    roslaunch elfin_robot_bringup elfin5_control_panel.launch \
    display:="$RVIZ" rviz_config:="$RVIZ_CONFIG" rviz_egl:="$RVIZ_EGL" \
    enable_octomap:="$ENABLE_OCTOMAP" panel_gui:="$PANEL_GUI"
