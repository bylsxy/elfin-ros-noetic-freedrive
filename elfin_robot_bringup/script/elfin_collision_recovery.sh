#!/usr/bin/env bash

# Put the Elfin stack into a non-driving state after unexpected contact and
# capture evidence before a fault is cleared.  This script never enables a
# motor, releases a brake, or sends a recovery trajectory.

set -uo pipefail

ROS_SETUP=/opt/ros/noetic/setup.bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WORKSPACE=$(cd "$PROJECT_DIR/../.." && pwd)
REPORT_DIR=${HOME:-/tmp}/elfin_recovery_reports
MODE=${1:---stop}

usage() {
    cat <<'EOF'
Usage:
  elfin_collision_recovery.sh
      Cancel motion, request Servo Off, keep all brakes closed, and save a report.

  elfin_collision_recovery.sh --clear-fault
      Do the same safe stop first, then clear the drive fault once. This still
      does not Servo On or move the robot. Use only after the obstruction/load
      has been removed or the arm has been secured with rated support.
EOF
}

case "$MODE" in
    --stop) ;;
    --clear-fault) ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 64 ;;
esac

source "$ROS_SETUP"
source "$WORKSPACE/devel/setup.bash"

if ! timeout 3 rosnode list >/dev/null 2>&1; then
    echo "No ROS master is reachable. The software stop cannot be confirmed." >&2
    echo "Keep people clear and use the upstream power cut-off if the robot may still be energized." >&2
    exit 69
fi

mkdir -p "$REPORT_DIR"
REPORT="$REPORT_DIR/$(date +%Y%m%d-%H%M%S)-collision-recovery.log"
exec > >(tee -a "$REPORT") 2>&1

service_exists() {
    rosservice list 2>/dev/null | grep -Fxq "$1"
}

topic_exists() {
    rostopic list 2>/dev/null | grep -Fxq "$1"
}

read_topic_once() {
    local topic=$1
    if topic_exists "$topic"; then
        echo "--- $topic"
        timeout 3 rostopic echo -n 1 "$topic" || true
    else
        echo "--- $topic (not available)"
    fi
}

call_setbool() {
    local service=$1
    if service_exists "$service"; then
        echo "--- $service"
        timeout 5 rosservice call "$service" "data: true" || true
    else
        echo "--- $service (not available)"
    fi
}

snapshot() {
    local label=$1
    echo
    echo "===== $label ====="
    date --iso-8601=seconds
    read_topic_once /elfin_ros_control/elfin/enable_state
    read_topic_once /elfin_ros_control/elfin/fault_state
    read_topic_once /joint_states
    call_setbool /elfin_ros_control/elfin/get_motion_state
    call_setbool /elfin_ros_control/elfin/get_pos_align_state
    call_setbool /elfin_ros_control/elfin/get_txpdo
    if service_exists /controller_manager/list_controllers; then
        echo "--- /controller_manager/list_controllers"
        timeout 5 rosservice call /controller_manager/list_controllers "{}" || true
    fi
}

echo "Elfin collision recovery: safe stop and diagnostics"
echo "This program will NOT open brakes, Servo On, or command a retreat."
echo "Report: $REPORT"

snapshot "BEFORE SAFE STOP"

echo
echo "===== CANCELLING COMMANDS AND REQUESTING SERVO OFF ====="
if topic_exists /elfin_arm_controller/follow_joint_trajectory/cancel; then
    timeout 3 rostopic pub -1 \
        /elfin_arm_controller/follow_joint_trajectory/cancel \
        actionlib_msgs/GoalID '{}' || true
fi

call_setbool /elfin_basic_api/teleop/stop_teleop

if service_exists /elfin_basic_api/disable_robot; then
    call_setbool /elfin_basic_api/disable_robot
else
    call_setbool /elfin_ros_control/elfin/disable_robot
fi

# Stop the position controller even if Basic API is missing or crashed.
if service_exists /controller_manager/switch_controller; then
    echo "--- /controller_manager/switch_controller (stop elfin_arm_controller)"
    timeout 5 rosservice call /controller_manager/switch_controller \
        "start_controllers: []
stop_controllers: ['elfin_arm_controller']
strictness: 1
start_asap: false
timeout: 0.0" || true
fi

# A second raw Servo Off request is intentional and idempotent.
call_setbool /elfin_ros_control/elfin/disable_robot
sleep 1

enable_state="$(timeout 3 rostopic echo -n 1 /elfin_ros_control/elfin/enable_state 2>&1 || true)"
if ! grep -Fq 'data: False' <<<"$enable_state"; then
    echo "ERROR: Servo Off was not confirmed. Use the upstream physical power cut-off now." >&2
    snapshot "SAFE STOP NOT CONFIRMED"
    exit 1
fi

snapshot "AFTER SAFE STOP"

if [[ "$MODE" == --clear-fault ]]; then
    fault_state="$(timeout 3 rostopic echo -n 1 /elfin_ros_control/elfin/fault_state 2>&1 || true)"
    if grep -Fq 'data: True' <<<"$fault_state"; then
        echo
        echo "===== CLEARING FAULT ONCE, WITH SERVOS OFF ====="
        call_setbool /elfin_ros_control/elfin/clear_fault
        sleep 1
    else
        echo "Fault is already clear; no reset command was sent."
    fi
    snapshot "AFTER ONE FAULT-CLEAR ATTEMPT"
fi

echo
echo "Safe recovery stage finished. All brakes remain closed and Servo On was not requested."
echo "Do not retry Servo On while the arm is still pressing an obstacle."
echo "Only after external support/obstruction handling and a clear workspace, use the Panel at 1% for a short retreat."
