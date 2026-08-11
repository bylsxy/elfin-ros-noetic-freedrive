#!/usr/bin/env bash

# Detect the E05 EtherCAT link, load the model, and start the Servo-Off driver.

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WORKSPACE=$(cd "$PROJECT_DIR/../.." && pwd)
RT_ROOT=/sys/fs/cgroup/cpu,cpuacct
RT_GROUP=$RT_ROOT/elfin-hardware
RT_RUNTIME_US=500000
RT_CPU=${ELFIN_RT_CPU:-14}
RT_MAX_LATENCY_US=${ELFIN_RT_MAX_LATENCY_US:-100}
RT_PREFLIGHT_SECONDS=${ELFIN_RT_PREFLIGHT_SECONDS:-5}
LOCK_FILE=/run/lock/elfin5-hardware.lock
PID_FILE=/run/elfin5-hardware.pid
FREEDRIVE_LOCKOUT_FILE=${ELFIN_FREEDRIVE_LOCKOUT_FILE:-/home/catas/.ros/ELFIN_FREEDRIVE_LOCKOUT}
GLOBAL_STOP_SCRIPT=${ELFIN_GLOBAL_STOP_SCRIPT:-/home/catas/ros_ws/src/elfin_vision/scripts/request_elfin_global_stop.sh}
ORIGINAL_ARGS=("$@")
ALLOW_FREEDRIVE=false
REPLACE_EXISTING=false
ROSLAUNCH_ARGS=()

hard_timeout() {
    timeout --signal=TERM --kill-after=1s "$@"
}

usage() {
    cat <<'EOF'
Usage: start_elfin5_hardware.sh [--replace-existing] [--freedrive] [roslaunch name:=value ...]

Without --freedrive, the gravity controller is loaded but real-hardware
freedrive remains locked. --freedrive opens that manager gate for this launch;
the controller still starts only after all runtime checks pass.

--replace-existing performs the bounded complete Elfin shutdown first. It
revokes automatic execution, exits FREE, requests and confirms Servo Off,
closes vision/Panel/MoveIt, and only then replaces the EtherCAT hardware stack.
EOF
}

while (( $# > 0 )); do
    case "$1" in
        --freedrive) ALLOW_FREEDRIVE=true ;;
        --replace-existing) REPLACE_EXISTING=true ;;
        -h|--help) usage; exit 0 ;;
        *) ROSLAUNCH_ARGS+=("$1") ;;
    esac
    shift
done

if (( EUID != 0 )); then
    exec sudo -E "$0" "${ORIGINAL_ARGS[@]}"
fi

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}
export ROS_IP=${ROS_IP:-127.0.0.1}
unset ROS_HOSTNAME
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

if ! command -v flock >/dev/null 2>&1; then
    echo "Required command is missing: flock" >&2
    exit 69
fi

# A hardware replacement owns the ROS master. Replacing only the EtherCAT
# child would strand Panel/MoveIt/vision against the old master and can leave
# an enabled arm behind. The explicit option therefore performs the existing
# ordered global stop and waits for the hardware launch lock to be released.
if [[ "$REPLACE_EXISTING" == true ]]; then
    existing_stack=false
    exec 8>"$LOCK_FILE"
    if flock -n 8; then
        flock -u 8
    else
        existing_stack=true
    fi
    if hard_timeout 2s rosnode list >/dev/null 2>&1; then
        existing_stack=true
    fi
    if [[ "$existing_stack" == true ]]; then
        echo "Replacing the existing Elfin stack with an ordered Servo-Off shutdown."
        if [[ ! -x "$GLOBAL_STOP_SCRIPT" ]]; then
            echo "Global stop helper is missing: $GLOBAL_STOP_SCRIPT" >&2
            exit 69
        fi
        if ! "$GLOBAL_STOP_SCRIPT" \
                "hardware --replace-existing requested a clean restart"; then
            echo "Existing Elfin stack did not stop cleanly; hardware restart aborted." >&2
            exit 77
        fi
        hardware_lock_released=false
        for _attempt in $(seq 1 100); do
            if flock -n 8; then
                flock -u 8
                hardware_lock_released=true
                break
            fi
            sleep 0.1
        done
        if [[ "$hardware_lock_released" != true ]]; then
            echo "The previous hardware launch lock was not released within 10 seconds." >&2
            exit 75
        fi
        echo "Previous Elfin stack stopped; starting a fresh Servo-Off hardware stack."
    else
        echo "No running Elfin stack needs replacement."
    fi
fi

# Clean dead ROS registrations left by a previously crashed launch.  This does
# not stop or unregister a node that still answers its XML-RPC health check.
if hard_timeout 3s rosnode list >/dev/null 2>&1; then
    cleanup_output="$(printf 'y\n' | hard_timeout 20s rosnode cleanup 2>&1 || true)"
    removed_nodes="$(grep '^Unregistering ' <<<"$cleanup_output" || true)"
    if [[ -n "$removed_nodes" ]]; then
        echo "Removed stale ROS node registrations:"
        sed 's/^Unregistering /  /' <<<"$removed_nodes"
    else
        echo "No stale ROS node registrations found."
    fi
fi

for command_name in chrt cyclictest flock taskset; do
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
echo "Preparing the Servo-Off hardware stack; only the hardware interface will use RR priority 10 on CPU $RT_CPU."
if [[ "$ALLOW_FREEDRIVE" == true ]]; then
    if [[ -e "$FREEDRIVE_LOCKOUT_FILE" || -L "$FREEDRIVE_LOCKOUT_FILE" ]]; then
        echo "Real-hardware FREE is locked after a recorded safety incident:" >&2
        if [[ -s "$FREEDRIVE_LOCKOUT_FILE" && -r "$FREEDRIVE_LOCKOUT_FILE" ]]; then
            sed 's/^/  /' "$FREEDRIVE_LOCKOUT_FILE" >&2
        else
            echo "  Lock file exists (content is empty or unreadable)." >&2
        fi
        echo "Normal position-mode bringup remains available without --freedrive." >&2
        exit 78
    fi
    echo "Freedrive manager gate: UNLOCKED for this launch (controller remains stopped)."
else
    echo "Freedrive manager gate: LOCKED (use --freedrive for a supervised trial)."
fi

cpu_online_file="/sys/devices/system/cpu/cpu$RT_CPU/online"
cpu_online=1
if [[ -r "$cpu_online_file" ]]; then
    cpu_online=$(<"$cpu_online_file")
fi
if ! [[ "$RT_CPU" =~ ^[0-9]+$ ]] ||
   [[ ! -d "/sys/devices/system/cpu/cpu$RT_CPU" ]] ||
   [[ "$cpu_online" != 1 ]]; then
    echo "Configured realtime CPU is unavailable: $RT_CPU" >&2
    exit 69
fi
if ! taskset -c "$RT_CPU" chrt --rr 10 true; then
    echo "Root cannot create an RR10 task pinned to CPU $RT_CPU." >&2
    exit 77
fi

echo "Running the bounded ${RT_PREFLIGHT_SECONDS}s/1ms realtime latency preflight..."
preflight_output=$(timeout --signal=TERM --kill-after=1s "$((RT_PREFLIGHT_SECONDS + 3))s" \
    cyclictest --policy=rr --priority=10 --mlockall \
    --interval=1000 --affinity="$RT_CPU" --threads=1 \
    --duration="${RT_PREFLIGHT_SECONDS}s" --quiet 2>&1) || {
        echo "$preflight_output" >&2
        echo "Realtime latency preflight failed." >&2
        exit 69
    }
printf '%s\n' "$preflight_output"
preflight_max=$(sed -n 's/.*Max:[[:space:]]*\([0-9][0-9]*\).*/\1/p' \
    <<<"$preflight_output" | tail -n 1)
if [[ -z "$preflight_max" || "$preflight_max" -gt "$RT_MAX_LATENCY_US" ]]; then
    echo "Realtime latency gate rejected max=${preflight_max:-unknown}us (limit ${RT_MAX_LATENCY_US}us)." >&2
    exit 69
fi

RT_CGROUP_ACTIVE=false
if [[ -e "$RT_ROOT/cpu.rt_runtime_us" && -w "$RT_ROOT/tasks" ]]; then
    mkdir -p "$RT_GROUP"
    if [[ -s "$RT_GROUP/tasks" ]]; then
        echo "The Elfin realtime cgroup is still occupied; run STOP_ELFIN.sh first." >&2
        exit 75
    fi
    printf '%s\n' "$RT_RUNTIME_US" > "$RT_GROUP/cpu.rt_runtime_us"
    printf '%s\n' "$$" > "$RT_GROUP/tasks"
    RT_CGROUP_ACTIVE=true
    echo "Realtime containment: dedicated cgroup plus CPU $RT_CPU affinity."
else
    echo "Realtime containment: CPU $RT_CPU affinity; roslaunch and watchdog remain SCHED_OTHER."
fi

ROSLAUNCH_PID=""

cleanup_runtime() {
    if [[ -n "$ROSLAUNCH_PID" ]] && kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
        kill -INT "$ROSLAUNCH_PID" 2>/dev/null || true
        wait "$ROSLAUNCH_PID" 2>/dev/null || true
    fi
    if [[ -r "$PID_FILE" ]] && [[ "$(<"$PID_FILE")" == "$ROSLAUNCH_PID" ]]; then
        rm -f "$PID_FILE"
    fi
    if [[ "$RT_CGROUP_ACTIVE" == true ]]; then
        printf '%s\n' "$$" > "$RT_ROOT/tasks" 2>/dev/null || true
        rmdir "$RT_GROUP" 2>/dev/null || true
    fi
}
trap cleanup_runtime EXIT

forward_signal() {
    local signal=$1
    if [[ -n "$ROSLAUNCH_PID" ]] && kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
        kill -"$signal" "$ROSLAUNCH_PID" 2>/dev/null || true
    fi
}

global_interrupt() {
    local reason=$1
    trap - INT HUP
    if [[ "${ELFIN_SUPERVISED_CHILD:-false}" == true ]]; then
        forward_signal INT
        echo "Hardware terminal received $reason; the parent supervisor is performing the complete shutdown."
    elif [[ -x "$GLOBAL_STOP_SCRIPT" ]]; then
        # Keep roslaunch (and therefore the ROS master and disable service)
        # alive until the ordered global stop has requested and confirmed
        # Servo Off.  Forwarding SIGINT first destroys that confirmation path.
        "$GLOBAL_STOP_SCRIPT" "hardware terminal received $reason" || \
            forward_signal INT
    else
        echo "Global stop helper is missing; only the hardware launch can be stopped." >&2
        forward_signal INT
    fi
    exit 130
}

trap 'global_interrupt Ctrl+C' INT
trap 'global_interrupt terminal-close' HUP
trap 'forward_signal TERM' TERM

roslaunch elfin_robot_bringup elfin5_hardware_bringup.launch \
    elfin_ethernet_name:="$interface" \
    allow_hardware_freedrive:="$ALLOW_FREEDRIVE" \
    hardware_launch_prefix:="taskset -c $RT_CPU chrt --rr 10" \
    "${ROSLAUNCH_ARGS[@]}" &
ROSLAUNCH_PID=$!
printf '%s\n' "$ROSLAUNCH_PID" > "$PID_FILE"

set +e
wait "$ROSLAUNCH_PID"
launch_status=$?
set -e
exit "$launch_status"
