#!/usr/bin/env bash

# Start the complete freedrive demo on loopback-only ROS/Gazebo masters.

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
WORKSPACE=$(cd "$PROJECT_DIR/../.." && pwd)
ROS_MASTER_URI=http://127.0.0.1:11312
GAZEBO_MASTER_URI=http://127.0.0.1:11346
GAZEBO_GUI=false
RVIZ=false
RVIZ_EGL=false

usage() {
    cat <<'EOF'
Usage: start_elfin5_freedrive_sim.sh [--gazebo-gui] [--rviz|--rviz-egl]

Starts the Elfin5 effort simulation, MoveIt, Basic API and the Chinese Panel.
It uses loopback-only ports and contains no EtherCAT hardware node.

  --gazebo-gui  Also open the Gazebo 3D window (higher GPU load).
  --rviz        Also open the low-load RViz configuration.
  --rviz-egl    Open low-load RViz through the Qt EGL fallback.
EOF
}

while (( $# > 0 )); do
    case "$1" in
        --gazebo-gui) GAZEBO_GUI=true ;;
        --rviz) RVIZ=true ;;
        --rviz-egl) RVIZ=true; RVIZ_EGL=true ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 64 ;;
    esac
    shift
done

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

export ROS_MASTER_URI GAZEBO_MASTER_URI
export GZ_IP=127.0.0.1
unset ROS_IP ROS_HOSTNAME

if timeout 2 rosnode list >/dev/null 2>&1; then
    echo "The isolated simulation master at $ROS_MASTER_URI is already running." >&2
    echo "Return to its terminal and press Ctrl+C before starting another demo." >&2
    exit 75
fi

echo "Starting the loopback-only Elfin freedrive demo. No EtherCAT node is included."
echo "ROS master: $ROS_MASTER_URI"
echo "Gazebo master: $GAZEBO_MASTER_URI"
echo "Close the complete demo with Ctrl+C in this terminal."

exec roslaunch elfin_robot_bringup elfin5_freedrive_demo.launch \
    gazebo_gui:="$GAZEBO_GUI" \
    rviz:="$RVIZ" \
    rviz_egl:="$RVIZ_EGL"
