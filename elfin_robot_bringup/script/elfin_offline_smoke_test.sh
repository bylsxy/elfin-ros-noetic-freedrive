#!/usr/bin/env bash

# Build and parse the Elfin stack without opening EtherCAT or commanding hardware.

set -euo pipefail

ROS_SETUP=/opt/ros/noetic/setup.bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT=$(cd "$SCRIPT_DIR/../.." && pwd)
WORKSPACE=$(cd "$PROJECT/../.." && pwd)
URDF_OUT=/tmp/elfin5_offline_smoke_test.urdf

source "$ROS_SETUP"
if [[ -r "$WORKSPACE/devel/setup.bash" ]]; then
    source "$WORKSPACE/devel/setup.bash"
fi

echo "[1/6] Checking shell scripts..."
for shell_script in \
    "$PROJECT/elfin_robot_bringup/script/start_elfin5_freedrive_sim.sh" \
    "$PROJECT/elfin_robot_bringup/script/detect_elfin_ethercat_interface.sh" \
    "$PROJECT/elfin_robot_bringup/script/elfin_collision_recovery.sh" \
    "$PROJECT/elfin_robot_bringup/script/start_elfin5_panel.sh" \
    "$PROJECT/elfin_robot_bringup/script/start_elfin5_hardware.sh" \
    "$PROJECT/elfin_robot_bringup/script/elfin_software_stop.sh"; do
    bash -n "$shell_script"
done

echo "[2/6] Building all catkin packages..."
cd "$WORKSPACE"
# A previous narrow developer build can leave this CMake cache variable set.
# Clear it explicitly so this command really traverses the whole workspace.
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
source "$WORKSPACE/devel/setup.bash"

echo "[3/6] Running the freedrive unit tests..."
catkin_make run_tests_elfin_freedrive_controller
catkin_test_results "$WORKSPACE/build/test_results/elfin_freedrive_controller"

echo "[4/6] Expanding and parsing the Elfin5 URDF..."
xacro -o "$URDF_OUT" "$PROJECT/elfin_description/urdf/elfin5.urdf.xacro"
check_urdf "$URDF_OUT"

echo "[5/6] Checking Python syntax..."
export PYTHONPYCACHEPREFIX=/tmp/elfin_python_cache
while IFS= read -r -d '' python_file; do
    /usr/bin/python3 -m py_compile "$python_file"
done < <(find "$PROJECT" -type f -name '*.py' -print0)

echo "[6/6] Resolving key launch files without starting nodes..."
roslaunch --nodes elfin_robot_bringup elfin5_bringup.launch
roslaunch --nodes elfin_robot_bringup elfin_ros_control.launch elfin_ethernet_name:=offline_test
roslaunch --nodes elfin_robot_bringup elfin5_hardware_bringup.launch elfin_ethernet_name:=offline_test
roslaunch --nodes elfin_robot_bringup elfin5_control_panel.launch display:=false
roslaunch --nodes elfin_gazebo elfin5_empty_world.launch
roslaunch --nodes elfin5_moveit_config moveit_planning_execution.launch display:=false
roslaunch --nodes elfin5_moveit_config moveit_rviz.launch config:=true use_egl:=false
roslaunch --nodes elfin5_moveit_config moveit_rviz.launch config:=true use_egl:=true
roslaunch --nodes elfin5_moveit_config demo.launch
roslaunch --nodes elfin_basic_api elfin_basic_api.launch
roslaunch --nodes elfin_freedrive_controller elfin5_freedrive_sim.launch
roslaunch --nodes elfin_robot_bringup elfin5_freedrive_demo.launch

echo "PASS: offline Elfin build, URDF, Python, and launch checks completed."
echo "This result does not test EtherCAT, motors, end I/O, or real motion."
