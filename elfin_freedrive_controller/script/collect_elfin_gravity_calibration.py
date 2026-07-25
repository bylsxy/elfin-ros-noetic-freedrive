#!/usr/bin/python3
"""Collect passive E05 gravity samples through normal position control."""

import argparse
import math
import sys
import time

import moveit_commander
import rospy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


JOINT_NAMES = [f"elfin_joint{index}" for index in range(1, 7)]
# These poses keep the wrist roughly upright and stay inside the posture that
# was already reached under normal CSP control during the supervised trial.
CALIBRATION_PAIRS = [
    (0.22, -0.22),
    (0.34, -0.40),
    (0.47, -0.56),
    (0.61, -0.69),
    (0.47, -0.56),
    (0.34, -0.40),
    (0.22, -0.22),
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Validate or execute a low-speed, position-controlled E05 "
            "gravity calibration sequence. No CST/freedrive is used."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="execute after collision validation; otherwise validate only",
    )
    parser.add_argument(
        "--manager-namespace",
        default="/elfin_freedrive_manager",
        help="manager namespace providing record/fit services",
    )
    parser.add_argument("--velocity-scale", type=float, default=0.05)
    parser.add_argument("--acceleration-scale", type=float, default=0.05)
    parser.add_argument("--settle-seconds", type=float, default=1.2)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def ordered_joint_state(timeout=2.0):
    message = rospy.wait_for_message("/joint_states", JointState, timeout=timeout)
    positions = dict(zip(message.name, message.position))
    velocities = dict(zip(message.name, message.velocity))
    if any(name not in positions or name not in velocities for name in JOINT_NAMES):
        raise RuntimeError("/joint_states does not contain all six Elfin joints")
    return (
        [positions[name] for name in JOINT_NAMES],
        [velocities[name] for name in JOINT_NAMES],
    )


def require_driver_ready():
    enabled = rospy.wait_for_message(
        "/elfin_ros_control/elfin/enable_state", Bool, timeout=2.0
    ).data
    faulted = rospy.wait_for_message(
        "/elfin_ros_control/elfin/fault_state", Bool, timeout=2.0
    ).data
    if not enabled or faulted:
        raise RuntimeError(
            f"driver is not ready: servo_enabled={enabled}, faulted={faulted}"
        )


def state_request(values):
    request = GetStateValidityRequest()
    request.group_name = "elfin_arm"
    request.robot_state = RobotState()
    request.robot_state.joint_state.name = list(JOINT_NAMES)
    request.robot_state.joint_state.position = list(values)
    return request


def validate_interpolated_path(validity_service, start, goal, label):
    maximum_delta = max(abs(goal[i] - start[i]) for i in range(6))
    steps = max(2, int(math.ceil(maximum_delta / 0.02)))
    for step in range(steps + 1):
        progress = float(step) / float(steps)
        values = [
            start[i] + progress * (goal[i] - start[i]) for i in range(6)
        ]
        result = validity_service(state_request(values))
        if not result.valid:
            contacts = ", ".join(
                sorted(
                    {
                        f"{contact.contact_body_1}<->{contact.contact_body_2}"
                        for contact in result.contacts
                    }
                )
            )
            raise RuntimeError(
                f"{label} fails MoveIt collision validation at "
                f"{progress:.2f}: {contacts or 'invalid robot state'}"
            )


def extract_plan(plan_result):
    if isinstance(plan_result, tuple):
        success, trajectory = bool(plan_result[0]), plan_result[1]
    else:
        trajectory = plan_result
        success = bool(trajectory.joint_trajectory.points)
    if not success or not trajectory.joint_trajectory.points:
        raise RuntimeError("MoveIt did not produce a non-empty trajectory")
    return trajectory


def wait_until_static(timeout=8.0):
    deadline = time.monotonic() + timeout
    stable_since = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        _, velocity = ordered_joint_state(timeout=1.0)
        if max(abs(value) for value in velocity) <= 0.003:
            if stable_since is None:
                stable_since = time.monotonic()
            if time.monotonic() - stable_since >= 0.5:
                return
        else:
            stable_since = None
        rospy.sleep(0.05)
    raise RuntimeError("robot did not become statically settled within 8 seconds")


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("collect_elfin_gravity_calibration", anonymous=True)
    args = parse_args()
    if not (0.01 <= args.velocity_scale <= 0.10):
        raise RuntimeError("velocity scale must stay in [0.01, 0.10]")
    if not (0.01 <= args.acceleration_scale <= 0.10):
        raise RuntimeError("acceleration scale must stay in [0.01, 0.10]")
    if args.settle_seconds < 0.8:
        raise RuntimeError("settle time must be at least 0.8 seconds")

    require_driver_ready()
    start, velocity = ordered_joint_state()
    if max(abs(value) for value in velocity) > 0.003:
        raise RuntimeError("robot must be static before calibration")
    if not (-0.15 <= start[1] <= 0.25 and -0.25 <= start[2] <= 0.15):
        raise RuntimeError(
            "calibration must start near upright Home: "
            f"current J2={start[1]:.3f}, J3={start[2]:.3f} rad"
        )
    if max(abs(start[index]) for index in (0, 3, 4, 5)) > 0.35:
        raise RuntimeError("J1/J4/J5/J6 must be near Home before calibration")

    targets = []
    for joint2, joint3 in CALIBRATION_PAIRS:
        target = list(start)
        target[1] = joint2
        target[2] = joint3
        targets.append(target)
    targets.append(list(start))

    rospy.wait_for_service("/check_state_validity", timeout=5.0)
    validity = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
    previous = list(start)
    for index, target in enumerate(targets, 1):
        validate_interpolated_path(validity, previous, target, f"segment {index}")
        previous = target
    print(f"PASS: {len(targets)} position-controlled segments are collision-free")
    if not args.execute:
        print("Validation only; no trajectory or calibration service was sent")
        return 0

    namespace = args.manager_namespace.rstrip("/")
    record_name = namespace + "/record_gravity_sample"
    fit_name = namespace + "/fit_gravity_calibration"
    rospy.wait_for_service(record_name, timeout=5.0)
    rospy.wait_for_service(fit_name, timeout=5.0)
    record_sample = rospy.ServiceProxy(record_name, Trigger)
    fit_calibration = rospy.ServiceProxy(fit_name, Trigger)

    group = moveit_commander.MoveGroupCommander("elfin_arm")
    group.set_planning_time(5.0)
    group.set_num_planning_attempts(5)
    group.allow_replanning(False)
    group.set_max_velocity_scaling_factor(args.velocity_scale)
    group.set_max_acceleration_scaling_factor(args.acceleration_scale)
    group.set_goal_joint_tolerance(0.003)

    for index, target in enumerate(targets, 1):
        require_driver_ready()
        group.set_start_state_to_current_state()
        group.set_joint_value_target(dict(zip(JOINT_NAMES, target)))
        trajectory = extract_plan(group.plan())
        duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        print(
            f"EXECUTE {index}/{len(targets)}: J2={target[1]:.3f}, "
            f"J3={target[2]:.3f}, planned_duration={duration:.2f}s"
        )
        if not group.execute(trajectory, wait=True):
            group.stop()
            raise RuntimeError(f"MoveIt execution failed at segment {index}")
        group.stop()
        wait_until_static()
        rospy.sleep(args.settle_seconds)
        result = record_sample()
        if not result.success:
            raise RuntimeError(
                f"sample {index} was rejected after motion: {result.message}"
            )
        print(f"SAMPLE {index}: {result.message}")

    result = fit_calibration()
    if not result.success:
        raise RuntimeError("fit rejected: " + result.message)
    print("FIT PASS: " + result.message)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(1)
    finally:
        moveit_commander.roscpp_shutdown()
