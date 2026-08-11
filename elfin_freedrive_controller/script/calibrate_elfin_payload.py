#!/usr/bin/python3
"""Supervised automatic rigid-payload calibration for the Elfin E05."""

import argparse
import csv
import datetime
import math
import os
import queue
import signal
import sys
import threading
import time

import moveit_commander
import numpy as np
import rospy
import yaml
from elfin_freedrive_controller.srv import (
    EvaluatePayloadModel,
    EvaluatePayloadModelRequest,
    GetPayloadModel,
    GetPayloadModelRequest,
    SetDampingScales,
    SetDampingScalesRequest,
    SetPayloadModel,
    SetPayloadModelRequest,
)
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import (
    GetPositionFK,
    GetPositionFKRequest,
    GetStateValidity,
    GetStateValidityRequest,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, SetBoolRequest


JOINT_NAMES = ["elfin_joint{}".format(index) for index in range(1, 7)]
END_LINK = "elfin_end_link"
BASE_LINK = "elfin_base"
APPROACH_DELTA = np.asarray([0.0, 0.04, -0.04, 0.06, -0.06, 0.06])
MINIMUM_FLANGE_HEIGHT = 0.65
MAXIMUM_PAYLOAD_MASS = 5.0
NOMINAL_PAYLOAD_CENTER_OF_MASS_RADIUS = 0.60
FIT_POSES = (
    ("拟合 A", [0.0, 0.28, -0.36, 0.0, 0.0, 0.0]),
    ("拟合 B", [0.0, 0.40, -0.50, 0.0, 0.35, 0.0]),
    ("拟合 C", [0.0, 0.40, -0.50, 0.0, -0.35, 0.0]),
    ("拟合 D", [0.0, 0.52, -0.62, 0.35, 0.30, -0.25]),
    ("拟合 E", [0.0, 0.52, -0.62, -0.35, -0.30, 0.25]),
    ("拟合 F", [0.0, 0.32, -0.44, 0.50, 0.42, -0.45]),
)
VALIDATION_POSES = (
    ("留出 G", [0.0, 0.32, -0.44, -0.50, -0.42, 0.45]),
    ("留出 H", [0.0, 0.46, -0.56, 0.25, -0.18, 0.30]),
)
ALL_POSES = FIT_POSES + VALIDATION_POSES
CANCEL_REQUESTED = threading.Event()


class CalibrationCancelled(RuntimeError):
    pass


def log(message):
    print(message, flush=True)


def request_cancel(signum, frame):
    del signum, frame
    if not CANCEL_REQUESTED.is_set():
        log("[中止] 已收到请求，正在停止轨迹并执行安全恢复，请勿关闭终端")
    CANCEL_REQUESTED.set()


def raise_if_cancelled():
    if CANCEL_REQUESTED.is_set():
        raise CalibrationCancelled("操作者请求中止自动负载标定")


def strict_preflight_outcome(publication_count, message):
    """Classify only live publications after the subscriber's latched sample."""
    if publication_count <= 1:
        return None
    if message.startswith("通过（位置模式静止预检）"):
        return True
    if message.startswith("未通过"):
        return False
    return None


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Validate or execute supervised automatic Elfin E05 rigid-payload "
            "calibration. The identification motion stays in position control."
        )
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--execute",
        action="store_true",
        help="execute the validated trajectory; otherwise perform read-only validation",
    )
    mode.add_argument(
        "--analyze-samples",
        metavar="CSV",
        help=(
            "recompute a measurement report from a completed static_pairs.csv; "
            "this only queries the gravity model and never commands motion"
        ),
    )
    mode.add_argument(
        "--resume-samples",
        metavar="CSV",
        help=(
            "reuse one completed static_pairs.csv and perform only the current-"
            "pose checks plus the <=1 s controlled hold; requires Panel confirmation"
        ),
    )
    parser.add_argument(
        "--confirmed-by-panel",
        action="store_true",
        help="confirmation token supplied after the Panel safety dialog",
    )
    parser.add_argument("--velocity-scale", type=float, default=0.03)
    parser.add_argument("--acceleration-scale", type=float, default=0.03)
    parser.add_argument("--settle-seconds", type=float, default=1.0)
    parser.add_argument("--sample-seconds", type=float, default=0.8)
    parser.add_argument("--validation-hold-seconds", type=float, default=0.8)
    parser.add_argument(
        "--manager-namespace",
        default="/elfin_freedrive_manager",
        help="freedrive manager namespace; mainly useful for isolated validation",
    )
    parser.add_argument(
        "--output-root",
        default=os.path.expanduser("~/.ros/elfin_payload_calibration_runs"),
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def extract_plan(result):
    if isinstance(result, tuple):
        success, trajectory = bool(result[0]), result[1]
    else:
        trajectory = result
        success = bool(trajectory.joint_trajectory.points)
    if not success or not trajectory.joint_trajectory.points:
        raise RuntimeError("MoveIt 没有生成有效轨迹")
    return trajectory


def robust_weighted_fit(matrix, values, sigma, iterations=8):
    """Return a Huber-IRLS fit and diagnostics for [m, mx, my, mz]."""
    matrix = np.asarray(matrix, dtype=float)
    values = np.asarray(values, dtype=float)
    sigma = np.maximum(np.asarray(sigma, dtype=float), 0.15)
    if matrix.ndim != 2 or matrix.shape[1] != 4:
        raise ValueError("payload matrix must have four columns")
    if values.shape != (matrix.shape[0],) or sigma.shape != values.shape:
        raise ValueError("payload fit vector dimensions do not match")
    weights = 1.0 / np.square(sigma)
    solution = np.zeros(4)
    for _ in range(iterations):
        root_weight = np.sqrt(weights)
        weighted_matrix = matrix * root_weight[:, None]
        weighted_values = values * root_weight
        solution, _, rank, singular = np.linalg.lstsq(
            weighted_matrix, weighted_values, rcond=None
        )
        if rank != 4 or singular[-1] <= 1e-9:
            raise RuntimeError("负载回归矩阵秩不足，无法同时辨识质量和三维重心")
        residual = values - matrix.dot(solution)
        median = float(np.median(residual))
        robust_scale = max(
            0.20, 1.4826 * float(np.median(np.abs(residual - median)))
        )
        normalized = np.abs(residual) / (1.5 * robust_scale)
        huber = np.ones_like(normalized)
        outside = normalized > 1.0
        huber[outside] = 1.0 / normalized[outside]
        weights = huber / np.square(sigma)
    singular = np.linalg.svd(matrix * np.sqrt(weights)[:, None], compute_uv=False)
    condition = float(singular[0] / singular[-1])
    residual = values - matrix.dot(solution)
    return solution, residual, condition, weights


def normalize_payload_solution(solution, matrix):
    del matrix
    solution = np.asarray(solution, dtype=float).copy()
    mass = float(solution[0])
    if mass <= 0.0:
        return np.zeros(4), 0.0, np.zeros(3)
    center = solution[1:] / mass
    return solution, mass, center


def validate_static_window(label, positions, velocities, efforts):
    positions = np.asarray(positions, dtype=float)
    velocities = np.asarray(velocities, dtype=float)
    efforts = np.asarray(efforts, dtype=float)
    reported_speed_peak = float(np.max(np.abs(velocities)))
    position_span = float(np.max(np.ptp(positions, axis=0)))
    effort_std = np.std(efforts, axis=0)
    if reported_speed_peak > 0.004:
        log(
            "[采样提示] {} 驱动速度字段峰值 {:.5f} rad/s，"
            "同期编码器位置跨度 {:.5f} rad；速度峰值仅记录，"
            "静止性按编码器实际跨度判断".format(
                label, reported_speed_peak, position_span
            )
        )
    if position_span > 0.002:
        raise RuntimeError(
            "{} 采样期间编码器实际位置跨度 {:.5f} rad，机械臂未保持静止".format(
                label, position_span
            )
        )
    if float(np.max(effort_std[1:])) > 2.5:
        raise RuntimeError(
            "{} 力矩波动过大：最大标准差 {:.3f} Nm".format(
                label, float(np.max(effort_std[1:]))
            )
        )
    return reported_speed_peak, position_span, effort_std


class RobotMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._joint = None
        self._joint_received = 0.0
        self._servo = None
        self._servo_received = 0.0
        self._fault = None
        self._fault_received = 0.0
        rospy.Subscriber("/joint_states", JointState, self._joint_callback, queue_size=20)
        rospy.Subscriber(
            "/elfin_ros_control/elfin/enable_state",
            Bool,
            self._servo_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/elfin_ros_control/elfin/fault_state",
            Bool,
            self._fault_callback,
            queue_size=5,
        )

    def _joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        efforts = dict(zip(message.name, message.effort))
        if any(
            name not in positions or name not in velocities or name not in efforts
            for name in JOINT_NAMES
        ):
            return
        sample = (
            np.asarray([positions[name] for name in JOINT_NAMES], dtype=float),
            np.asarray([velocities[name] for name in JOINT_NAMES], dtype=float),
            np.asarray([efforts[name] for name in JOINT_NAMES], dtype=float),
        )
        if not all(np.all(np.isfinite(values)) for values in sample):
            return
        with self._lock:
            self._joint = sample
            self._joint_received = time.monotonic()

    def _servo_callback(self, message):
        with self._lock:
            self._servo = bool(message.data)
            self._servo_received = time.monotonic()

    def _fault_callback(self, message):
        with self._lock:
            self._fault = bool(message.data)
            self._fault_received = time.monotonic()

    def snapshot(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                if self._joint is not None and time.monotonic() - self._joint_received < 0.35:
                    return tuple(values.copy() for values in self._joint)
            rospy.sleep(0.02)
        raise RuntimeError("六轴 joint_states 缺失、不完整或已过期")

    def require_driver_ready(self):
        self.snapshot()
        with self._lock:
            servo, fault = self._servo, self._fault
            servo_age = time.monotonic() - self._servo_received
            fault_age = time.monotonic() - self._fault_received
        if servo is None or fault is None:
            raise RuntimeError("尚未收到 Servo/Fault 状态")
        if servo_age >= 0.75 or fault_age >= 0.75:
            raise RuntimeError(
                "Servo/Fault 状态已过期：{:.3f}/{:.3f} s（门限 0.750 s）".format(
                    servo_age, fault_age
                )
            )
        if not servo or fault:
            raise RuntimeError(
                "驱动未就绪：Servo On={}，Fault={}".format(servo, fault)
            )

    def wait_for_initial_driver_state(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        last_error = "尚未收到 Servo/Fault 状态"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                self.require_driver_ready()
                return
            except RuntimeError as error:
                last_error = str(error)
                if not last_error.startswith("尚未收到"):
                    raise
            rospy.sleep(0.02)
        raise RuntimeError(
            "等待首帧驱动状态超时（{:.1f} s）：{}".format(timeout, last_error)
        )


class PayloadCalibrator:
    def __init__(self, args):
        self.args = args
        self.manager_namespace = "/" + args.manager_namespace.strip("/")
        self.analysis_only = bool(args.analyze_samples)
        self.monitor = None
        self.group = None
        self.validity = None
        self.fk = None
        if not self.analysis_only:
            self.monitor = RobotMonitor()
            self.group = moveit_commander.MoveGroupCommander("elfin_arm")
            self.group.set_planning_time(8.0)
            self.group.set_num_planning_attempts(8)
            self.group.allow_replanning(False)
            self.group.set_max_velocity_scaling_factor(args.velocity_scale)
            self.group.set_max_acceleration_scaling_factor(args.acceleration_scale)
            self.group.set_goal_joint_tolerance(0.003)
            self.validity = rospy.ServiceProxy(
                "/check_state_validity", GetStateValidity
            )
            self.fk = rospy.ServiceProxy("/compute_fk", GetPositionFK)
        self.evaluate = rospy.ServiceProxy(
            self.manager_namespace + "/evaluate_payload_model", EvaluatePayloadModel
        )
        self.get_payload = rospy.ServiceProxy(
            self.manager_namespace + "/get_payload_model", GetPayloadModel
        )
        self.set_payload = rospy.ServiceProxy(
            self.manager_namespace + "/set_payload_model", SetPayloadModel
        )
        self.set_freedrive = rospy.ServiceProxy(
            self.manager_namespace + "/set_freedrive", SetBool
        )
        self.set_damping = rospy.ServiceProxy(
            self.manager_namespace + "/set_damping_scales", SetDampingScales
        )
        self.output_dir = None
        self.previous_payload = None
        self.candidate_staged = False
        self.current_damping = None
        self.effort_limits = None
        self.maximum_gravity_fraction = None

    def wait_for_services(self):
        if self.analysis_only:
            names = (self.manager_namespace + "/evaluate_payload_model",)
        else:
            names = (
                "/check_state_validity",
                "/compute_fk",
                self.manager_namespace + "/evaluate_payload_model",
                self.manager_namespace + "/get_payload_model",
                self.manager_namespace + "/set_payload_model",
            )
        if self.args.execute and not self.analysis_only:
            names += (
                self.manager_namespace + "/set_freedrive",
                self.manager_namespace + "/set_damping_scales",
            )
        for name in names:
            raise_if_cancelled()
            rospy.wait_for_service(name, timeout=5.0)

    def state_request(self, values):
        request = GetStateValidityRequest()
        request.group_name = "elfin_arm"
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState(
            name=list(JOINT_NAMES), position=list(values)
        )
        return request

    def flange_height(self, values):
        request = GetPositionFKRequest()
        request.header.frame_id = BASE_LINK
        request.fk_link_names = [END_LINK]
        request.robot_state.joint_state = JointState(
            name=list(JOINT_NAMES), position=list(values)
        )
        result = self.fk(request)
        if result.error_code.val != result.error_code.SUCCESS or not result.pose_stamped:
            raise RuntimeError("MoveIt 无法计算 {} 的法兰高度".format(values))
        return float(result.pose_stamped[0].pose.position.z)

    def validate_state(
        self, values, label, minimum_height=MINIMUM_FLANGE_HEIGHT
    ):
        result = self.validity(self.state_request(values))
        if not result.valid:
            contacts = sorted(
                {
                    "{}<->{}".format(contact.contact_body_1, contact.contact_body_2)
                    for contact in result.contacts
                }
            )
            raise RuntimeError(
                "{} 未通过 MoveIt 碰撞检查：{}".format(
                    label, ", ".join(contacts) or "无效机器人状态"
                )
            )
        height = self.flange_height(values)
        if height < minimum_height:
            raise RuntimeError(
                "{} 法兰高度 {:.3f} m，低于上半工作区门限 {:.3f} m".format(
                    label, height, minimum_height
                )
            )
        return height

    def validate_segment(self, start, goal, label):
        maximum_delta = float(np.max(np.abs(np.asarray(goal) - np.asarray(start))))
        steps = max(2, int(math.ceil(maximum_delta / 0.025)))
        minimum_height = float("inf")
        for step in range(steps + 1):
            raise_if_cancelled()
            progress = float(step) / float(steps)
            values = np.asarray(start) + progress * (
                np.asarray(goal) - np.asarray(start)
            )
            minimum_height = min(
                minimum_height,
                self.validate_state(values, "{} @ {:.0f}%".format(label, 100 * progress)),
            )
        return minimum_height

    @staticmethod
    def calibration_goals():
        goals = []
        for name, target_values in ALL_POSES:
            target = np.asarray(target_values, dtype=float)
            goals.extend(
                (
                    (name + " 负向预靠近", target - APPROACH_DELTA),
                    (name + " 第一次到达", target),
                    (name + " 正向预靠近", target + APPROACH_DELTA),
                    (name + " 第二次到达", target),
                )
            )
        return goals

    def validate_complete_sequence(self, start):
        previous = np.asarray(start, dtype=float)
        minimum_height = self.validate_state(previous, "当前起点")
        for label, goal in self.calibration_goals():
            minimum_height = min(
                minimum_height, self.validate_segment(previous, goal, label)
            )
            previous = np.asarray(goal, dtype=float)
        minimum_height = min(
            minimum_height,
            self.validate_segment(previous, start, "标定结束返回原姿态"),
        )
        return minimum_height

    def evaluate_model(self, values):
        response = self.evaluate(
            EvaluatePayloadModelRequest(joint_positions=list(values))
        )
        if not response.success:
            raise RuntimeError("重力模型查询失败：" + response.message)
        self.effort_limits = np.asarray(response.effort_limits, dtype=float)
        self.maximum_gravity_fraction = float(
            response.maximum_gravity_effort_fraction
        )
        return (
            np.asarray(response.base_effort, dtype=float),
            np.asarray(response.payload_regressor, dtype=float).reshape(6, 4),
        )

    def validate_planned_trajectory(
        self, trajectory, label, minimum_height=MINIMUM_FLANGE_HEIGHT
    ):
        for index, point in enumerate(trajectory.joint_trajectory.points):
            self.validate_state(
                point.positions,
                "{} 轨迹点 {}".format(label, index + 1),
                minimum_height=minimum_height,
            )

    def check_driver_guard(self):
        raise_if_cancelled()
        self.monitor.require_driver_ready()
        return self.monitor.snapshot()

    def move_to(self, target, label, minimum_height=MINIMUM_FLANGE_HEIGHT):
        _, _, initial_effort = self.check_driver_guard()
        peak_effort = np.abs(initial_effort)
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(dict(zip(JOINT_NAMES, target)))
        trajectory = extract_plan(self.group.plan())
        raise_if_cancelled()
        self.validate_planned_trajectory(
            trajectory, label, minimum_height=minimum_height
        )
        duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        log(
            "[运动] {}，计划 {:.2f} s，速度/加速度倍率 {:.0f}%/{:.0f}%".format(
                label,
                duration,
                self.args.velocity_scale * 100,
                self.args.acceleration_scale * 100,
            )
        )
        if not self.group.execute(trajectory, wait=False):
            raise RuntimeError(label + " 的 MoveIt 执行请求被拒绝")
        deadline = time.monotonic() + max(6.0, duration * 2.5 + 3.0)
        stable_since = None
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                position, velocity, effort = self.check_driver_guard()
                peak_effort = np.maximum(peak_effort, np.abs(effort))
                error = float(np.max(np.abs(position - np.asarray(target))))
                speed = float(np.max(np.abs(velocity)))
                if error <= 0.004 and speed <= 0.004:
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= 0.30:
                        self.group.stop()
                        peak_joint = int(np.argmax(peak_effort))
                        log(
                            "[运动] {} 已稳定到达；位置模式反馈峰值 J{}={:.2f} Nm（仅记录）".format(
                                label, peak_joint + 1, peak_effort[peak_joint]
                            )
                        )
                        return
                else:
                    stable_since = None
                rospy.sleep(0.02)
        except Exception:
            self.group.stop()
            raise
        self.group.stop()
        raise RuntimeError(label + " 未在超时前稳定到目标姿态")

    def collect_static_sample(self, label):
        raise_if_cancelled()
        rospy.sleep(self.args.settle_seconds)
        raise_if_cancelled()
        positions, velocities, efforts = [], [], []
        deadline = time.monotonic() + self.args.sample_seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            raise_if_cancelled()
            position, velocity, effort = self.check_driver_guard()
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)
            rospy.sleep(0.02)
        if len(positions) < 20:
            raise RuntimeError(label + " 静态样本不足")
        positions = np.asarray(positions)
        velocities = np.asarray(velocities)
        efforts = np.asarray(efforts)
        reported_speed_peak, position_span, effort_std = validate_static_window(
            label, positions, velocities, efforts
        )
        return {
            "label": label,
            "position": np.mean(positions, axis=0),
            "effort": np.mean(efforts, axis=0),
            "effort_std": effort_std,
            "reported_speed_peak": reported_speed_peak,
            "position_span": position_span,
            "count": len(positions),
        }

    def collect_pose_pair(self, name, target):
        target = np.asarray(target, dtype=float)
        self.move_to(target - APPROACH_DELTA, name + " 负向预靠近")
        self.move_to(target, name + " 第一次到达")
        first = self.collect_static_sample(name + " 第一次静态样本")
        self.move_to(target + APPROACH_DELTA, name + " 正向预靠近")
        self.move_to(target, name + " 第二次到达")
        second = self.collect_static_sample(name + " 第二次静态样本")
        return self.build_pose_pair(name, first, second, announce=True)

    def build_pose_pair(self, name, first, second, announce=False):
        position_delta = float(
            np.max(np.abs(first["position"] - second["position"]))
        )
        centered_position = 0.5 * (first["position"] + second["position"])
        centered_effort = 0.5 * (first["effort"] + second["effort"])
        first_base, first_regressor = self.evaluate_model(first["position"])
        second_base, second_regressor = self.evaluate_model(second["position"])
        base = 0.5 * (first_base + second_base)
        regressor = 0.5 * (first_regressor + second_regressor)
        sigma = (
            0.20
            + np.maximum(first["effort_std"], second["effort_std"])
            + 0.05 * np.abs(first["effort"] - second["effort"])
        )
        if announce:
            log(
                "[采样] {} 完成；双向位置差 {:.5f} rad（仅记录），"
                "J2/J3 力矩差 {:.2f}/{:.2f} Nm".format(
                    name,
                    position_delta,
                    abs(first["effort"][1] - second["effort"][1]),
                    abs(first["effort"][2] - second["effort"][2]),
                )
            )
        return {
            "name": name,
            "first": first,
            "second": second,
            "position": centered_position,
            "effort": centered_effort,
            "base": base,
            "regressor": regressor,
            "sigma": sigma,
        }

    def read_raw_samples(self, raw_path):
        raw_path = os.path.realpath(os.path.expanduser(raw_path))
        if not os.path.isfile(raw_path):
            raise RuntimeError("找不到原始样本文件：" + raw_path)
        grouped = {}
        expected_names = {name for name, _ in ALL_POSES}
        with open(raw_path, newline="") as source:
            reader = csv.DictReader(source)
            required = (
                ["pose", "approach"]
                + JOINT_NAMES
                + ["effort_{}".format(name) for name in JOINT_NAMES]
                + ["stddev_{}".format(name) for name in JOINT_NAMES]
            )
            missing_columns = [
                name for name in required if name not in (reader.fieldnames or [])
            ]
            if missing_columns:
                raise RuntimeError(
                    "原始样本缺少字段：" + ", ".join(missing_columns)
                )
            for line_number, row in enumerate(reader, start=2):
                pose = row["pose"]
                approach = row["approach"]
                if pose not in expected_names or approach not in ("first", "second"):
                    raise RuntimeError(
                        "原始样本第 {} 行姿态/方向无效：{}/{}".format(
                            line_number, pose, approach
                        )
                    )
                if approach in grouped.setdefault(pose, {}):
                    raise RuntimeError(
                        "原始样本重复：{} {}".format(pose, approach)
                    )
                try:
                    sample = {
                        "position": np.asarray(
                            [float(row[name]) for name in JOINT_NAMES], dtype=float
                        ),
                        "effort": np.asarray(
                            [
                                float(row["effort_{}".format(name)])
                                for name in JOINT_NAMES
                            ],
                            dtype=float,
                        ),
                        "effort_std": np.asarray(
                            [
                                float(row["stddev_{}".format(name)])
                                for name in JOINT_NAMES
                            ],
                            dtype=float,
                        ),
                        "reported_speed_peak": float(
                            row.get("reported_speed_peak") or 0.0
                        ),
                        "position_span": float(row.get("position_span") or 0.0),
                    }
                except (TypeError, ValueError) as error:
                    raise RuntimeError(
                        "原始样本第 {} 行包含无效数字：{}".format(
                            line_number, error
                        )
                    )
                if not all(
                    np.all(np.isfinite(values))
                    for values in (
                        sample["position"],
                        sample["effort"],
                        sample["effort_std"],
                    )
                ) or np.any(sample["effort_std"] < 0.0):
                    raise RuntimeError(
                        "原始样本第 {} 行包含非有限数字或负标准差".format(
                            line_number
                        )
                    )
                grouped[pose][approach] = sample
        pairs = []
        for name, _ in ALL_POSES:
            samples = grouped.get(name, {})
            missing = [
                approach
                for approach in ("first", "second")
                if approach not in samples
            ]
            if missing:
                raise RuntimeError(
                    "原始样本不完整：{} 缺少 {}".format(name, "/".join(missing))
                )
            pairs.append(
                self.build_pose_pair(
                    name, samples["first"], samples["second"], announce=False
                )
            )
        return raw_path, pairs

    @staticmethod
    def rows_from_pairs(pairs):
        matrix, values, sigma = [], [], []
        for pair in pairs:
            residual = pair["effort"] - pair["base"]
            for joint in range(1, 6):
                row = pair["regressor"][joint]
                if float(np.linalg.norm(row)) < 0.05:
                    continue
                matrix.append(row)
                values.append(residual[joint])
                sigma.append(pair["sigma"][joint])
        return np.asarray(matrix), np.asarray(values), np.asarray(sigma)

    def fit_payload(self, fit_pairs, validation_pairs):
        fit_matrix, fit_values, fit_sigma = self.rows_from_pairs(fit_pairs)
        solution, _, condition, _ = robust_weighted_fit(
            fit_matrix, fit_values, fit_sigma
        )
        unconstrained_mass = float(solution[0])
        solution, mass, center = normalize_payload_solution(solution, fit_matrix)
        if unconstrained_mass <= 0.0:
            log(
                "[辨识] 无约束解质量 {:.3f} kg 不满足刚性负载物理约束；"
                "候选投影为空负载边界，并继续接受全部拟合/留出残差验证".format(
                    unconstrained_mass
                )
            )
        center_radius = float(np.linalg.norm(center))
        fit_residual = fit_values - fit_matrix.dot(solution)
        fit_rmse = float(np.sqrt(np.mean(np.square(fit_residual))))
        fit_max = float(np.max(np.abs(fit_residual)))
        validation_matrix, validation_values, _ = self.rows_from_pairs(
            validation_pairs
        )
        validation_residual = validation_values - validation_matrix.dot(solution)
        validation_rmse = float(
            np.sqrt(np.mean(np.square(validation_residual)))
        )
        validation_max = float(np.max(np.abs(validation_residual)))
        return {
            "solution": solution,
            "unconstrained_mass": unconstrained_mass,
            "mass": mass,
            "center": center,
            "center_radius": center_radius,
            "condition": condition,
            "fit_rmse": fit_rmse,
            "fit_max": fit_max,
            "validation_rmse": validation_rmse,
            "validation_max": validation_max,
        }

    @staticmethod
    def measurement_quality_rejections(model):
        reasons = []
        if model["condition"] > 30.0:
            reasons.append(
                "负载辨识矩阵条件数 {:.1f} 过大".format(model["condition"])
            )
        if model["fit_rmse"] > 2.5 or model["fit_max"] > 6.0:
            reasons.append(
                "拟合残差过大：RMSE {:.2f} Nm，最大 {:.2f} Nm".format(
                    model["fit_rmse"], model["fit_max"]
                )
            )
        if model["validation_rmse"] > 2.5 or model["validation_max"] > 5.0:
            reasons.append(
                "留出姿态未通过：RMSE {:.2f} Nm，最大 {:.2f} Nm；"
                "末端可能松动、线缆受力随姿态变化或负载不是刚体".format(
                    model["validation_rmse"], model["validation_max"]
                )
            )
        return reasons

    @staticmethod
    def control_rejections(model, capacity):
        reasons = []
        if model["mass"] > MAXIMUM_PAYLOAD_MASS + 1e-6:
            reasons.append(
                "辨识质量 {:.3f} kg 超过 E05 铭牌额定负载 {:.1f} kg".format(
                    model["mass"], MAXIMUM_PAYLOAD_MASS
                )
            )
        if capacity["worst_ratio"] > capacity["allowed_ratio"]:
            reasons.append(
                "候选模型在已检查路径的 J{} 需要 {:.1f}% 力矩容量，"
                "自动标定只允许 {:.1f}%".format(
                    capacity["worst_joint"],
                    100 * capacity["worst_ratio"],
                    100 * capacity["allowed_ratio"],
                )
            )
        return reasons

    def evaluate_capacity(self, model, start=None):
        solution = model["solution"]
        poses = [np.asarray(goal, dtype=float) for _, goal in self.calibration_goals()]
        includes_start = start is not None
        if includes_start:
            poses.insert(0, np.asarray(start, dtype=float))
            poses.append(np.asarray(start, dtype=float))
        worst_ratio = 0.0
        worst_joint = 0
        worst_effort = 0.0
        previous = poses[0]
        for goal in poses:
            maximum_delta = float(np.max(np.abs(np.asarray(goal) - previous)))
            steps = max(2, int(math.ceil(maximum_delta / 0.04)))
            for step in range(steps + 1):
                progress = float(step) / float(steps)
                values = previous + progress * (np.asarray(goal) - previous)
                base, regressor = self.evaluate_model(values)
                requested = base + regressor.dot(solution)
                ratios = np.abs(requested) / self.effort_limits
                if float(np.max(ratios)) > worst_ratio:
                    worst_ratio = float(np.max(ratios))
                    worst_joint = int(np.argmax(ratios))
                    worst_effort = float(requested[worst_joint])
            previous = np.asarray(goal)
        allowed = self.maximum_gravity_fraction - 0.02
        return {
            "worst_ratio": worst_ratio,
            "worst_joint": worst_joint + 1,
            "worst_effort_nm": worst_effort,
            "allowed_ratio": allowed,
            "includes_original_start": includes_start,
        }

    def make_output_directory(self):
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.output_dir = os.path.join(self.args.output_root, stamp)
        os.makedirs(self.output_dir, exist_ok=False)
        return stamp

    def write_raw_samples(self, pairs):
        raw_path = os.path.join(self.output_dir, "static_pairs.csv")
        temporary_path = raw_path + ".tmp"
        with open(temporary_path, "w", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(
                ["pose", "approach"]
                + JOINT_NAMES
                + ["effort_{}".format(name) for name in JOINT_NAMES]
                + ["stddev_{}".format(name) for name in JOINT_NAMES]
                + ["reported_speed_peak", "position_span"]
            )
            for pair in pairs:
                for approach in ("first", "second"):
                    sample = pair[approach]
                    writer.writerow(
                        [pair["name"], approach]
                        + sample["position"].tolist()
                        + sample["effort"].tolist()
                        + sample["effort_std"].tolist()
                        + [
                            sample.get("reported_speed_peak", ""),
                            sample.get("position_span", ""),
                        ]
                    )
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, raw_path)
        return raw_path

    def leave_one_pose_out_sensitivity(self, fit_pairs, validation_pairs):
        masses = []
        radii = []
        for omitted in range(len(fit_pairs)):
            reduced = [
                pair for index, pair in enumerate(fit_pairs) if index != omitted
            ]
            try:
                candidate = self.fit_payload(reduced, validation_pairs)
            except RuntimeError:
                continue
            masses.append(float(candidate["mass"]))
            radii.append(float(candidate["center_radius"]))
        if not masses:
            return None
        return {
            "mass_min_kg": min(masses),
            "mass_max_kg": max(masses),
            "center_radius_min_m": min(radii),
            "center_radius_max_m": max(radii),
            "successful_refits": len(masses),
        }

    def write_measurement_report(
        self,
        raw_path,
        model,
        profile_name,
        capacity,
        quality_rejections,
        control_rejections,
        sensitivity,
    ):
        measurement_valid = not quality_rejections
        control_eligible = measurement_valid and not control_rejections
        if not measurement_valid:
            status = "measurement_quality_rejected"
        elif not control_eligible:
            status = "measurement_valid_control_rejected"
        else:
            status = "measurement_valid_control_candidate"
        report = {
            "schema_version": 2,
            "status": status,
            "measurement_valid": measurement_valid,
            "eligible_for_control_hold_test": control_eligible,
            "profile_name": profile_name,
            "mass_kg": float(model["mass"]),
            "weight_newton": float(model["mass"] * 9.81),
            "center_of_mass_m": [float(value) for value in model["center"]],
            "center_of_mass_radius_m": float(model["center_radius"]),
            "linear_parameters_m_mx_my_mz": [
                float(value) for value in model["solution"]
            ],
            "unconstrained_mass_kg": float(model["unconstrained_mass"]),
            "fit_rmse_nm": float(model["fit_rmse"]),
            "fit_max_error_nm": float(model["fit_max"]),
            "held_out_rmse_nm": float(model["validation_rmse"]),
            "held_out_max_error_nm": float(model["validation_max"]),
            "regressor_condition": float(model["condition"]),
            "sample_pair_count": len(ALL_POSES),
            "raw_samples": os.path.realpath(raw_path),
            "leave_one_fit_pose_out_sensitivity": sensitivity,
            "measurement_quality_rejections": list(quality_rejections),
            "control_assessment": {
                "maximum_payload_mass_kg": MAXIMUM_PAYLOAD_MASS,
                "nominal_center_of_mass_radius_m": (
                    NOMINAL_PAYLOAD_CENTER_OF_MASS_RADIUS
                ),
                "center_of_mass_radius_gate_applied": False,
                "extended_center_of_mass_radius": bool(
                    model["center_radius"]
                    > NOMINAL_PAYLOAD_CENTER_OF_MASS_RADIUS
                ),
                "path_includes_original_start": bool(
                    capacity["includes_original_start"]
                ),
                "path_worst_joint": int(capacity["worst_joint"]),
                "path_worst_effort_nm": float(capacity["worst_effort_nm"]),
                "path_worst_capacity_fraction": float(capacity["worst_ratio"]),
                "path_allowed_capacity_fraction": float(
                    capacity["allowed_ratio"]
                ),
                "rejections": list(control_rejections),
            },
        }
        report_path = os.path.join(self.output_dir, "measurement_report.yaml")
        temporary_path = report_path + ".tmp"
        with open(temporary_path, "w") as output:
            output.write(
                "# Read-only measurement result. This file does not enable FREE control.\n"
            )
            yaml.safe_dump(report, output, allow_unicode=True, sort_keys=False)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, report_path)
        return report_path

    def write_results(self, pairs, model, profile_name):
        raw_path = self.write_raw_samples(pairs)
        candidate = {
            "schema_version": 1,
            "profile_name": profile_name,
            "mass_kg": float(model["mass"]),
            "center_of_mass_m": [float(value) for value in model["center"]],
            "fit_rmse_nm": float(model["fit_rmse"]),
            "fit_max_error_nm": float(model["fit_max"]),
            "held_out_rmse_nm": float(model["validation_rmse"]),
            "held_out_max_error_nm": float(model["validation_max"]),
            "regressor_condition": float(model["condition"]),
            "sample_pair_count": len(pairs),
            "raw_samples": raw_path,
        }
        candidate_path = os.path.join(self.output_dir, "payload_candidate.yaml")
        with open(candidate_path, "w") as output:
            output.write("# Candidate generated before the <=1 s CST hold test.\n")
            yaml.safe_dump(candidate, output, allow_unicode=True, sort_keys=False)
        return raw_path, candidate_path

    def set_payload_request(
        self,
        profile_name,
        model,
        drift,
        persist,
        controlled_hold_validation=False,
        hold_verified=False,
    ):
        request = SetPayloadModelRequest()
        request.profile_name = profile_name
        request.mass = float(model["mass"])
        request.center_of_mass = [float(value) for value in model["center"]]
        request.fit_rmse = float(model["fit_rmse"])
        request.fit_max_error = float(model["validation_max"])
        request.validation_drift = float(drift)
        request.sample_count = 2 * len(ALL_POSES)
        request.persist = bool(persist)
        request.controlled_hold_validation = bool(controlled_hold_validation)
        request.hold_verified = bool(hold_verified)
        return request

    def restore_previous_payload(self):
        if self.previous_payload is None:
            return
        request = SetPayloadModelRequest()
        request.profile_name = self.previous_payload.profile_name
        request.mass = self.previous_payload.mass
        request.center_of_mass = list(self.previous_payload.center_of_mass)
        request.fit_rmse = self.previous_payload.fit_rmse
        request.fit_max_error = self.previous_payload.fit_max_error
        request.validation_drift = self.previous_payload.validation_drift
        request.sample_count = self.previous_payload.sample_count
        request.persist = False
        request.controlled_hold_validation = False
        request.hold_verified = self.previous_payload.hold_verified
        response = self.set_payload(request)
        if not response.success:
            raise RuntimeError("候选失败后无法回滚原负载模型：" + response.message)
        self.candidate_staged = False
        log("[回滚] 已恢复原负载配置：" + response.message)

    def wait_for_strict_preflight(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        last = ""
        publications = queue.Queue()
        subscriber = rospy.Subscriber(
            self.manager_namespace + "/model_validation",
            String,
            lambda message: publications.put(message.data),
            queue_size=10,
        )
        publication_count = 0
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                raise_if_cancelled()
                remaining = max(0.0, deadline - time.monotonic())
                try:
                    message = publications.get(timeout=min(1.0, remaining))
                except queue.Empty:
                    continue
                publication_count += 1
                last = message
                outcome = strict_preflight_outcome(publication_count, message)
                if publication_count == 1:
                    log(
                        "[保持验证] 已忽略订阅建立时的锁存预检；"
                        "等待候选模型新一轮静止采样"
                    )
                if outcome is True:
                    return message
                if outcome is False:
                    raise RuntimeError("候选负载的静态预检未通过：" + message)
        finally:
            subscriber.unregister()
        raise RuntimeError("等待候选负载严格预检超时：" + last)

    def wait_for_manager_state(self, expected, timeout, honor_cancel=True):
        deadline = time.monotonic() + timeout
        last = ""
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if honor_cancel:
                raise_if_cancelled()
            last = rospy.wait_for_message(
                self.manager_namespace + "/state", String, timeout=1.0
            ).data
            if last == expected:
                return
            if last in ("ERROR", "FAULT", "FALLBACK"):
                raise RuntimeError("FREE 状态机进入 " + last)
            rospy.sleep(0.05)
        raise RuntimeError("等待 FREE 状态 {} 超时，最后状态 {}".format(expected, last))

    def controlled_hold_test(self):
        damping_message = rospy.wait_for_message(
            self.manager_namespace + "/damping_scales",
            Float64MultiArray,
            timeout=2.0,
        )
        self.current_damping = list(damping_message.data)
        verification_damping = [max(2.0, value) for value in self.current_damping]
        response = self.set_damping(
            SetDampingScalesRequest(scales=verification_damping)
        )
        if not response.success:
            raise RuntimeError("无法设置标定验证阻尼：" + response.message)
        self.wait_for_strict_preflight()
        start, _, _ = self.monitor.snapshot()
        log(
            "[保持验证] 进入高阻尼零力保持 {:.2f} s；硬上限 1.00 s".format(
                self.args.validation_hold_seconds
            )
        )
        entered = self.set_freedrive(SetBoolRequest(data=True))
        if not entered.success:
            raise RuntimeError("候选负载无法进入 FREE：" + entered.message)
        exit_lock = threading.Lock()
        exit_done = threading.Event()
        exit_outcome = {"started": False, "response": None, "error": None}

        def request_hold_exit(source):
            with exit_lock:
                if exit_outcome["started"]:
                    return
                exit_outcome["started"] = True
            if source == "watchdog":
                log("[保持验证] 1.00 s 硬看门狗请求退出 FREE")
            try:
                exit_outcome["response"] = self.set_freedrive(
                    SetBoolRequest(data=False)
                )
            except Exception as error:
                exit_outcome["error"] = error
            finally:
                exit_done.set()

        hold_watchdog = threading.Timer(1.0, request_hold_exit, args=("watchdog",))
        hold_watchdog.daemon = True
        hold_watchdog.start()
        maximum_drift = 0.0
        maximum_speed = 0.0
        failure = None
        deadline = time.monotonic() + self.args.validation_hold_seconds
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                raise_if_cancelled()
                self.monitor.require_driver_ready()
                position, velocity, _ = self.monitor.snapshot()
                maximum_drift = max(
                    maximum_drift, float(np.max(np.abs(position - start)))
                )
                maximum_speed = max(maximum_speed, float(np.max(np.abs(velocity))))
                if maximum_drift > 0.015 or maximum_speed > 0.08:
                    failure = (
                        "保持验证超限：漂移 {:.5f} rad，速度 {:.5f} rad/s".format(
                            maximum_drift, maximum_speed
                        )
                    )
                    break
                rospy.sleep(0.01)
        finally:
            request_hold_exit("main")
            hold_watchdog.cancel()
            if not exit_done.wait(2.0):
                failure = (failure + "；" if failure else "") + "退出 FREE 服务 2 秒未返回"
            elif exit_outcome["error"] is not None:
                failure = (failure + "；" if failure else "") + str(
                    exit_outcome["error"]
                )
            elif not exit_outcome["response"].success:
                failure = (failure + "；" if failure else "") + exit_outcome[
                    "response"
                ].message
            self.wait_for_manager_state("READY", 10.0, honor_cancel=False)
        if failure:
            raise RuntimeError(failure)
        log(
            "[保持验证] 通过：最大漂移 {:.6f} rad，最大速度 {:.6f} rad/s".format(
                maximum_drift, maximum_speed
            )
        )
        return maximum_drift

    def restore_damping(self):
        if self.current_damping is None:
            return
        response = self.set_damping(
            SetDampingScalesRequest(scales=self.current_damping)
        )
        if not response.success:
            raise RuntimeError("无法恢复标定前阻尼：" + response.message)
        self.current_damping = None

    def recover_after_failure(self, primary_error):
        recovery_errors = []
        try:
            self.group.stop()
        except Exception as error:
            recovery_errors.append("停止 MoveIt 轨迹失败：" + str(error))
        try:
            response = self.set_freedrive(SetBoolRequest(data=False))
            if not response.success:
                recovery_errors.append("退出 FREE 失败：" + response.message)
        except Exception as error:
            recovery_errors.append("请求退出 FREE 异常：" + str(error))
        try:
            self.wait_for_manager_state("READY", 5.0, honor_cancel=False)
        except Exception as error:
            recovery_errors.append("恢复 READY 失败：" + str(error))
        if self.candidate_staged:
            try:
                self.restore_previous_payload()
            except Exception as error:
                recovery_errors.append(str(error))
        if self.current_damping is not None:
            try:
                self.restore_damping()
            except Exception as error:
                recovery_errors.append(str(error))
        if recovery_errors:
            raise RuntimeError(
                "{}；恢复阶段另有异常：{}".format(
                    primary_error, "；".join(recovery_errors)
                )
            )
        raise primary_error

    def identify_and_report(self, pairs, raw_path, profile_name, start=None):
        if len(pairs) != len(ALL_POSES):
            raise RuntimeError(
                "负载辨识需要完整的 {} 组双向样本，当前只有 {} 组".format(
                    len(ALL_POSES), len(pairs)
                )
            )
        fit_pairs = pairs[: len(FIT_POSES)]
        validation_pairs = pairs[len(FIT_POSES) :]
        model = self.fit_payload(fit_pairs, validation_pairs)
        capacity = self.evaluate_capacity(model, start=start)
        quality_rejections = self.measurement_quality_rejections(model)
        control_rejections = self.control_rejections(model, capacity)
        sensitivity = self.leave_one_pose_out_sensitivity(
            fit_pairs, validation_pairs
        )
        report_path = self.write_measurement_report(
            raw_path,
            model,
            profile_name,
            capacity,
            quality_rejections,
            control_rejections,
            sensitivity,
        )
        log(
            "[测量] 质量 {:.3f} kg（重量 {:.2f} N），重心 "
            "[{:.3f}, {:.3f}, {:.3f}] m，半径 {:.3f} m；"
            "拟合/留出 RMSE {:.2f}/{:.2f} Nm，路径容量峰值 {:.1f}%".format(
                model["mass"],
                model["mass"] * 9.81,
                model["center"][0],
                model["center"][1],
                model["center"][2],
                model["center_radius"],
                model["fit_rmse"],
                model["validation_rmse"],
                100 * capacity["worst_ratio"],
            )
        )
        if model["center_radius"] > NOMINAL_PAYLOAD_CENTER_OF_MASS_RADIUS:
            log(
                "[扩展距离] 重心半径 {:.3f} m 超过 {:.2f} m 名义参考值；"
                "不按距离单独拒绝，仍必须通过留出误差、整路径力矩容量和"
                "最长 1 秒实际保持验证".format(
                    model["center_radius"],
                    NOMINAL_PAYLOAD_CENTER_OF_MASS_RADIUS,
                )
            )
        log("[记录] 只读测量报告：" + report_path)
        return (
            model,
            capacity,
            quality_rejections,
            control_rejections,
            report_path,
        )

    def assess_and_activate(self, pairs, raw_path, profile_name, start, return_to_start):
        (
            model,
            capacity,
            quality_rejections,
            control_rejections,
            report_path,
        ) = self.identify_and_report(
            pairs, raw_path, profile_name, start=start
        )
        if quality_rejections:
            raise RuntimeError(
                "测量报告已保存，但辨识质量未通过：{}；报告={}".format(
                    "；".join(quality_rejections), report_path
                )
            )
        if control_rejections:
            raise RuntimeError(
                "测量已完成且报告已保存，但候选未启用 FREE：{}；"
                "原负载配置保持不变；报告={}".format(
                    "；".join(control_rejections), report_path
                )
            )
        raw_path, candidate_path = self.write_results(
            pairs, model, profile_name
        )
        log(
            "[辨识] 质量 {:.3f} kg，重心 [{:.3f}, {:.3f}, {:.3f}] m；"
            "拟合/留出 RMSE {:.2f}/{:.2f} Nm，路径容量峰值 {:.1f}%".format(
                model["mass"],
                model["center"][0],
                model["center"][1],
                model["center"][2],
                model["fit_rmse"],
                model["validation_rmse"],
                100 * capacity["worst_ratio"],
            )
        )
        log("[记录] 原始样本：" + raw_path)
        log("[记录] 候选模型：" + candidate_path)
        staged = self.set_payload(
            self.set_payload_request(
                profile_name,
                model,
                0.0,
                False,
                controlled_hold_validation=True,
            )
        )
        if not staged.success:
            raise RuntimeError("无法暂存候选负载模型：" + staged.message)
        self.candidate_staged = True
        drift = self.controlled_hold_test()
        self.restore_damping()
        persisted = self.set_payload(
            self.set_payload_request(
                profile_name, model, drift, True, hold_verified=True
            )
        )
        if not persisted.success:
            raise RuntimeError("候选验证通过但持久化失败：" + persisted.message)
        self.candidate_staged = False
        log("[启用] " + persisted.message)
        if return_to_start:
            self.move_to(start, "标定完成返回原姿态")
        log("[完成] 自动末端负载标定全部通过；下一次 FREE 使用新配置")
        return 0

    def resume_saved_samples(self, start, current_height):
        if not self.args.confirmed_by_panel:
            raise RuntimeError("复用样本的真机保持验证只能从 Panel 清场确认后启动")
        if current_height < MINIMUM_FLANGE_HEIGHT:
            target = np.asarray(VALIDATION_POSES[-1][1], dtype=float)
            target_height = self.validate_state(target, "短保持高位 H")
            recovery_floor = max(0.05, current_height - 0.015)
            log(
                "[复用] 当前法兰 {:.3f} m；先以 3% 规划抬升到高位 H "
                "{:.3f} m，轨迹不得低于 {:.3f} m".format(
                    current_height, target_height, recovery_floor
                )
            )
            self.move_to(
                target,
                "复用样本前抬升到高位 H",
                minimum_height=recovery_floor,
            )
            start, _, _ = self.monitor.snapshot()
            current_height = self.validate_state(start, "当前短时保持验证姿态")
        self.validate_state(start, "当前短时保持验证姿态")
        self.monitor.require_driver_ready()
        self.previous_payload = self.get_payload(GetPayloadModelRequest())
        if not self.previous_payload.success:
            raise RuntimeError("无法读取原负载配置：" + self.previous_payload.message)
        source_path, pairs = self.read_raw_samples(self.args.resume_samples)
        stamp = self.make_output_directory()
        profile_name = "resumed-payload-" + stamp
        raw_path = self.write_raw_samples(pairs)
        log("[复用] 已重新解析完整双向样本：" + source_path)
        log("[复用] 不重走标定轨迹，仅检查当前姿态并进行最多 1 秒保持验证")
        try:
            return self.assess_and_activate(
                pairs, raw_path, profile_name, start, return_to_start=False
            )
        except Exception as error:
            self.recover_after_failure(error)

    def analyze_saved_samples(self):
        raw_path, pairs = self.read_raw_samples(self.args.analyze_samples)
        self.output_dir = os.path.dirname(raw_path)
        profile_name = "reanalyzed-" + os.path.basename(self.output_dir)
        (
            _,
            _,
            quality_rejections,
            control_rejections,
            report_path,
        ) = self.identify_and_report(pairs, raw_path, profile_name, start=None)
        if quality_rejections:
            log("[分析结论] 测量质量未通过：" + "；".join(quality_rejections))
        elif control_rejections:
            log(
                "[分析结论] 测量有效，但不会启用 FREE："
                + "；".join(control_rejections)
            )
        else:
            log("[分析结论] 测量有效，候选满足进入短时保持验证的条件")
        log("[完成] 只读复算完成；未发送轨迹、FREE 或负载修改命令：" + report_path)
        return 0

    def run(self):
        self.wait_for_services()
        if self.analysis_only:
            return self.analyze_saved_samples()
        self.monitor.wait_for_initial_driver_state()
        start, velocity, _ = self.monitor.snapshot()
        if float(np.max(np.abs(velocity))) > 0.004:
            raise RuntimeError("开始标定前机械臂必须静止")
        self.evaluate_model(start)
        validation_start = start
        current_height = self.flange_height(start)
        if self.args.resume_samples:
            return self.resume_saved_samples(start, current_height)
        if not self.args.execute and current_height < MINIMUM_FLANGE_HEIGHT:
            validation_start = np.asarray(FIT_POSES[0][1], dtype=float)
            log(
                "[只读检查] 当前法兰仅 {:.3f} m；真机执行会拒绝。"
                "本次仅以虚拟高位起点验证预设路径，不移动机械臂。".format(
                    current_height
                )
            )
        minimum_height = self.validate_complete_sequence(validation_start)
        log(
            "[只读检查] 全部双向标定路径通过 MoveIt；法兰最低 {:.3f} m，"
            "始终高于 {:.3f} m 上半工作区门限".format(
                minimum_height, MINIMUM_FLANGE_HEIGHT
            )
        )
        if not self.args.execute:
            log("[完成] 当前为只读验证，没有发送轨迹、FREE 或负载修改命令")
            return 0
        if not self.args.confirmed_by_panel:
            raise RuntimeError("真机执行只能从 Panel 清场确认对话框启动")
        self.monitor.require_driver_ready()
        self.previous_payload = self.get_payload(GetPayloadModelRequest())
        if not self.previous_payload.success:
            raise RuntimeError("无法读取原负载配置：" + self.previous_payload.message)
        stamp = self.make_output_directory()
        profile_name = "auto-payload-" + stamp
        pairs = []
        try:
            for name, target in ALL_POSES:
                pairs.append(self.collect_pose_pair(name, target))
                raw_path = self.write_raw_samples(pairs)
                log(
                    "[记录] 已保存 {}/{} 组双向原始样本：{}".format(
                        len(pairs), len(ALL_POSES), raw_path
                    )
                )
            raw_path = self.write_raw_samples(pairs)
            return self.assess_and_activate(
                pairs, raw_path, profile_name, start, return_to_start=True
            )
        except Exception as primary_error:
            self.recover_after_failure(primary_error)


def validate_arguments(args):
    if args.confirmed_by_panel and not (args.execute or args.resume_samples):
        raise RuntimeError(
            "--confirmed-by-panel 只能与 --execute 或 --resume-samples 一起使用"
        )
    if args.resume_samples and not args.confirmed_by_panel:
        raise RuntimeError("--resume-samples 必须由 Panel 清场确认后调用")
    if not (0.01 <= args.velocity_scale <= 0.05):
        raise RuntimeError("标定速度倍率必须在 1% 到 5%")
    if not (0.01 <= args.acceleration_scale <= 0.05):
        raise RuntimeError("标定加速度倍率必须在 1% 到 5%")
    if args.settle_seconds < 0.8:
        raise RuntimeError("静止等待必须至少 0.8 s")
    if args.sample_seconds < 0.6:
        raise RuntimeError("每个静态采样必须至少 0.6 s")
    if not (0.20 <= args.validation_hold_seconds <= 1.0):
        raise RuntimeError("零力保持验证必须在 0.20 s 到 1.00 s")


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("calibrate_elfin_payload", anonymous=False)
    signal.signal(signal.SIGINT, request_cancel)
    signal.signal(signal.SIGTERM, request_cancel)
    args = parse_args()
    validate_arguments(args)
    calibrator = PayloadCalibrator(args)
    return calibrator.run()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (rospy.ROSException, rospy.ServiceException, RuntimeError, ValueError) as error:
        log("[失败] " + str(error))
        sys.exit(1)
    finally:
        moveit_commander.roscpp_shutdown()
