#!/usr/bin/python3

import importlib.util
import os
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np
import yaml


SCRIPT = os.path.join(
    os.path.dirname(__file__), "..", "script", "calibrate_elfin_payload.py"
)
SPEC = importlib.util.spec_from_file_location("calibrate_elfin_payload", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PayloadCalibrationMathTest(unittest.TestCase):
    @staticmethod
    def valid_arguments(**overrides):
        values = {
            "confirmed_by_panel": False,
            "execute": False,
            "resume_samples": None,
            "velocity_scale": 0.05,
            "acceleration_scale": 0.05,
            "settle_seconds": 0.8,
            "sample_seconds": 0.6,
            "validation_hold_seconds": 0.8,
        }
        values.update(overrides)
        return SimpleNamespace(**values)

    def test_robust_fit_recovers_payload_with_one_outlier(self):
        generator = np.random.RandomState(7)
        matrix = generator.normal(size=(60, 4))
        matrix[:, 0] *= 4.0
        matrix[:, 1:] *= 9.81
        expected = np.asarray([2.2, 0.22, -0.11, 0.33])
        values = matrix.dot(expected) + generator.normal(scale=0.08, size=60)
        values[8] += 12.0
        sigma = np.full(60, 0.2)

        fitted, _, condition, _ = MODULE.robust_weighted_fit(
            matrix, values, sigma
        )

        self.assertLess(condition, 10.0)
        np.testing.assert_allclose(fitted, expected, atol=0.035)

    def test_positive_small_payload_remains_a_physical_payload(self):
        matrix = np.eye(4)
        solution = np.asarray([0.04, 0.002, -0.001, 0.003])
        normalized, mass, center = MODULE.normalize_payload_solution(
            solution, matrix
        )
        np.testing.assert_array_equal(normalized, solution)
        self.assertEqual(mass, 0.04)
        np.testing.assert_allclose(center, np.asarray([0.05, -0.025, 0.075]))

    def test_negative_unconstrained_solution_projects_to_empty_boundary(self):
        normalized, mass, center = MODULE.normalize_payload_solution(
            np.asarray([-0.5, 0.1, 0.0, 0.0]), np.eye(4)
        )
        np.testing.assert_array_equal(normalized, np.zeros(4))
        self.assertEqual(mass, 0.0)
        np.testing.assert_array_equal(center, np.zeros(3))

    def test_cancel_signal_becomes_recoverable_script_exception(self):
        MODULE.CANCEL_REQUESTED.clear()
        try:
            MODULE.request_cancel(None, None)
            with self.assertRaises(MODULE.CalibrationCancelled):
                MODULE.raise_if_cancelled()
        finally:
            MODULE.CANCEL_REQUESTED.clear()

    def test_resume_samples_requires_panel_confirmation(self):
        arguments = self.valid_arguments(resume_samples="static_pairs.csv")

        with self.assertRaises(RuntimeError):
            MODULE.validate_arguments(arguments)

    def test_panel_confirmed_resume_samples_is_accepted(self):
        arguments = self.valid_arguments(
            resume_samples="static_pairs.csv", confirmed_by_panel=True
        )

        MODULE.validate_arguments(arguments)

    def test_initial_latched_preflight_result_is_never_accepted(self):
        self.assertIsNone(
            MODULE.strict_preflight_outcome(
                1, "未通过（位置模式静止预检）；旧模型结果"
            )
        )
        self.assertIsNone(
            MODULE.strict_preflight_outcome(
                1, "通过（位置模式静止预检）；旧模型结果"
            )
        )

    def test_second_preflight_publication_is_classified(self):
        self.assertTrue(
            MODULE.strict_preflight_outcome(
                2, "通过（位置模式静止预检）；候选模型结果"
            )
        )
        self.assertFalse(
            MODULE.strict_preflight_outcome(
                2, "未通过（位置模式静止预检）；候选模型结果"
            )
        )
        self.assertIsNone(
            MODULE.strict_preflight_outcome(2, "等待静止力矩样本 8/12")
        )

    def test_bidirectional_samples_use_each_actual_joint_position(self):
        calibrator = object.__new__(MODULE.PayloadCalibrator)
        calibrator.move_to = lambda target, label: None
        first = {
            "position": np.zeros(6),
            "effort": np.arange(6, dtype=float),
            "effort_std": np.full(6, 0.1),
        }
        second = {
            "position": np.full(6, 0.02),
            "effort": np.arange(6, dtype=float) + 2.0,
            "effort_std": np.full(6, 0.2),
        }
        samples = iter((first, second))
        calibrator.collect_static_sample = lambda label: next(samples)
        evaluated = []

        def evaluate(values):
            values = np.asarray(values, dtype=float)
            evaluated.append(values.copy())
            scale = 1.0 if np.allclose(values, first["position"]) else 3.0
            return np.full(6, scale), np.full((6, 4), scale)

        calibrator.evaluate_model = evaluate
        pair = calibrator.collect_pose_pair("test", np.zeros(6))

        self.assertEqual(len(evaluated), 2)
        np.testing.assert_array_equal(evaluated[0], first["position"])
        np.testing.assert_array_equal(evaluated[1], second["position"])
        np.testing.assert_allclose(pair["position"], np.full(6, 0.01))
        np.testing.assert_allclose(pair["base"], np.full(6, 2.0))
        np.testing.assert_allclose(pair["regressor"], np.full((6, 4), 2.0))

    def test_raw_samples_are_replaced_atomically(self):
        calibrator = object.__new__(MODULE.PayloadCalibrator)
        sample = {
            "position": np.arange(6, dtype=float),
            "effort": np.arange(6, dtype=float) + 10.0,
            "effort_std": np.full(6, 0.2),
        }
        pair = {"name": "test", "first": sample, "second": sample}
        with tempfile.TemporaryDirectory() as directory:
            calibrator.output_dir = directory
            path = calibrator.write_raw_samples([pair])

            self.assertTrue(os.path.isfile(path))
            self.assertFalse(os.path.exists(path + ".tmp"))
            with open(path) as saved:
                self.assertEqual(sum(1 for _ in saved), 3)

    def test_isolated_reported_velocity_spike_does_not_fake_motion(self):
        positions = np.zeros((40, 6))
        positions[:, 0] = np.linspace(0.0, 0.00001, 40)
        velocities = np.zeros((40, 6))
        velocities[7, 2] = 0.09116
        efforts = np.tile(np.arange(6, dtype=float), (40, 1))

        speed, span, _ = MODULE.validate_static_window(
            "test", positions, velocities, efforts
        )

        self.assertAlmostEqual(speed, 0.09116)
        self.assertAlmostEqual(span, 0.00001)

    def test_real_encoder_motion_rejects_static_sample(self):
        positions = np.zeros((40, 6))
        positions[:, 2] = np.linspace(0.0, 0.003, 40)
        velocities = np.zeros((40, 6))
        efforts = np.tile(np.arange(6, dtype=float), (40, 1))

        with self.assertRaises(RuntimeError):
            MODULE.validate_static_window("test", positions, velocities, efforts)

    def test_latest_measured_tool_is_eligible_with_rated_effort_limits(self):
        model = {
            "mass": 0.613,
            "center_radius": 0.615,
            "condition": 7.7,
            "fit_rmse": 0.97,
            "fit_max": 2.83,
            "validation_rmse": 0.79,
            "validation_max": 1.61,
        }
        capacity = {
            "worst_ratio": 29.6975 / 200.0,
            "allowed_ratio": 0.88,
            "worst_joint": 3,
        }

        self.assertEqual(MODULE.PayloadCalibrator.measurement_quality_rejections(model), [])
        self.assertEqual(
            MODULE.PayloadCalibrator.control_rejections(model, capacity), []
        )

    def test_hold_pose_gate_is_separate_from_calibration_path_diagnostic(self):
        calibrator = object.__new__(MODULE.PayloadCalibrator)
        calibrator.effort_limits = np.asarray([420.0, 420.0, 200.0, 200.0, 69.0, 69.0])
        calibrator.maximum_gravity_fraction = 0.90
        calibrator.calibration_goals = lambda: [("high-load", np.ones(6))]

        def evaluate(values):
            base = np.zeros(6)
            base[2] = 190.0 if values[0] > 0.5 else 20.0
            return base, np.zeros((6, 4))

        calibrator.evaluate_model = evaluate
        model = {"solution": np.zeros(4), "mass": 1.5}

        capacity = calibrator.evaluate_capacity(
            model, hold_position=np.zeros(6)
        )

        self.assertAlmostEqual(capacity["worst_ratio"], 20.0 / 200.0)
        self.assertAlmostEqual(capacity["path_worst_ratio"], 0.95)
        self.assertEqual(
            MODULE.PayloadCalibrator.control_rejections(model, capacity), []
        )

    def test_extended_distance_tool_is_bounded_by_capacity_not_radius(self):
        model = {
            "mass": 0.466,
            "center_radius": 0.895,
        }
        capacity = {
            "worst_ratio": 0.673,
            "allowed_ratio": 0.88,
            "worst_joint": 3,
        }

        self.assertEqual(
            MODULE.PayloadCalibrator.control_rejections(model, capacity), []
        )

    def test_extended_distance_tool_still_rejects_capacity_overrun(self):
        model = {
            "mass": 0.466,
            "center_radius": 0.895,
        }
        capacity = {
            "worst_ratio": 0.881,
            "allowed_ratio": 0.88,
            "worst_joint": 3,
        }

        rejections = MODULE.PayloadCalibrator.control_rejections(model, capacity)

        self.assertEqual(len(rejections), 1)
        self.assertIn("J3", rejections[0])

    def test_above_nominal_radius_keeps_capacity_and_hold_gates(self):
        model = {
            "mass": 0.613,
            "center_radius": 0.615,
        }
        capacity = {
            "worst_ratio": 0.683,
            "allowed_ratio": 0.88,
            "worst_joint": 3,
        }

        self.assertEqual(
            MODULE.PayloadCalibrator.control_rejections(model, capacity), []
        )

    def test_measurement_report_is_saved_when_control_is_rejected(self):
        calibrator = object.__new__(MODULE.PayloadCalibrator)
        model = {
            "solution": np.asarray([1.47, 0.15, -0.08, 0.54]),
            "unconstrained_mass": 1.47,
            "mass": 1.47,
            "center": np.asarray([0.10, -0.05, 0.37]),
            "center_radius": 0.3865,
            "condition": 7.1,
            "fit_rmse": 1.22,
            "fit_max": 3.40,
            "validation_rmse": 1.26,
            "validation_max": 2.46,
        }
        capacity = {
            "worst_ratio": 0.82,
            "allowed_ratio": 0.88,
            "worst_joint": 3,
            "worst_effort_nm": 29.5,
            "path_worst_ratio": 1.03,
            "path_worst_joint": 3,
            "path_worst_effort_nm": 37.1,
            "includes_original_start": False,
        }
        with tempfile.TemporaryDirectory() as directory:
            calibrator.output_dir = directory
            raw_path = os.path.join(directory, "static_pairs.csv")
            with open(raw_path, "w") as output:
                output.write("test\n")
            path = calibrator.write_measurement_report(
                raw_path,
                model,
                "test-profile",
                capacity,
                [],
                ["control rejected"],
                None,
            )

            self.assertFalse(os.path.exists(path + ".tmp"))
            with open(path) as saved:
                report = yaml.safe_load(saved)
            self.assertTrue(report["measurement_valid"])
            self.assertFalse(report["eligible_for_control_hold_test"])
            self.assertEqual(report["status"], "measurement_valid_control_rejected")
            self.assertAlmostEqual(report["mass_kg"], 1.47)
            self.assertEqual(report["schema_version"], 3)
            self.assertAlmostEqual(
                report["control_assessment"]["hold_pose_capacity_fraction"],
                0.82,
            )
            self.assertAlmostEqual(
                report["control_assessment"]["path_worst_capacity_fraction"],
                1.03,
            )
            self.assertFalse(
                report["control_assessment"]["center_of_mass_radius_gate_applied"]
            )


    def test_reduced_pose_set_covers_actual_workload_and_all_wrist_directions(self):
        self.assertEqual(len(MODULE.FIT_POSES), 5)
        self.assertEqual(len(MODULE.VALIDATION_POSES), 2)
        poses = np.asarray([values for _, values in MODULE.ALL_POSES])
        self.assertLessEqual(float(np.min(poses[:, 1])), 0.28)
        self.assertGreaterEqual(float(np.max(poses[:, 1])), 0.75)
        self.assertLessEqual(float(np.min(poses[:, 2])), -0.76)
        self.assertLessEqual(float(np.min(poses[:, 3])), -0.45)
        self.assertGreaterEqual(float(np.max(poses[:, 3])), 0.55)
        self.assertLessEqual(float(np.min(poses[:, 4])), -0.50)
        self.assertGreaterEqual(float(np.max(poses[:, 4])), 0.50)
        self.assertLessEqual(float(np.min(poses[:, 5])), -0.35)
        self.assertGreaterEqual(float(np.max(poses[:, 5])), 0.50)
        np.testing.assert_allclose(
            MODULE.VALIDATION_POSES[-1][1],
            np.asarray([0.0, 0.75, -0.76, 0.0, -0.15, 0.0]),
        )
        np.testing.assert_allclose(
            MODULE.APPROACH_DELTA,
            np.asarray([0.0, 0.03, -0.03, 0.04, -0.04, 0.04]),
        )

    def test_step_envelope_loader_includes_configured_padding(self):
        document = {
            "collision_model": {
                "enabled": True,
                "attach_link": "elfin_end_link",
                "cad_bounds_mm": {
                    "min": [-100.0, -200.0, -300.0],
                    "max": [100.0, 200.0, 300.0],
                },
                "cad_to_attach": {
                    "translation_m": [0.01, 0.02, 0.03],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "conservative_envelope": {
                    "enabled": True,
                    "padding_m": 0.005,
                    "boxes": [{"object_id": "box_a"}],
                },
            },
            "validation": {
                "geometry_matches_step": True,
                "execution_ready": True,
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "tool.yaml")
            with open(path, "w") as output:
                yaml.safe_dump(document, output)
            resolved, object_ids, attach_link, corners = (
                MODULE.load_tool_floor_corners(path)
            )
        self.assertEqual(resolved, os.path.realpath(path))
        self.assertEqual(object_ids, ("box_a",))
        self.assertEqual(attach_link, "elfin_end_link")
        self.assertEqual(len(corners), 8)
        values = np.asarray(corners)
        np.testing.assert_allclose(
            np.min(values, axis=0), [-0.095, -0.185, -0.275]
        )
        np.testing.assert_allclose(
            np.max(values, axis=0), [0.115, 0.225, 0.335]
        )

    def test_height_gate_rejects_floor_flange_and_tool_violations(self):
        safe = {
            "lowest_link": "elfin_link1",
            "lowest_link_height": 0.22,
            "flange": 0.51,
            "tool_height": 0.41,
        }
        MODULE.require_safe_height_report(
            safe, "safe", MODULE.MINIMUM_FLANGE_HEIGHT
        )
        for key, value, message in (
            ("lowest_link_height", -0.001, "z=0"),
            ("flange", 0.449, "法兰高度"),
            ("tool_height", 0.299, "STEP 包络"),
        ):
            unsafe = dict(safe)
            unsafe[key] = value
            with self.assertRaisesRegex(RuntimeError, message):
                MODULE.require_safe_height_report(
                    unsafe, "unsafe", MODULE.MINIMUM_FLANGE_HEIGHT
                )

    def test_attached_tool_gate_requires_every_step_proxy_on_attach_link(self):
        calibrator = object.__new__(MODULE.PayloadCalibrator)
        calibrator.tool_object_ids = ("box_a", "box_b")
        calibrator.tool_attach_link = "elfin_end_link"
        calibrator.scene = SimpleNamespace(
            get_attached_objects=lambda requested: {
                name: SimpleNamespace(link_name="elfin_end_link")
                for name in requested
            }
        )
        calibrator.verify_tool_collision_model_attached()
        calibrator.scene = SimpleNamespace(
            get_attached_objects=lambda _requested: {
                "box_a": SimpleNamespace(link_name="elfin_end_link")
            }
        )
        self.assertEqual(len(calibrator.attached_tool_objects), 2)
        with self.assertRaisesRegex(RuntimeError, "box_b"):
            calibrator.verify_tool_collision_model_attached()

if __name__ == "__main__":
    unittest.main()
