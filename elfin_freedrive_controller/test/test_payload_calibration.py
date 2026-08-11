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
            "velocity_scale": 0.03,
            "acceleration_scale": 0.03,
            "settle_seconds": 1.0,
            "sample_seconds": 0.8,
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

    def test_latest_measured_tool_is_eligible_with_validated_limits(self):
        model = {
            "mass": 1.2415,
            "center_radius": 0.4614,
            "condition": 7.18,
            "fit_rmse": 0.97,
            "fit_max": 2.83,
            "validation_rmse": 0.79,
            "validation_max": 1.61,
        }
        capacity = {
            "worst_ratio": 29.6975 / 36.0,
            "allowed_ratio": 0.88,
            "worst_joint": 3,
        }

        self.assertEqual(MODULE.PayloadCalibrator.measurement_quality_rejections(model), [])
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
            "worst_ratio": 1.03,
            "allowed_ratio": 0.88,
            "worst_joint": 3,
            "worst_effort_nm": 30.9,
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
            self.assertEqual(report["schema_version"], 2)
            self.assertFalse(
                report["control_assessment"]["center_of_mass_radius_gate_applied"]
            )


if __name__ == "__main__":
    unittest.main()
