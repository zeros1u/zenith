from __future__ import annotations

import math
import unittest

from zenith.camera import (
    CameraModel,
    detect_box,
    estimate_range,
    minimum_horizontal_resolution,
    range_from_apparent_size,
)
from zenith.guidance import TargetTrack, build_prediction_ovals
from zenith.math3d import Vec3, basis_from_forward
from zenith.meshes import get_mesh
from zenith.models import DRONE_SPECS, ROCKET_SPECS, TARGET_SPECS
from zenith.physics import DroneState, maximum_travel_distance
from zenith.simulation import InterceptionSimulation, SCENARIOS, SimulationConfig


class VectorTests(unittest.TestCase):
    def test_camera_basis_is_orthonormal(self) -> None:
        right, up, forward = basis_from_forward(Vec3(1, 0.3, 2))
        self.assertAlmostEqual(right.length(), 1.0)
        self.assertAlmostEqual(up.length(), 1.0)
        self.assertAlmostEqual(forward.length(), 1.0)
        self.assertAlmostEqual(right.dot(up), 0.0, places=7)
        self.assertAlmostEqual(right.dot(forward), 0.0, places=7)


class CameraTests(unittest.TestCase):
    def test_direct_pinhole_formula(self) -> None:
        self.assertAlmostEqual(range_from_apparent_size(0.7, 7.0, 1000.0), 100.0)

    def test_known_box_range_is_close_to_truth(self) -> None:
        camera = CameraModel(1920, 1080, 75)
        spec = DRONE_SPECS[0]
        target = DroneState(spec, Vec3(8, 34, 250), Vec3(), Vec3(0.1, 0.6, 0.2))
        origin = Vec3(0, 30, 0)
        forward = (target.position - origin).normalized()
        detection = detect_box(target, origin, forward, camera)
        estimate = estimate_range(detection, spec, target.orientation, forward, camera)
        self.assertIsNotNone(estimate)
        assert estimate is not None
        error = abs(estimate.distance_m - origin.distance_to(target.position))
        # At this distance the target is only a few pixels wide. The error must
        # remain inside the estimator's explicit pixel-quantisation uncertainty.
        self.assertLess(error, estimate.sigma_m)

    def test_minimum_resolution_scales_linearly(self) -> None:
        first = minimum_horizontal_resolution(100, 0.7, 75, 12)
        second = minimum_horizontal_resolution(200, 0.7, 75, 12)
        self.assertIn(second, (first * 2 - 1, first * 2, first * 2 + 1))


class GuidanceTests(unittest.TestCase):
    def test_maximum_travel_respects_speed_cap(self) -> None:
        spec = DRONE_SPECS[0]
        distance = maximum_travel_distance(spec.max_speed, spec, 5.0)
        self.assertAlmostEqual(distance, spec.max_speed * 5.0)

    def test_four_extremes_per_prediction_oval(self) -> None:
        interceptor = DroneState(DRONE_SPECS[3], Vec3(0, 30, 0), Vec3(0, 0, 25))
        ovals = build_prediction_ovals(
            Vec3(0, 35, 150), Vec3(6, 0, 20), DRONE_SPECS[0], interceptor
        )
        self.assertEqual([oval.horizon_s for oval in ovals], [1, 2, 3, 5])
        self.assertTrue(all(len(oval.extremes) == 4 for oval in ovals))
        self.assertTrue(all(len(oval.reachable) == 4 for oval in ovals))


class ProjectTests(unittest.TestCase):
    def test_five_expandable_models_exist(self) -> None:
        self.assertEqual(len(DRONE_SPECS), 5)
        self.assertEqual(len({spec.code for spec in DRONE_SPECS}), 5)
        self.assertEqual(len(ROCKET_SPECS), 2)
        self.assertTrue(all(spec.vehicle_type == "rocket" for spec in ROCKET_SPECS))

    def test_every_vehicle_has_a_polygon_mesh(self) -> None:
        for spec in TARGET_SPECS:
            with self.subTest(vehicle=spec.name):
                mesh = get_mesh(spec.mesh_id)
                self.assertGreater(len(mesh.vertices), 8)
                self.assertGreater(len(mesh.faces), 6)

    def test_signal_lookup_precedes_identity(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="steady"))
        for _ in range(20):
            sim.step()
        self.assertFalse(sim.identity_confirmed)
        self.assertIn("SIGNAL", sim.status)
        for _ in range(60):
            sim.step()
        self.assertTrue(sim.identity_confirmed)

    def test_default_scenarios_intercept(self) -> None:
        for scenario, _ in SCENARIOS:
            with self.subTest(scenario=scenario):
                sim = InterceptionSimulation(SimulationConfig(scenario=scenario))
                if scenario == "rocket_attack":
                    sim = InterceptionSimulation(
                        SimulationConfig(target_code="SR1", scenario=scenario)
                    )
                for _ in range(60 * 30):
                    sim.step()
                    if sim.hit:
                        break
                self.assertTrue(sim.hit)

    def test_every_catalogue_target_can_be_intercepted(self) -> None:
        for target in DRONE_SPECS:
            with self.subTest(target=target.name):
                sim = InterceptionSimulation(
                    SimulationConfig(target_code=target.code, scenario="evasive")
                )
                for _ in range(60 * 40):
                    sim.step()
                    if sim.hit:
                        break
                self.assertTrue(sim.hit)

    def test_success_state_persists_during_crash(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="steady"))
        for _ in range(60 * 20):
            sim.step()
            if sim.hit:
                break
        self.assertTrue(sim.hit)
        for _ in range(30):
            sim.step()
        self.assertEqual(sim.status, "DRONE HIT SUCCESSFULLY")

    def test_configured_target_altitude_is_respected(self) -> None:
        config = SimulationConfig(
            target_position=Vec3(10, 80, 240),
            interceptor_position=Vec3(0, 60, 0),
            scenario="steady",
        )
        sim = InterceptionSimulation(config)
        for _ in range(60 * 3):
            sim.step()
        self.assertLess(abs(sim.target.position.y - 80.0), 2.0)

    def test_rocket_profiles_are_intercepted(self) -> None:
        for rocket in ROCKET_SPECS:
            with self.subTest(rocket=rocket.name):
                sim = InterceptionSimulation(
                    SimulationConfig(
                        target_code=rocket.code,
                        scenario="rocket_attack",
                    )
                )
                for _ in range(60 * 10):
                    sim.step()
                    if sim.hit:
                        break
                self.assertTrue(sim.hit)
                self.assertEqual(sim.status, "ROCKET INTERCEPTED SUCCESSFULLY")


if __name__ == "__main__":
    unittest.main()
