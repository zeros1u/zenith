from __future__ import annotations

import math
import unittest

import pygame

from app import DISPLAY_OPTIONS, SENSOR_OPTIONS, WINDOW_SIZE
from zenith.camera import (
    CameraModel,
    detect_box,
    estimate_range,
    minimum_horizontal_resolution,
    range_from_apparent_size,
)
from zenith.controls import ControlMode, ManualControlInput
from zenith.guidance import (
    TargetTrack,
    build_prediction_ovals,
    solve_guidance,
)
from zenith.math3d import Vec3, angle_between, basis_from_forward, clamp
from zenith.meshes import get_mesh
from zenith.models import DRONE_SPECS, ROCKET_SPECS, TARGET_SPECS
from zenith.physics import DroneState, maximum_travel_distance
from zenith.rendering import (
    GREEN,
    INFO_PAGE_COUNT,
    RED,
    ViewCamera,
    WorldRenderer,
)
from zenith.simulation import InterceptionSimulation, SCENARIOS, SimulationConfig


class VectorTests(unittest.TestCase):
    def test_camera_basis_is_orthonormal(self) -> None:
        right, up, forward = basis_from_forward(Vec3(1, 0.3, 2))
        self.assertAlmostEqual(right.length(), 1.0)
        self.assertAlmostEqual(up.length(), 1.0)
        self.assertAlmostEqual(forward.length(), 1.0)
        self.assertAlmostEqual(right.dot(up), 0.0, places=7)
        self.assertAlmostEqual(right.dot(forward), 0.0, places=7)


class InterfaceConfigTests(unittest.TestCase):
    def test_default_window_is_smaller_and_independent_from_sensor_resolution(self) -> None:
        self.assertEqual(WINDOW_SIZE, (1050, 700))
        self.assertEqual(DISPLAY_OPTIONS[0], WINDOW_SIZE)
        self.assertNotIn(WINDOW_SIZE, SENSOR_OPTIONS)
        self.assertEqual(SENSOR_OPTIONS[1], (1920, 1080))

    def test_mouse_free_look_does_not_change_sensor_direction(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        renderer = WorldRenderer()
        sensor_forward = Vec3(
            sim.camera_forward.x,
            sim.camera_forward.y,
            sim.camera_forward.z,
        )
        base_camera = renderer.get_view_camera(sim)
        renderer.rotate_view(120, -45)
        renderer.rotate_view(80, 0, roll_mode=True)
        moved_camera = renderer.get_view_camera(sim)
        self.assertGreater(
            angle_between(base_camera.forward, moved_camera.forward),
            math.radians(10),
        )
        self.assertNotEqual(moved_camera.roll_rad, 0.0)
        self.assertEqual(sim.camera_forward, sensor_forward)
        renderer.reset_view_offset()
        centered = renderer.get_view_camera(sim)
        self.assertAlmostEqual(
            angle_between(base_camera.forward, centered.forward),
            0.0,
            places=7,
        )

    def test_camera_roll_rotates_screen_axes(self) -> None:
        rolled = ViewCamera(Vec3(), Vec3(0, 0, 1), "TEST", math.pi * 0.5)
        projected = WorldRenderer._project(Vec3(1, 0, 10), rolled, 1000, 600)
        self.assertIsNotNone(projected)
        assert projected is not None
        self.assertAlmostEqual(projected[0], 500, delta=1)
        self.assertGreater(projected[1], 300)

    def test_manual_takeover_camera_follows_selected_vehicle(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        renderer = WorldRenderer()
        sim.cycle_control_mode()
        sim.cycle_control_mode()
        renderer.view_mode = 1
        camera = renderer.get_view_camera(sim)
        self.assertIs(sim.controlled_vehicle, sim.target)
        self.assertLess(
            camera.position.distance_to(sim.target.position),
            camera.position.distance_to(sim.interceptor.position),
        )
        self.assertIn(sim.target.spec.code, camera.name)

    def test_spectator_camera_is_not_an_onboard_view(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        renderer = WorldRenderer()
        renderer.view_mode = 2
        camera = renderer.get_view_camera(sim)
        self.assertIn("SPECTATOR", camera.name)
        self.assertGreater(camera.position.distance_to(sim.interceptor.position), 10.0)
        self.assertGreater(camera.position.distance_to(sim.target.position), 10.0)

    def test_full_information_pages_render_at_minimum_window_size(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        renderer = WorldRenderer()
        surface = pygame.Surface(WINDOW_SIZE)
        self.assertEqual(INFO_PAGE_COUNT, 4)
        for page in range(INFO_PAGE_COUNT):
            renderer.draw_info(surface, sim, page)


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

    def test_entire_oval_color_reports_four_point_reachability(self) -> None:
        interceptor = DroneState(DRONE_SPECS[3], Vec3(0, 30, 0), Vec3(0, 0, 25))
        oval = build_prediction_ovals(
            Vec3(0, 35, 150),
            Vec3(6, 0, 20),
            DRONE_SPECS[0],
            interceptor,
        )[0]
        oval.reachable = (True, True, True, True)
        self.assertEqual(WorldRenderer._oval_reachability_color(oval), GREEN)
        oval.reachable = (True, False, False, False)
        self.assertEqual(WorldRenderer._oval_reachability_color(oval), RED)
        oval.reachable = (False, False, False, False)
        self.assertEqual(WorldRenderer._oval_reachability_color(oval), RED)

    def test_oval_plane_is_perpendicular_to_camera_view(self) -> None:
        interceptor = DroneState(DRONE_SPECS[3], Vec3(0, 30, 0), Vec3(0, 0, 25))
        view = Vec3(1.0, 0.25, 2.0).normalized()
        ovals = build_prediction_ovals(
            Vec3(0, 35, 150),
            Vec3(6, 0, 20),
            DRONE_SPECS[0],
            interceptor,
            view,
        )
        for oval in ovals:
            self.assertAlmostEqual(oval.plane_normal.dot(view), 1.0, places=7)
            for edge in (
                oval.center,
                oval.ballistic_center,
                *oval.extremes,
            ):
                self.assertAlmostEqual(
                    (edge - Vec3(0, 35, 150)).dot(oval.plane_normal),
                    0.0,
                    places=7,
                )
            for edge in oval.extremes:
                self.assertAlmostEqual(
                    (edge - oval.center).dot(oval.plane_normal),
                    0.0,
                    places=7,
                )
            averaged_extremes = (
                oval.extremes[0]
                + oval.extremes[1]
                + oval.extremes[2]
                + oval.extremes[3]
            ) / 4.0
            self.assertLess(
                averaged_extremes.distance_to(oval.center),
                1e-8,
            )
            self.assertTrue(oval.contains_projected(oval.ballistic_center))

    def test_rendered_border_satisfies_ellipse_equation(self) -> None:
        interceptor = DroneState(
            DRONE_SPECS[3],
            Vec3(0, 30, 0),
            Vec3(0, 0, 25),
        )
        oval = build_prediction_ovals(
            Vec3(0, 35, 150),
            Vec3(6, 0, 20),
            DRONE_SPECS[1],
            interceptor,
            Vec3(0.2, 0.1, 1.0).normalized(),
        )[2]
        self.assertNotAlmostEqual(oval.radius_x, oval.radius_y, places=4)
        for point in WorldRenderer._oval_points(oval):
            offset = point - oval.center
            x = offset.dot(oval.plane_x) / oval.radius_x
            y = offset.dot(oval.plane_y) / oval.radius_y
            self.assertAlmostEqual(x * x + y * y, 1.0, places=7)

    def test_guidance_aims_at_largest_fully_reachable_oval_center(self) -> None:
        interceptor = DroneState(
            DRONE_SPECS[2],
            Vec3(0, 30, 0),
            Vec3(0, 0, DRONE_SPECS[2].max_speed),
        )
        track = TargetTrack(
            position=Vec3(0, 35, 125),
            last_measurement=Vec3(0, 35, 125),
            velocity=Vec3(0, 0, 2),
            sample_count=5,
        )
        solution = solve_guidance(
            interceptor,
            track,
            DRONE_SPECS[4],
            Vec3(0, 0, 1),
        )
        self.assertIsNotNone(solution)
        assert solution is not None
        self.assertEqual(solution.mode, "OVAL CENTER")
        selected = next(
            oval
            for oval in solution.ovals
            if oval.horizon_s == solution.selected_horizon_s
        )
        self.assertTrue(selected.fully_reachable)
        self.assertLess(solution.aim_point.distance_to(selected.center), 1e-8)

    def test_conservative_envelope_contains_allowed_acceleration_samples(self) -> None:
        interceptor = DroneState(DRONE_SPECS[3], Vec3(0, 30, 0), Vec3(0, 0, 25))
        view = Vec3(1.0, 0.25, 2.0).normalized()
        position = Vec3(0, 35, 150)
        velocity = Vec3(6, 0, 20)
        horizon = 2.0
        oval = build_prediction_ovals(
            position, velocity, DRONE_SPECS[1], interceptor, view
        )[1]
        forward = velocity.normalized()
        lateral_a, lateral_b, _ = basis_from_forward(forward)
        ballistic = position + velocity * horizon
        for axial in (-DRONE_SPECS[1].brake_accel, 0.0, DRONE_SPECS[1].max_accel):
            for index in range(24):
                angle = math.tau * index / 24
                lateral = (
                    lateral_a * math.cos(angle) + lateral_b * math.sin(angle)
                ) * DRONE_SPECS[1].lateral_accel
                endpoint = (
                    ballistic
                    + (forward * axial + lateral) * (0.5 * horizon**2)
                )
                self.assertTrue(oval.contains_projected(endpoint, 1e-7))

        multirotor = DRONE_SPECS[0]
        multirotor_oval = build_prediction_ovals(
            position, velocity, multirotor, interceptor, view
        )[1]
        for elevation_index in range(13):
            elevation = -math.pi * 0.5 + math.pi * elevation_index / 12
            for azimuth_index in range(24):
                azimuth = math.tau * azimuth_index / 24
                acceleration = Vec3(
                    math.cos(azimuth)
                    * math.cos(elevation)
                    * multirotor.lateral_accel,
                    math.sin(elevation) * multirotor.max_accel,
                    math.sin(azimuth)
                    * math.cos(elevation)
                    * multirotor.lateral_accel,
                )
                endpoint = ballistic + acceleration * (0.5 * horizon**2)
                self.assertTrue(
                    multirotor_oval.contains_projected(endpoint, 1e-7)
                )


class PropulsionTests(unittest.TestCase):
    def test_fixed_wing_turn_is_rate_limited_and_thrust_is_axial(self) -> None:
        spec = DRONE_SPECS[3]
        state = DroneState(spec, Vec3(0, 30, 0), Vec3(0, 0, 30))
        previous_forward = state.velocity.normalized()
        dt = 1.0 / 60.0
        state.integrate(Vec3(100, 0, 12), dt)
        new_forward = state.velocity.normalized()
        allowed_rate = min(
            math.radians(spec.max_turn_rate_deg),
            spec.lateral_accel / 30.0,
        )
        self.assertLessEqual(
            angle_between(previous_forward, new_forward),
            allowed_rate * dt + 1e-8,
        )
        self.assertAlmostEqual(state.thrust_vector.x, 0.0, places=7)
        self.assertGreater(state.thrust_vector.z, 0.0)

    def test_rocket_cannot_apply_reverse_engine_thrust(self) -> None:
        spec = ROCKET_SPECS[0]
        state = DroneState(spec, Vec3(0, 30, 0), Vec3(0, 0, 60))
        state.integrate(Vec3(0, 0, -100), 1.0 / 60.0)
        self.assertAlmostEqual(state.thrust_vector.length(), 0.0, places=7)
        self.assertEqual(state.engine_output, 0.0)

    def test_every_unpowered_vehicle_class_receives_gravity(self) -> None:
        for spec in (
            DRONE_SPECS[0],
            DRONE_SPECS[1],
            DRONE_SPECS[4],
            ROCKET_SPECS[0],
        ):
            with self.subTest(flight_model=spec.flight_model):
                state = DroneState(spec, Vec3(0, 100, 0), Vec3())
                state.engine_enabled = False
                state.integrate(Vec3(), 0.1)
                self.assertLess(state.position.y, 100.0)
                self.assertLess(state.acceleration.y, 0.0)

    def test_powered_multirotor_hovers_but_engine_cut_falls(self) -> None:
        powered = DroneState(DRONE_SPECS[0], Vec3(0, 100, 0), Vec3())
        unpowered = DroneState(DRONE_SPECS[0], Vec3(0, 100, 0), Vec3())
        unpowered.engine_enabled = False
        for _ in range(60):
            powered.integrate(Vec3(), 1.0 / 60.0)
            unpowered.integrate(Vec3(), 1.0 / 60.0)
        self.assertAlmostEqual(powered.position.y, 100.0, delta=0.1)
        self.assertLess(unpowered.position.y, powered.position.y - 3.0)
        self.assertEqual(unpowered.engine_output, 0.0)

    def test_horizontal_wing_glides_slower_than_unlifted_rocket_falls(self) -> None:
        wing = DroneState(
            DRONE_SPECS[1],
            Vec3(0, 100, 0),
            Vec3(0, 0, 30),
        )
        rocket = DroneState(
            ROCKET_SPECS[0],
            Vec3(0, 100, 0),
            Vec3(0, 0, 30),
        )
        wing.engine_enabled = False
        rocket.engine_enabled = False
        for _ in range(60):
            wing.integrate(Vec3(), 1.0 / 60.0)
            rocket.integrate(Vec3(), 1.0 / 60.0)
        self.assertGreater(wing.lift_acceleration.length(), 0.0)
        self.assertAlmostEqual(rocket.lift_acceleration.length(), 0.0)
        self.assertGreater(wing.position.y, rocket.position.y + 3.0)

    def test_wing_lift_weakens_below_stall_speed(self) -> None:
        slow = DroneState(
            DRONE_SPECS[1],
            Vec3(0, 100, 0),
            Vec3(0, 0, 5),
        )
        fast = DroneState(
            DRONE_SPECS[1],
            Vec3(0, 100, 0),
            Vec3(0, 0, 30),
        )
        slow.engine_enabled = False
        fast.engine_enabled = False
        slow.integrate(Vec3(), 1.0 / 60.0)
        fast.integrate(Vec3(), 1.0 / 60.0)
        self.assertLess(
            slow.lift_acceleration.length(),
            fast.lift_acceleration.length() * 0.2,
        )


class ManualControlTests(unittest.TestCase):
    def test_tab_cycles_auto_own_target_auto(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        self.assertIs(sim.control_mode, ControlMode.AUTO)
        self.assertIs(sim.cycle_control_mode(), ControlMode.INTERCEPTOR)
        self.assertIs(sim.controlled_vehicle, sim.interceptor)
        self.assertIs(sim.cycle_control_mode(), ControlMode.TARGET)
        self.assertIs(sim.controlled_vehicle, sim.target)
        self.assertIs(sim.cycle_control_mode(), ControlMode.AUTO)
        self.assertIsNone(sim.controlled_vehicle)

    def test_player_can_cut_and_restart_selected_engine(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        self.assertIsNone(sim.toggle_controlled_engine())
        sim.cycle_control_mode()
        self.assertFalse(sim.toggle_controlled_engine())
        self.assertFalse(sim.interceptor.engine_enabled)
        self.assertTrue(sim.toggle_controlled_engine())
        self.assertTrue(sim.interceptor.engine_enabled)

    def test_player_own_command_overrides_but_preserves_guidance_advisory(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="steady"))
        for _ in range(120):
            sim.step()
            if sim.guidance is not None:
                break
        self.assertIsNotNone(sim.guidance)
        sim.cycle_control_mode()
        sim.set_manual_input(ManualControlInput())
        sim.step()
        self.assertGreater(sim.guidance_advisory_command.length(), 0.0)
        self.assertAlmostEqual(sim.manual_command.acceleration.x, 0.0, places=7)
        self.assertAlmostEqual(sim.manual_command.acceleration.z, 0.0, places=7)
        self.assertLess(
            sim.manual_command.acceleration.length(),
            sim.guidance_advisory_command.length(),
        )
        self.assertIn("PLAYER CONTROL", sim.status)

    def test_multirotor_player_can_turn_heading_independently(self) -> None:
        sim = InterceptionSimulation(
            SimulationConfig(target_code="FX1", scenario="steady")
        )
        sim.cycle_control_mode()
        sim.cycle_control_mode()
        initial_yaw = sim.target.orientation.y
        sim.set_manual_input(
            ManualControlInput(forward=1.0, turn=1.0)
        )
        for _ in range(60):
            sim.step()
        self.assertGreater(
            abs(sim.target.orientation.y - initial_yaw),
            math.radians(20.0),
        )
        self.assertLessEqual(
            sim.manual_command.acceleration.length(),
            max(
                sim.target.spec.max_accel,
                sim.target.spec.lateral_accel,
            )
            + 1e-7,
        )

    def test_rocket_reverse_and_airbrake_are_unavailable(self) -> None:
        sim = InterceptionSimulation(
            SimulationConfig(target_code="SR1", scenario="rocket_attack")
        )
        sim.cycle_control_mode()
        sim.cycle_control_mode()
        sim.set_manual_input(
            ManualControlInput(forward=-1.0, brake=True)
        )
        sim.step()
        self.assertFalse(sim.manual_command.brake_available)
        self.assertFalse(sim.target.airbrake)
        self.assertGreaterEqual(
            sim.manual_command.acceleration.dot(
                sim.target.forward_direction()
            ),
            -1e-7,
        )

    def test_manual_altitude_floor_blocks_descent(self) -> None:
        sim = InterceptionSimulation(
            SimulationConfig(
                target_code="FX1",
                target_position=Vec3(10.0, 2.1, 240.0),
                scenario="steady",
            )
        )
        sim.cycle_control_mode()
        sim.cycle_control_mode()
        sim.set_manual_input(ManualControlInput(vertical=-1.0))
        sim.step()
        self.assertTrue(sim.manual_command.floor_protection)
        self.assertGreaterEqual(sim.manual_command.acceleration.y, 0.0)

    def test_player_own_control_overrides_automatic_search_turn(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="evasive"))
        for _ in range(150):
            sim.step()
            if sim.guidance is not None:
                break
        sim.cycle_control_mode()
        sim.toggle_sensor_occlusion()
        sim.set_manual_input(ManualControlInput())
        for _ in range(10):
            sim.step()
        self.assertFalse(sim.visual_locked)
        self.assertIsNone(sim.guidance)
        self.assertAlmostEqual(sim.manual_command.acceleration.x, 0.0, places=7)
        self.assertAlmostEqual(sim.manual_command.acceleration.z, 0.0, places=7)
        self.assertGreater(sim.lost_time_s, 0.0)


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

    def test_loss_disables_guidance_until_visual_reacquisition(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="steady"))
        for _ in range(120):
            sim.step()
            if sim.identity_confirmed and sim.guidance is not None:
                break
        self.assertTrue(sim.visual_locked)
        self.assertIsNotNone(sim.guidance)

        sim.toggle_sensor_occlusion()
        sim.step()
        self.assertFalse(sim.visual_locked)
        self.assertIsNone(sim.guidance)
        self.assertIn("SEARCHING", sim.status)

        for _ in range(8):
            sim.step()
        sim.toggle_sensor_occlusion()
        for _ in range(120):
            sim.step()
            if sim.visual_locked and sim.guidance is not None:
                break
        self.assertTrue(sim.visual_locked)
        self.assertIsNotNone(sim.guidance)
        self.assertGreaterEqual(sim.reacquisition_count, 1)

    def test_lost_target_search_is_horizon_limited(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="evasive"))
        for _ in range(150):
            sim.step()
            if sim.guidance is not None:
                break
        held_altitude = sim.interceptor.position.y
        sim.toggle_sensor_occlusion()
        for _ in range(100):
            sim.step()
        elevation = math.degrees(
            math.asin(clamp(sim.search_direction.y, -1.0, 1.0))
        )
        self.assertLessEqual(abs(elevation), 35.0 + 1e-7)
        self.assertAlmostEqual(sim.search_hold_altitude_m, held_altitude, places=5)
        self.assertFalse(sim.visual_locked)
        self.assertIsNone(sim.guidance)

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
