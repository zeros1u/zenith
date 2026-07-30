from __future__ import annotations

import math
import unittest
from unittest.mock import patch

import pygame

from app import (
    DEFAULT_TIME_SCALE_INDEX,
    DISPLAY_OPTIONS,
    SENSOR_OPTIONS,
    SetupScreen,
    TIME_SCALES,
    WINDOW_SIZE,
)
from zenith.camera import (
    CameraModel,
    Detection,
    detect_box,
    estimate_range,
    minimum_horizontal_resolution,
    range_from_apparent_size,
)
from zenith.controls import ControlMode, ManualControlInput
from zenith.guidance import (
    PredictionOval,
    TargetTrack,
    build_prediction_ovals,
    shared_oval_overlap,
    solve_guidance,
)
from zenith.math3d import Vec3, angle_between, basis_from_forward, clamp
from zenith.meshes import get_mesh
from zenith.models import (
    DRONE_SPECS,
    INTERCEPTOR_SPECS,
    ROCKET_SPECS,
    TARGET_SPECS,
)
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
        self.assertEqual(TIME_SCALES[DEFAULT_TIME_SCALE_INDEX], 0.5)
        self.assertEqual(CameraModel().horizontal_fov_deg, 90.0)

    def test_setup_can_request_three_separately_tracked_enemies(self) -> None:
        setup = SetupScreen(WINDOW_SIZE)
        setup.enemy_count = 3
        config = setup.build_config()
        self.assertIsNotNone(config)
        assert config is not None
        self.assertEqual(config.enemy_count, 3)

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

    def test_mouse_wheel_zoom_is_presentation_only_and_bounded(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        renderer = WorldRenderer()
        camera = renderer.get_view_camera(sim)
        sensor_fov = sim.config.camera.horizontal_fov_deg
        sensor_forward = sim.camera_forward
        point = camera.position + camera.forward * 100.0 + Vec3(10.0, 0.0, 0.0)
        before = renderer._project_cached(point, camera, 1050, 700)
        renderer.adjust_zoom(4)
        after = renderer._project_cached(point, camera, 1050, 700)
        self.assertIsNotNone(before)
        self.assertIsNotNone(after)
        assert before is not None and after is not None
        self.assertGreater(abs(after[0] - 525), abs(before[0] - 525))
        self.assertGreater(renderer.zoom_multiplier, 1.0)
        self.assertEqual(sim.config.camera.horizontal_fov_deg, sensor_fov)
        self.assertEqual(sim.camera_forward, sensor_forward)
        renderer.adjust_zoom(100)
        self.assertEqual(
            renderer.presentation_fov_deg,
            renderer.MIN_PRESENTATION_FOV_DEG,
        )
        renderer.adjust_zoom(-200)
        self.assertEqual(
            renderer.presentation_fov_deg,
            renderer.MAX_PRESENTATION_FOV_DEG,
        )
        renderer.reset_view_offset()
        self.assertEqual(
            renderer.presentation_fov_deg,
            renderer.DEFAULT_PRESENTATION_FOV_DEG,
        )

    def test_camera_roll_rotates_screen_axes(self) -> None:
        rolled = ViewCamera(Vec3(), Vec3(0, 0, 1), "TEST", math.pi * 0.5)
        projected = WorldRenderer._project(Vec3(1, 0, 10), rolled, 1000, 600)
        self.assertIsNotNone(projected)
        assert projected is not None
        self.assertAlmostEqual(projected[0], 500, delta=1)
        self.assertGreater(projected[1], 300)

    def test_cached_projection_matches_reference_projection(self) -> None:
        renderer = WorldRenderer()
        camera = ViewCamera(
            Vec3(4.0, 12.0, -7.0),
            Vec3(0.2, -0.08, 1.0),
            "CACHE TEST",
            0.31,
        )
        for point in (
            Vec3(-8.0, 4.0, 20.0),
            Vec3(5.0, 18.0, 80.0),
            Vec3(42.0, 1.0, 190.0),
            Vec3(4.0, 12.0, -20.0),
        ):
            with self.subTest(point=point):
                self.assertEqual(
                    renderer._project_cached(point, camera, 1050, 700),
                    WorldRenderer._project(point, camera, 1050, 700),
                )

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

    def test_collapsible_panels_and_widgets_render_at_minimum_size(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        renderer = WorldRenderer()
        surface = pygame.Surface(WINDOW_SIZE)
        renderer.draw_hud(surface, sim, 0.5, 60.0)
        self.assertTrue(renderer.panel_expanded["target"])
        self.assertTrue(renderer.panel_expanded["calculations"])
        calculations_header = renderer.click_regions["panel:calculations"]
        renderer.handle_left_click(calculations_header.center, sim)
        self.assertFalse(renderer.panel_expanded["calculations"])
        renderer.minimap_visible = True
        renderer.settings_visible = True
        renderer.verification_visible = True
        renderer.draw_hud(surface, sim, 0.5, 60.0)
        self.assertIn("minimap", renderer.click_regions)
        self.assertIn("terminal_mode", renderer.click_regions)
        self.assertIn("capture_check", renderer.click_regions)

    def test_sensor_boresight_only_draws_in_onboard_auto_view(self) -> None:
        sim = InterceptionSimulation(SimulationConfig())
        renderer = WorldRenderer()
        onboard = pygame.Surface((400, 300))
        onboard.fill((0, 0, 0))
        renderer.view_mode = 0
        renderer._draw_reticle(onboard, sim)
        self.assertNotEqual(
            pygame.image.tostring(onboard, "RGB"),
            bytes(len(pygame.image.tostring(onboard, "RGB"))),
        )
        spectator = pygame.Surface((400, 300))
        spectator.fill((0, 0, 0))
        renderer.view_mode = 2
        renderer._draw_reticle(spectator, sim)
        self.assertEqual(
            pygame.image.tostring(spectator, "RGB"),
            bytes(len(pygame.image.tostring(spectator, "RGB"))),
        )


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
        self.assertIsNotNone(detection.pose_estimate)
        estimate = estimate_range(
            detection,
            spec,
            detection.pose_estimate or Vec3(),
            forward,
            camera,
        )
        self.assertIsNotNone(estimate)
        assert estimate is not None
        error = abs(estimate.distance_m - origin.distance_to(target.position))
        # At this distance the target is only a few pixels wide. The error must
        # remain inside the estimator's explicit pixel-quantisation uncertainty.
        self.assertLess(error, estimate.sigma_m)

    def test_subpixel_box_change_does_not_create_a_range_step(self) -> None:
        camera = CameraModel(1920, 1080, 90)
        spec = DRONE_SPECS[0]

        def detection_with_span(span_px: float) -> Detection:
            return Detection(
                True,
                (960.0 - span_px * 0.5, 539.0, 960.0 + span_px * 0.5, 541.0),
                (960.0, 540.0),
                span_px,
                2.0,
                0.0,
                0.0,
                0.1,
                200.0,
                (),
                Vec3(),
            )

        before = estimate_range(
            detection_with_span(3.49),
            spec,
            Vec3(),
            Vec3(0.0, 0.0, 1.0),
            camera,
        )
        after = estimate_range(
            detection_with_span(3.50),
            spec,
            Vec3(),
            Vec3(0.0, 0.0, 1.0),
            camera,
        )
        self.assertIsNotNone(before)
        self.assertIsNotNone(after)
        assert before is not None and after is not None
        relative_change = abs(after.distance_m - before.distance_m) / before.distance_m
        self.assertLess(relative_change, 0.01)

    def test_simulation_consumes_reported_pose_not_target_orientation(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="steady"))
        sim.identity_confirmed = True
        reported_pose = Vec3(0.12, -0.34, 0.56)
        detector_output = Detection(
            True,
            (950.0, 535.0, 970.0, 545.0),
            (960.0, 540.0),
            20.0,
            10.0,
            0.0,
            0.0,
            1.0,
            100.0,
            (),
            reported_pose,
        )
        with (
            patch("zenith.simulation.detect_box", return_value=detector_output),
            patch("zenith.simulation.estimate_range", return_value=None)
            as range_solver,
        ):
            sim._update_sensor(1.0 / 60.0)
        self.assertEqual(range_solver.call_args.args[2], reported_pose)

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
        self.assertTrue(all(len(oval.edge_points) == 96 for oval in ovals))
        self.assertTrue(all(len(oval.edge_reachable) == 96 for oval in ovals))

    def test_entire_oval_color_reports_complete_edge_reachability(self) -> None:
        interceptor = DroneState(DRONE_SPECS[3], Vec3(0, 30, 0), Vec3(0, 0, 25))
        oval = build_prediction_ovals(
            Vec3(0, 35, 150),
            Vec3(6, 0, 20),
            DRONE_SPECS[0],
            interceptor,
        )[0]
        oval.reachable = (True, True, True, True)
        oval.edge_reachable = tuple(True for _ in oval.edge_points)
        self.assertEqual(WorldRenderer._oval_reachability_color(oval), GREEN)
        # Four green cardinals do not hide a diagonal escape direction.
        edge = list(oval.edge_reachable)
        edge[len(edge) // 8] = False
        oval.edge_reachable = tuple(edge)
        self.assertEqual(WorldRenderer._oval_reachability_color(oval), RED)
        edge_colors = WorldRenderer._oval_edge_colors(oval)
        self.assertEqual(edge_colors.count(GREEN), len(edge) - 1)
        self.assertEqual(edge_colors.count(RED), 1)
        # A blocked outer edge must never paint a misleading red interior over
        # a smaller green oval.
        self.assertIsNone(WorldRenderer._oval_fill_color(oval))
        oval.edge_reachable = tuple(True for _ in oval.edge_points)
        self.assertEqual(WorldRenderer._oval_fill_color(oval), GREEN)
        oval.reachable = (False, False, False, False)
        oval.edge_reachable = tuple(False for _ in oval.edge_points)
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

    def test_shared_overlap_requires_matching_horizons_and_mutual_centers(self) -> None:
        observer = Vec3()

        def oval(
            horizon: float,
            center: Vec3,
            radius_x: float,
            radius_y: float,
        ) -> PredictionOval:
            plane_x = Vec3(1, 0, 0)
            plane_y = Vec3(0, 1, 0)
            plane_normal = Vec3(0, 0, 1)
            extremes = (
                center + plane_x * radius_x,
                center - plane_x * radius_x,
                center + plane_y * radius_y,
                center - plane_y * radius_y,
            )
            return PredictionOval(
                horizon_s=horizon,
                center=center,
                ballistic_center=center,
                extremes=extremes,
                reachable=(True, True, True, True),
                plane_x=plane_x,
                plane_y=plane_y,
                plane_normal=plane_normal,
                radius_x=radius_x,
                radius_y=radius_y,
                edge_points=extremes,
                edge_reachable=(True, True, True, True),
                observer_position=observer,
            )

        first = oval(2.0, Vec3(0, 0, 100), 30, 20)
        second = oval(2.0, Vec3(10, 0, 120), 48, 18)
        self.assertNotEqual(first.radius_x, second.radius_x)
        self.assertNotEqual(first.radius_y, second.radius_y)
        overlap = shared_oval_overlap(first, second, observer)
        self.assertIsNotNone(overlap)
        assert overlap is not None
        self.assertEqual(overlap.horizon_s, 2.0)
        self.assertGreater(overlap.normalized_area, 0.0)
        self.assertGreater(len(overlap.polygon_world), 8)
        self.assertTrue(first.contains_projected(overlap.aim_point))
        self.assertTrue(second.contains_projected(overlap.aim_point))

        mismatched_horizon = oval(3.0, Vec3(10, 0, 120), 48, 18)
        self.assertIsNone(
            shared_oval_overlap(first, mismatched_horizon, observer)
        )
        non_mutual = oval(2.0, Vec3(80, 0, 120), 20, 18)
        self.assertIsNone(shared_oval_overlap(first, non_mutual, observer))

    def test_guidance_aims_inside_largest_fully_reachable_weighted_oval(self) -> None:
        interceptor = DroneState(
            DRONE_SPECS[2],
            Vec3(0, 30, 0),
            Vec3(0, 0, DRONE_SPECS[2].max_speed),
        )
        track = TargetTrack(
            position=Vec3(0, 35, 125),
            last_measurement=Vec3(0, 35, 125),
            velocity=Vec3(0, 0, 2),
            acceleration=Vec3(5, 1, 0),
            sample_count=30,
            confidence=1.0,
        )
        solution = solve_guidance(
            interceptor,
            track,
            DRONE_SPECS[4],
            Vec3(0, 0, 1),
        )
        self.assertIsNotNone(solution)
        assert solution is not None
        self.assertEqual(solution.mode, "WEIGHTED OVAL")
        selected = next(
            oval
            for oval in solution.ovals
            if oval.horizon_s == solution.selected_horizon_s
        )
        self.assertTrue(selected.fully_reachable)
        offset = solution.aim_point - selected.center
        normalized = math.hypot(
            offset.dot(selected.plane_x) / selected.radius_x,
            offset.dot(selected.plane_y) / selected.radius_y,
        )
        self.assertLessEqual(normalized, 0.65 + 1e-8)
        self.assertGreater(solution.aim_point.distance_to(selected.center), 0.01)

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

    def test_approaching_camera_crossing_is_reported_unbounded(self) -> None:
        interceptor = DroneState(
            DRONE_SPECS[3],
            Vec3(0, 30, 0),
            Vec3(0, 0, 25),
        )
        ovals = build_prediction_ovals(
            Vec3(0, 35, 120),
            Vec3(0, 0, -70),
            ROCKET_SPECS[0],
            interceptor,
            Vec3(0, 0, 1),
        )
        self.assertIsNone(ovals[0].invalid_reason)
        self.assertTrue(
            any(oval.invalid_reason == "CAMERA CROSSING" for oval in ovals[1:])
        )
        self.assertTrue(
            all(
                not oval.fully_reachable
                for oval in ovals
                if oval.invalid_reason is not None
            )
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

    def test_rocket_ignores_reverse_request_and_keeps_full_booster(self) -> None:
        spec = ROCKET_SPECS[0]
        state = DroneState(spec, Vec3(0, 30, 0), Vec3(0, 0, 60))
        state.integrate(Vec3(0, 0, -100), 1.0 / 60.0)
        self.assertAlmostEqual(
            state.thrust_vector.length(),
            spec.max_accel,
            places=7,
        )
        self.assertEqual(state.engine_output, 1.0)

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

    def test_downward_vectored_command_cannot_reflect_into_a_climb(self) -> None:
        for spec in (DRONE_SPECS[2], DRONE_SPECS[5]):
            with self.subTest(vehicle=spec.name):
                state = DroneState(
                    spec,
                    Vec3(0, 100, 0),
                    Vec3(0, 0, 18),
                )
                initial_altitude = state.position.y
                downward_turn = Vec3(
                    spec.lateral_accel,
                    -spec.max_accel,
                    spec.lateral_accel * 0.5,
                )
                for _ in range(60):
                    state.integrate(downward_turn, 1.0 / 60.0)
                self.assertLess(state.position.y, initial_altitude - 0.5)
                self.assertLess(state.velocity.y, 0.0)

    def test_forward_vectored_thrust_tilts_the_nose_down(self) -> None:
        for spec in (DRONE_SPECS[2], DRONE_SPECS[5]):
            with self.subTest(vehicle=spec.name):
                state = DroneState(
                    spec,
                    Vec3(0, 100, 0),
                    Vec3(0, 0, spec.max_speed * 0.5),
                )
                for _ in range(30):
                    state.integrate(
                        Vec3(0, 0, spec.lateral_accel * 0.5),
                        1.0 / 60.0,
                    )
                self.assertGreater(state.orientation.x, 0.0)
                self.assertLess(state.forward_direction().y, 0.0)

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

    def test_rocket_booster_burns_out_permanently(self) -> None:
        spec = ROCKET_SPECS[0]
        state = DroneState(
            spec,
            Vec3(0, 1000, 0),
            Vec3(0, 0, 20),
        )
        for _ in range(int(spec.main_burn_duration_s * 60) + 2):
            state.integrate(Vec3(0, 0, spec.max_accel), 1.0 / 60.0)
        self.assertTrue(state.burned_out)
        self.assertFalse(state.engine_enabled)
        self.assertEqual(state.main_burn_remaining_s, 0.0)
        state.integrate(Vec3(0, 0, spec.max_accel), 1.0 / 60.0)
        self.assertEqual(state.engine_output, 0.0)
        self.assertEqual(state.thrust_vector.length(), 0.0)

    def test_rocket_rcs_has_separate_limited_supply(self) -> None:
        spec = ROCKET_SPECS[0]
        state = DroneState(
            spec,
            Vec3(0, 1000, 0),
            Vec3(0, 0, 30),
        )
        initial_rcs = state.rcs_remaining_s
        state.integrate(Vec3(spec.lateral_accel, 0, 0), 0.25)
        self.assertLess(state.rcs_remaining_s, initial_rcs)
        burn_after_turn = state.main_burn_remaining_s
        state.integrate(
            state.velocity.normalized(state.forward_direction()) * spec.max_accel,
            0.25,
        )
        self.assertAlmostEqual(state.rcs_remaining_s, initial_rcs - 0.25)
        self.assertLess(state.main_burn_remaining_s, burn_after_turn)


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

    def test_player_cannot_cut_or_restart_fixed_rocket_booster(self) -> None:
        sim = InterceptionSimulation(
            SimulationConfig(interceptor_code="SR1", scenario="steady")
        )
        sim.cycle_control_mode()
        remaining = sim.interceptor.main_burn_remaining_s
        self.assertTrue(sim.toggle_controlled_engine())
        self.assertTrue(sim.interceptor.engine_enabled)
        self.assertEqual(sim.interceptor.main_burn_remaining_s, remaining)
        self.assertIn("fixed nonrestartable", sim.last_event)

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
    def test_six_expandable_models_exist(self) -> None:
        self.assertEqual(len(DRONE_SPECS), 6)
        self.assertEqual(len({spec.code for spec in DRONE_SPECS}), 6)
        self.assertEqual(len(ROCKET_SPECS), 2)
        self.assertTrue(all(spec.vehicle_type == "rocket" for spec in ROCKET_SPECS))
        self.assertEqual(INTERCEPTOR_SPECS, TARGET_SPECS)
        smart_evader = next(spec for spec in DRONE_SPECS if spec.code == "SEV")
        wraith = next(spec for spec in DRONE_SPECS if spec.code == "WRS")
        self.assertEqual(smart_evader.name, "SMART EVADER")
        self.assertEqual(smart_evader.mesh_id, "smart_evader_ufo")
        self.assertEqual(wraith.max_speed, max(spec.max_speed for spec in DRONE_SPECS))
        self.assertGreater(smart_evader.max_accel, wraith.max_accel)
        self.assertGreater(smart_evader.brake_accel, wraith.brake_accel)
        self.assertGreater(smart_evader.max_turn_rate_deg, wraith.max_turn_rate_deg)

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

    def test_sensor_is_fixed_to_nose_and_cannot_lock_behind_airframe(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="steady"))
        for _ in range(150):
            sim.step()
            if sim.identity_confirmed and sim.visual_locked:
                break
        self.assertTrue(sim.visual_locked)

        away = (
            sim.interceptor.position - sim.target.position
        ).normalized(Vec3(0, 0, -1))
        sim.interceptor.velocity = away * max(
            20.0,
            sim.interceptor.velocity.length(),
        )
        sim.interceptor.orientation = Vec3(
            -math.asin(clamp(away.y, -1.0, 1.0)),
            math.atan2(away.x, away.z),
            0.0,
        )
        sim.step()

        nose = sim.interceptor.forward_direction()
        target_direction = (
            sim.target.position - sim.interceptor.position
        ).normalized()
        self.assertGreater(sim.camera_forward.dot(nose), 0.999999)
        self.assertLess(sim.camera_forward.dot(target_direction), 0.0)
        self.assertFalse(sim.visual_locked)
        self.assertIsNone(sim.guidance)

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

    def test_tricky_ai_is_reactive_deterministic_and_physically_limited(self) -> None:
        self.assertIn(("tricky", "TRICKY AI"), SCENARIOS)
        first = InterceptionSimulation(
            SimulationConfig(target_code="SEV", scenario="tricky")
        )
        second = InterceptionSimulation(
            SimulationConfig(target_code="SEV", scenario="tricky")
        )
        decisions: set[str] = set()
        airbrake_seen = False
        for _ in range(60 * 15):
            first.step()
            second.step()
            decisions.add(first.evader_decision)
            airbrake_seen = airbrake_seen or first.target.airbrake
            self.assertEqual(first.evader_decision, second.evader_decision)
            self.assertLess(
                first.target.position.distance_to(second.target.position),
                1e-9,
            )
            self.assertLessEqual(
                first.target.velocity.length(),
                first.target.spec.max_speed + 1e-7,
            )
            if first.hit:
                break
        self.assertTrue(first.hit)
        self.assertGreaterEqual(len(decisions), 4)
        self.assertTrue(airbrake_seen)
        self.assertGreater(first.evader_decision_index, 3)

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

    def test_tricky_mode_is_a_real_but_bounded_stress_case(self) -> None:
        intercepted = 0
        escaped = 0
        smart_evader = next(spec for spec in DRONE_SPECS if spec.code == "SEV")
        for interceptor in DRONE_SPECS:
            with self.subTest(interceptor=interceptor.name):
                sim = InterceptionSimulation(
                    SimulationConfig(
                        interceptor_code=interceptor.code,
                        target_code=smart_evader.code,
                        scenario="tricky",
                    )
                )
                for _ in range(60 * 30):
                    sim.step()
                    if sim.hit:
                        break
                self.assertGreater(sim.evader_decision_index, 1)
                self.assertLessEqual(
                    sim.target.velocity.length(),
                    sim.target.spec.max_speed + 1e-7,
                )
                if sim.hit:
                    intercepted += 1
                else:
                    escaped += 1
                    self.assertFalse(sim.target.crashed)
        self.assertGreaterEqual(intercepted, 2)
        self.assertGreaterEqual(escaped, 1)

    def test_every_drone_interceptor_hits_the_baseline_evasive_target(self) -> None:
        for interceptor in DRONE_SPECS:
            with self.subTest(interceptor=interceptor.name):
                sim = InterceptionSimulation(
                    SimulationConfig(
                        interceptor_code=interceptor.code,
                        target_code="FX1",
                        scenario="evasive",
                    )
                )
                for _ in range(60 * 35):
                    sim.step()
                    if sim.hit:
                        break
                self.assertTrue(sim.hit)

    def test_no_detection_cruise_holds_altitude_at_half_speed(self) -> None:
        for interceptor in DRONE_SPECS:
            with self.subTest(interceptor=interceptor.name):
                sim = InterceptionSimulation(
                    SimulationConfig(
                        interceptor_code=interceptor.code,
                        scenario="steady",
                    )
                )
                initial_altitude = sim.interceptor.position.y
                sim.toggle_sensor_occlusion()
                for _ in range(60 * 10):
                    sim.step()
                self.assertFalse(sim.interceptor.crashed)
                self.assertAlmostEqual(
                    sim.interceptor.position.y,
                    initial_altitude,
                    delta=1.5,
                )
                self.assertAlmostEqual(
                    sim.interceptor.velocity.length(),
                    interceptor.max_speed * 0.5,
                    delta=2.5,
                )

    def test_aegis_and_smart_evader_spawn_level_at_half_speed(self) -> None:
        for interceptor_code in ("AQ4", "SEV"):
            with self.subTest(interceptor=interceptor_code):
                sim = InterceptionSimulation(
                    SimulationConfig(
                        interceptor_code=interceptor_code,
                        target_code="FX1",
                        scenario="evasive",
                    )
                )
                initial_altitude = sim.interceptor.position.y
                self.assertAlmostEqual(sim.interceptor.velocity.y, 0.0, places=9)
                self.assertAlmostEqual(
                    sim.interceptor.velocity.length(),
                    sim.interceptor.spec.max_speed * 0.5,
                    places=9,
                )
                for _ in range(30):
                    sim.step()
                self.assertAlmostEqual(
                    sim.interceptor.position.y,
                    initial_altitude,
                    delta=0.05,
                )
                self.assertLessEqual(
                    sim.interceptor.forward_direction().y,
                    1e-9,
                )

    def test_vectored_camera_mounts_keep_a_continuous_body_fixed_lock(self) -> None:
        for interceptor_code in ("AQ4", "SEV"):
            with self.subTest(interceptor=interceptor_code):
                sim = InterceptionSimulation(
                    SimulationConfig(
                        interceptor_code=interceptor_code,
                        target_code="FX1",
                        scenario="evasive",
                    )
                )
                mount_angle = angle_between(
                    sim.interceptor.forward_direction(),
                    sim.interceptor.sensor_direction(),
                )
                self.assertAlmostEqual(
                    mount_angle,
                    math.radians(
                        abs(sim.interceptor.spec.camera_mount_pitch_deg)
                    ),
                    places=7,
                )
                maximum_pitch = 0.0
                for _ in range(60 * 25):
                    sim.step()
                    maximum_pitch = max(
                        maximum_pitch,
                        abs(math.degrees(sim.interceptor.orientation.x)),
                    )
                    if sim.hit:
                        break
                self.assertTrue(sim.hit)
                self.assertEqual(sim.reacquisition_count, 0)
                self.assertLessEqual(
                    maximum_pitch,
                    sim.interceptor.spec.max_body_tilt_deg + 0.1,
                )

    def test_multi_enemy_mode_tracks_and_solves_every_contact(self) -> None:
        sim = InterceptionSimulation(
            SimulationConfig(
                target_code="FX1",
                scenario="evasive",
                enemy_count=3,
            )
        )
        self.assertEqual(len(sim.targets), 3)
        self.assertEqual(len({id(contact.track) for contact in sim.contacts}), 3)
        shared_snapshot = None
        shared_pair_snapshot = None
        for _ in range(60 * 3):
            sim.step()
            if sim.shared_overlap is not None:
                shared_snapshot = sim.shared_overlap
                shared_pair_snapshot = sim.shared_overlap_pair
        self.assertEqual(sim.visible_contact_count, 3)
        self.assertEqual(sim.identified_contact_count, 3)
        self.assertTrue(
            all(contact.track.sample_count > 2 for contact in sim.contacts)
        )
        self.assertTrue(
            all(contact.guidance is not None for contact in sim.contacts)
        )
        self.assertTrue(
            all(math.isfinite(contact.priority_score) for contact in sim.contacts)
        )
        self.assertIsNotNone(shared_snapshot)
        self.assertIsNotNone(shared_pair_snapshot)
        assert shared_snapshot is not None
        self.assertIn(
            shared_snapshot.horizon_s,
            (1.0, 2.0, 3.0, 5.0),
        )
        self.assertTrue(
            shared_snapshot.first.contains_projected(
                shared_snapshot.aim_point
            )
        )
        self.assertTrue(
            shared_snapshot.second.contains_projected(
                shared_snapshot.aim_point
            )
        )
        self.assertTrue(
            any("Shared aim" in message for _, message in sim.events)
        )
        self.assertIn(sim.target, sim.targets)
        self.assertIs(sim.target, sim.active_contact.vehicle)
        for _ in range(60 * 12):
            sim.step()
            if sim.multi_committed_contact_index is not None:
                break
        self.assertIsNotNone(sim.multi_committed_contact_index)
        self.assertTrue(sim.multi_guidance_mode.startswith("COMMITTED"))

    def test_unloaded_ai_backend_cannot_be_falsely_selected(self) -> None:
        with self.assertRaisesRegex(ValueError, "no loaded image-model weights"):
            InterceptionSimulation(
                SimulationConfig(detector_backend="yolo")
            )

    def test_displayed_oval_radius_is_frame_stable(self) -> None:
        # AQ4 at long range used to cross a 3 px -> 4 px rounding boundary,
        # producing a 240% one-frame jump in the rendered prediction edge.
        sim = InterceptionSimulation(
            SimulationConfig(
                interceptor_code="TLR",
                target_code="AQ4",
                scenario="evasive",
            )
        )
        previous_radius_px: float | None = None
        previous_time_s: float | None = None
        largest_relative_jump = 0.0
        for _ in range(60 * 8):
            sim.step()
            if sim.guidance is None:
                previous_radius_px = None
                previous_time_s = None
                continue
            radius_px = sim.guidance.ovals[0].approximate_radius_px
            if (
                previous_radius_px is not None
                and previous_time_s is not None
                and sim.time_s - previous_time_s < 0.02
            ):
                largest_relative_jump = max(
                    largest_relative_jump,
                    abs(radius_px - previous_radius_px)
                    / max(previous_radius_px, 1.0),
                )
            previous_radius_px = radius_px
            previous_time_s = sim.time_s
        self.assertLess(largest_relative_jump, 0.08)

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

    def test_both_rockets_can_be_our_one_way_interceptor(self) -> None:
        for rocket in ROCKET_SPECS:
            with self.subTest(rocket=rocket.name):
                sim = InterceptionSimulation(
                    SimulationConfig(
                        interceptor_code=rocket.code,
                        target_code="FX1",
                        scenario="steady",
                    )
                )
                for _ in range(60 * 12):
                    sim.step()
                    if sim.hit:
                        break
                self.assertTrue(sim.hit)
                self.assertEqual(sim.interceptor.spec.code, rocket.code)

    def test_two_second_prediction_check_uses_recorded_camera(self) -> None:
        sim = InterceptionSimulation(SimulationConfig(scenario="steady"))
        for _ in range(120):
            sim.step()
            if sim.guidance is not None:
                break
        self.assertTrue(sim.capture_prediction_check())
        assert sim.prediction_check is not None
        recorded_position = sim.prediction_check.camera_position
        recorded_forward = sim.prediction_check.camera_forward
        for _ in range(125):
            sim.step()
        check = sim.prediction_check
        assert check is not None
        self.assertTrue(check.evaluated)
        self.assertTrue(check.result_inside)
        self.assertEqual(check.camera_position, recorded_position)
        self.assertEqual(check.camera_forward, recorded_forward)
        self.assertGreater(
            sim.interceptor.position.distance_to(recorded_position),
            1.0,
        )

    def test_two_second_outer_oval_contains_all_drone_scenarios(self) -> None:
        for scenario, _ in SCENARIOS:
            if scenario == "rocket_attack":
                continue
            with self.subTest(scenario=scenario):
                sim = InterceptionSimulation(
                    SimulationConfig(scenario=scenario, target_code="FX1")
                )
                for _ in range(120):
                    sim.step()
                    if sim.guidance is not None and sim.time_s > 0.8:
                        break
                self.assertTrue(sim.capture_prediction_check())
                for _ in range(125):
                    sim.step()
                check = sim.prediction_check
                assert check is not None
                self.assertTrue(check.evaluated)
                self.assertTrue(check.result_inside)


if __name__ == "__main__":
    unittest.main()
