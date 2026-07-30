"""Pygame software 3D renderer, HUD, and engineering-analysis overlay."""

from __future__ import annotations

from dataclasses import dataclass
import math

import pygame

from .camera import (
    YOLO_DRONE_BOX_CALIBRATION,
    minimum_horizontal_resolution,
    model_vertices,
)
from .controls import ControlMode
from .guidance import PredictionOval
from .math3d import (
    Vec3,
    basis_from_forward,
    camera_coordinates,
    clamp,
)
from .meshes import get_mesh
from .simulation import InterceptionSimulation, SCENARIOS


BG_TOP = (7, 17, 31)
BG_BOTTOM = (39, 83, 112)
CYAN = (72, 224, 238)
GREEN = (83, 232, 166)
AMBER = (255, 190, 86)
RED = (255, 91, 99)
WHITE = (224, 238, 244)
MUTED = (130, 156, 170)
PANEL = (5, 16, 25, 238)
INFO_PAGE_COUNT = 4
GREY = (102, 119, 128)


@dataclass(slots=True)
class ViewCamera:
    position: Vec3
    forward: Vec3
    name: str
    roll_rad: float = 0.0


@dataclass(slots=True)
class _ProjectionState:
    camera: ViewCamera
    width: int
    height: int
    fov_deg: float
    position: Vec3
    right: Vec3
    up: Vec3
    forward: Vec3
    roll_cos: float
    roll_sin: float
    focal_px: float


class WorldRenderer:
    DEFAULT_PRESENTATION_FOV_DEG = 72.0
    MIN_PRESENTATION_FOV_DEG = 24.0
    MAX_PRESENTATION_FOV_DEG = 100.0

    def __init__(self) -> None:
        pygame.font.init()
        self.font_tiny = pygame.font.SysFont("consolas", 12)
        self.font_small = pygame.font.SysFont("consolas", 14)
        self.font = pygame.font.SysFont("consolas", 16)
        self.font_bold = pygame.font.SysFont("consolas", 16, bold=True)
        self.font_title = pygame.font.SysFont("arial", 24, bold=True)
        self.font_large = pygame.font.SysFont("arial", 32, bold=True)
        self._background: pygame.Surface | None = None
        self._background_size = (0, 0)
        self._prediction_fill: pygame.Surface | None = None
        self._prediction_fill_size = (0, 0)
        self.view_mode = 0
        self.interceptor_trail: list[Vec3] = []
        self.target_trail: list[Vec3] = []
        self.estimated_target_trail: list[Vec3] = []
        self._trail_timer = 0.0
        self.look_yaw_rad = 0.0
        self.look_pitch_rad = 0.0
        self.look_roll_rad = 0.0
        self.presentation_fov_deg = self.DEFAULT_PRESENTATION_FOV_DEG
        self.panel_expanded = {
            "target": True,
            "calculations": True,
            "own": True,
            "relative": True,
        }
        self.minimap_visible = False
        self.settings_visible = False
        self.verification_visible = False
        self.aim_aids_visible = True
        self.click_regions: dict[str, pygame.Rect] = {}
        self._projection_state: _ProjectionState | None = None

    def cycle_view(self) -> None:
        self.view_mode = (self.view_mode + 1) % 3
        self.reset_view_offset()

    @property
    def free_look_active(self) -> bool:
        return (
            abs(self.look_yaw_rad) > 1e-5
            or abs(self.look_pitch_rad) > 1e-5
            or abs(self.look_roll_rad) > 1e-5
        )

    def reset_view_offset(self) -> None:
        self.look_yaw_rad = 0.0
        self.look_pitch_rad = 0.0
        self.look_roll_rad = 0.0
        self.presentation_fov_deg = self.DEFAULT_PRESENTATION_FOV_DEG
        self._projection_state = None

    def adjust_zoom(self, wheel_steps: int) -> None:
        """Zoom only the presentation camera; the simulated sensor stays fixed."""
        if not wheel_steps:
            return
        self.presentation_fov_deg = clamp(
            self.presentation_fov_deg * (0.90 ** wheel_steps),
            self.MIN_PRESENTATION_FOV_DEG,
            self.MAX_PRESENTATION_FOV_DEG,
        )
        self._projection_state = None

    @property
    def zoom_multiplier(self) -> float:
        default_tangent = math.tan(
            math.radians(self.DEFAULT_PRESENTATION_FOV_DEG) * 0.5
        )
        current_tangent = math.tan(
            math.radians(self.presentation_fov_deg) * 0.5
        )
        return default_tangent / current_tangent

    def rotate_view(
        self,
        relative_x: float,
        relative_y: float,
        roll_mode: bool = False,
    ) -> None:
        """Apply an independent presentation-camera mouse offset."""
        if roll_mode:
            self.look_roll_rad = (
                self.look_roll_rad
                + relative_x * math.radians(0.15)
                + math.pi
            ) % math.tau - math.pi
            return
        self.look_yaw_rad = (
            self.look_yaw_rad
            + relative_x * math.radians(0.12)
            + math.pi
        ) % math.tau - math.pi
        self.look_pitch_rad = clamp(
            self.look_pitch_rad - relative_y * math.radians(0.12),
            math.radians(-85.0),
            math.radians(85.0),
        )

    def update_trails(self, sim: InterceptionSimulation, dt: float) -> None:
        self._trail_timer += dt
        if self._trail_timer < 0.08:
            return
        self._trail_timer = 0.0
        self.interceptor_trail.append(
            Vec3(
                sim.interceptor.position.x,
                sim.interceptor.position.y,
                sim.interceptor.position.z,
            )
        )
        self.target_trail.append(
            Vec3(sim.target.position.x, sim.target.position.y, sim.target.position.z)
        )
        if sim.track.position is not None and sim.visual_locked:
            self.estimated_target_trail.append(
                Vec3(
                    sim.track.position.x,
                    sim.track.position.y,
                    sim.track.position.z,
                )
            )
        self.interceptor_trail = self.interceptor_trail[-180:]
        self.target_trail = self.target_trail[-180:]
        self.estimated_target_trail = self.estimated_target_trail[-180:]

    def handle_left_click(
        self,
        position: tuple[int, int],
        sim: InterceptionSimulation,
    ) -> str | None:
        """Handle the clickable HUD without coupling renderer state to app state."""
        for key, rect in reversed(tuple(self.click_regions.items())):
            if not rect.collidepoint(position):
                continue
            if key.startswith("panel:"):
                panel = key.split(":", 1)[1]
                self.panel_expanded[panel] = not self.panel_expanded[panel]
                return "handled"
            if key == "minimap":
                self.minimap_visible = not self.minimap_visible
                return "handled"
            if key == "settings":
                self.settings_visible = not self.settings_visible
                return "handled"
            if key == "verification":
                self.verification_visible = not self.verification_visible
                return "handled"
            if key == "aim_aids":
                self.aim_aids_visible = not self.aim_aids_visible
                return "handled"
            if key == "terminal_mode":
                sim.terminal_mode = (
                    "TTC_1S"
                    if sim.terminal_mode == "ONE_SECOND_ENVELOPE"
                    else "ONE_SECOND_ENVELOPE"
                )
                return "handled"
            if key == "capture_check":
                self.verification_visible = True
                sim.capture_prediction_check()
                return "handled"
            if key == "clear_check":
                sim.clear_prediction_check()
                return "handled"
            if key == "panels_reset":
                self.panel_expanded = {
                    "target": True,
                    "calculations": True,
                    "own": True,
                    "relative": True,
                }
                return "handled"
            return key
        return None

    def _build_background(self, size: tuple[int, int]) -> pygame.Surface:
        surface = pygame.Surface(size)
        width, height = size
        horizon = int(height * 0.64)
        for y in range(horizon):
            t = y / max(1, horizon)
            color = tuple(
                int(BG_TOP[channel] * (1 - t) + BG_BOTTOM[channel] * t)
                for channel in range(3)
            )
            pygame.draw.line(surface, color, (0, y), (width, y))
        pygame.draw.rect(surface, (21, 42, 43), (0, horizon, width, height - horizon))

        # Distant mountain silhouettes make the synthetic scene spatially legible.
        mountains = [
            (0, horizon),
            (0, horizon - 18),
            (width * 0.12, horizon - 72),
            (width * 0.24, horizon - 24),
            (width * 0.37, horizon - 92),
            (width * 0.50, horizon - 30),
            (width * 0.64, horizon - 68),
            (width * 0.78, horizon - 20),
            (width * 0.91, horizon - 78),
            (width, horizon - 32),
            (width, horizon),
        ]
        pygame.draw.polygon(surface, (24, 54, 59), mountains)
        pygame.draw.circle(surface, (233, 199, 112), (width - 130, 100), 27)
        return surface

    def get_view_camera(self, sim: InterceptionSimulation) -> ViewCamera:
        line = (sim.target.position - sim.interceptor.position).normalized(
            sim.camera_forward
        )
        controlled = sim.controlled_vehicle
        followed = controlled or sim.interceptor
        travel_direction = followed.velocity.normalized(
            followed.forward_direction()
        )
        if self.view_mode == 0:
            if controlled is None:
                base = ViewCamera(
                    sim.interceptor.position,
                    sim.camera_forward,
                    "ONBOARD / FIXED BORESIGHT",
                )
            else:
                base = ViewCamera(
                    followed.position,
                    followed.sensor_direction(),
                    f"ONBOARD / {followed.spec.code} PLAYER",
                )
        elif self.view_mode == 1:
            position = followed.position - travel_direction * 14.0 + Vec3(0, 5.0, 0)
            focus = followed.position + travel_direction * 8.0
            base = ViewCamera(
                position,
                (focus - position).normalized(travel_direction),
                (
                    f"CHASE / {followed.spec.code} PLAYER"
                    if controlled is not None
                    else "CHASE"
                ),
            )
        else:
            midpoint = (sim.interceptor.position + sim.target.position) * 0.5
            separation = clamp(sim.true_range_m, 35.0, 280.0)
            position = midpoint + Vec3(
                -separation * 0.54,
                separation * 0.40,
                -separation * 0.58,
            )
            base = ViewCamera(
                position,
                (midpoint - position).normalized(line),
                "SPECTATOR / TACTICAL",
            )

        if not self.free_look_active:
            return base
        right, up, forward = basis_from_forward(base.forward)
        yawed = (
            forward * math.cos(self.look_yaw_rad)
            + right * math.sin(self.look_yaw_rad)
        ).normalized(forward)
        yawed_right = up.cross(yawed).normalized(right)
        yawed_up = yawed.cross(yawed_right).normalized(up)
        looked = (
            yawed * math.cos(self.look_pitch_rad)
            + yawed_up * math.sin(self.look_pitch_rad)
        ).normalized(yawed)
        return ViewCamera(
            base.position,
            looked,
            f"{base.name} / FREE LOOK",
            self.look_roll_rad,
        )

    @staticmethod
    def _project(
        point: Vec3,
        camera: ViewCamera,
        width: int,
        height: int,
        fov_deg: float = 72.0,
    ) -> tuple[int, int, float] | None:
        relative = camera_coordinates(point, camera.position, camera.forward)
        if abs(camera.roll_rad) > 1e-8:
            cosine = math.cos(camera.roll_rad)
            sine = math.sin(camera.roll_rad)
            relative = Vec3(
                relative.x * cosine + relative.y * sine,
                -relative.x * sine + relative.y * cosine,
                relative.z,
            )
        if relative.z <= 0.08:
            return None
        focal = width / (2.0 * math.tan(math.radians(fov_deg) * 0.5))
        x = int(width * 0.5 + focal * relative.x / relative.z)
        y = int(height * 0.5 - focal * relative.y / relative.z)
        return x, y, relative.z

    def _prepare_projection(
        self,
        camera: ViewCamera,
        width: int,
        height: int,
        fov_deg: float | None = None,
    ) -> _ProjectionState:
        if fov_deg is None:
            fov_deg = self.presentation_fov_deg
        state = self._projection_state
        if (
            state is not None
            and state.camera is camera
            and state.width == width
            and state.height == height
            and state.fov_deg == fov_deg
        ):
            return state
        right, up, forward = basis_from_forward(camera.forward)
        state = _ProjectionState(
            camera,
            width,
            height,
            fov_deg,
            camera.position,
            right,
            up,
            forward,
            math.cos(camera.roll_rad),
            math.sin(camera.roll_rad),
            width / (2.0 * math.tan(math.radians(fov_deg) * 0.5)),
        )
        self._projection_state = state
        return state

    @staticmethod
    def _camera_space(
        point: Vec3,
        state: _ProjectionState,
    ) -> tuple[float, float, float]:
        dx = point.x - state.position.x
        dy = point.y - state.position.y
        dz = point.z - state.position.z
        x = dx * state.right.x + dy * state.right.y + dz * state.right.z
        y = dx * state.up.x + dy * state.up.y + dz * state.up.z
        depth = (
            dx * state.forward.x
            + dy * state.forward.y
            + dz * state.forward.z
        )
        if abs(state.camera.roll_rad) > 1e-8:
            x, y = (
                x * state.roll_cos + y * state.roll_sin,
                -x * state.roll_sin + y * state.roll_cos,
            )
        return x, y, depth

    @staticmethod
    def _screen_from_camera(
        point: tuple[float, float, float],
        state: _ProjectionState,
    ) -> tuple[int, int, float]:
        x, y, depth = point
        return (
            int(state.width * 0.5 + state.focal_px * x / depth),
            int(state.height * 0.5 - state.focal_px * y / depth),
            depth,
        )

    def _project_cached(
        self,
        point: Vec3,
        camera: ViewCamera,
        width: int,
        height: int,
        fov_deg: float | None = None,
    ) -> tuple[int, int, float] | None:
        state = self._prepare_projection(camera, width, height, fov_deg)
        camera_point = self._camera_space(point, state)
        if camera_point[2] <= 0.08:
            return None
        return self._screen_from_camera(camera_point, state)

    def _draw_ground_grid(
        self, surface: pygame.Surface, camera: ViewCamera, usable_height: int
    ) -> None:
        width, height = surface.get_size()
        grid_color = (48, 90, 83)
        major_color = (55, 111, 100)
        state = self._prepare_projection(camera, width, height)

        def clip_screen_line(
            first: tuple[float, float],
            second: tuple[float, float],
        ) -> tuple[tuple[int, int], tuple[int, int]] | None:
            """Liang-Barsky clipping avoids SDL overflow near the camera plane."""
            x0, y0 = first
            dx = second[0] - x0
            dy = second[1] - y0
            lower, upper = 0.0, 1.0
            for direction, distance in (
                (-dx, x0),
                (dx, width - 1 - x0),
                (-dy, y0),
                (dy, usable_height - 1 - y0),
            ):
                if abs(direction) < 1e-12:
                    if distance < 0.0:
                        return None
                    continue
                ratio = distance / direction
                if direction < 0.0:
                    if ratio > upper:
                        return None
                    lower = max(lower, ratio)
                else:
                    if ratio < lower:
                        return None
                    upper = min(upper, ratio)
            return (
                (int(x0 + dx * lower), int(y0 + dy * lower)),
                (int(x0 + dx * upper), int(y0 + dy * upper)),
            )

        def draw_world_line(
            start: Vec3,
            end: Vec3,
            color: tuple[int, int, int],
        ) -> None:
            first = self._camera_space(start, state)
            second = self._camera_space(end, state)
            near = 0.08
            if first[2] <= near and second[2] <= near:
                return
            if first[2] <= near:
                amount = (near - first[2]) / (second[2] - first[2])
                first = (
                    first[0] + (second[0] - first[0]) * amount,
                    first[1] + (second[1] - first[1]) * amount,
                    near,
                )
            elif second[2] <= near:
                amount = (near - second[2]) / (first[2] - second[2])
                second = (
                    second[0] + (first[0] - second[0]) * amount,
                    second[1] + (first[1] - second[1]) * amount,
                    near,
                )
            first_screen = (
                state.width * 0.5 + state.focal_px * first[0] / first[2],
                state.height * 0.5 - state.focal_px * first[1] / first[2],
            )
            second_screen = (
                state.width * 0.5 + state.focal_px * second[0] / second[2],
                state.height * 0.5 - state.focal_px * second[1] / second[2],
            )
            clipped = clip_screen_line(first_screen, second_screen)
            if clipped:
                pygame.draw.line(surface, color, clipped[0], clipped[1], 1)

        for x in range(-400, 401, 25):
            draw_world_line(
                Vec3(x, 0, -100),
                Vec3(x, 0, 900),
                major_color if x == 0 else grid_color,
            )
        for z in range(-100, 901, 25):
            draw_world_line(
                Vec3(-400, 0, z),
                Vec3(400, 0, z),
                major_color if z == 0 else grid_color,
            )

    def _draw_polyline_3d(
        self,
        surface: pygame.Surface,
        points: list[Vec3],
        camera: ViewCamera,
        color: tuple[int, int, int],
        width_px: int = 1,
        fov_deg: float | None = None,
    ) -> None:
        width, height = surface.get_size()
        segment: list[tuple[int, int]] = []
        for point in points:
            projected = self._project_cached(
                point,
                camera,
                width,
                height,
                fov_deg,
            )
            if projected is None:
                if len(segment) > 1:
                    pygame.draw.lines(surface, color, False, segment, width_px)
                segment = []
            else:
                segment.append(projected[:2])
        if len(segment) > 1:
            pygame.draw.lines(surface, color, False, segment, width_px)

    def _draw_trails(self, surface: pygame.Surface, camera: ViewCamera) -> None:
        self._draw_polyline_3d(surface, self.target_trail, camera, (146, 75, 81), 2)
        self._draw_polyline_3d(surface, self.interceptor_trail, camera, (44, 133, 147), 2)

    def _draw_drone(
        self,
        surface: pygame.Surface,
        state,
        camera: ViewCamera,
        label: str,
        highlighted: bool,
    ) -> tuple[int, int, int, int] | None:
        width, height = surface.get_size()
        vertices = model_vertices(state.spec, state.orientation, state.position)
        projected = [
            self._project_cached(vertex, camera, width, height)
            for vertex in vertices
        ]
        if any(point is None for point in projected):
            return None
        projected_ok = [point for point in projected if point is not None]
        mesh = get_mesh(state.spec.mesh_id)
        light = Vec3(-0.35, 0.82, -0.45).normalized()
        sortable = []
        for face in mesh.faces:
            depth = sum(projected_ok[index][2] for index in face.indices) / len(face.indices)
            sortable.append((depth, face))
        for _, face in sorted(sortable, reverse=True, key=lambda item: item[0]):
            first, second, third = (vertices[index] for index in face.indices[:3])
            normal = (second - first).cross(third - first).normalized()
            shade = 0.42 + 0.68 * abs(normal.dot(light))
            if face.material == "rotor":
                base_color = (27, 37, 43)
            elif face.material == "dark":
                base_color = (39, 53, 61)
            elif face.material == "metal":
                base_color = (105, 124, 132)
            elif face.material == "glow":
                base_color = (255, 133, 36)
                shade = 1.0
            elif face.material == "accent":
                base_color = tuple(min(255, channel + 55) for channel in state.spec.color)
            else:
                base_color = state.spec.color
            color = tuple(int(clamp(channel * shade, 0, 255)) for channel in base_color)
            face_points = [projected_ok[index][:2] for index in face.indices]
            pygame.draw.polygon(surface, color, face_points)
            pygame.draw.polygon(
                surface,
                (20, 28, 34),
                face_points,
                1,
            )

        xs = [point[0] for point in projected_ok]
        ys = [point[1] for point in projected_ok]
        bounds = (min(xs), min(ys), max(xs), max(ys))
        center = (int((bounds[0] + bounds[2]) * 0.5), int((bounds[1] + bounds[3]) * 0.5))
        if highlighted:
            marker_size = max(18, int(max(bounds[2] - bounds[0], bounds[3] - bounds[1]) * 0.8))
            pygame.draw.circle(surface, state.spec.color, center, marker_size, 1)
            text = self.font_tiny.render(label, True, state.spec.color)
            surface.blit(text, (center[0] + marker_size + 4, center[1] - 7))
        return bounds

    @staticmethod
    def _oval_points(oval: PredictionOval, count: int = 72) -> list[Vec3]:
        if oval.edge_points:
            return [*oval.edge_points, oval.edge_points[0]]
        return [
            oval.center
            + oval.plane_x * (math.cos(math.tau * index / count) * oval.radius_x)
            + oval.plane_y * (math.sin(math.tau * index / count) * oval.radius_y)
            for index in range(count + 1)
        ]

    @staticmethod
    def _oval_reachability_color(
        oval: PredictionOval,
    ) -> tuple[int, int, int]:
        if oval.invalid_reason:
            return GREY
        if oval.fully_reachable:
            return GREEN
        # All rendered edge directions must pass, not only four cardinals.
        return RED

    @staticmethod
    def _oval_edge_colors(
        oval: PredictionOval,
    ) -> tuple[tuple[int, int, int], ...]:
        """One displayed color per evaluated edge direction."""
        if oval.invalid_reason:
            return tuple(GREY for _ in oval.edge_points)
        return tuple(
            GREEN if reachable else RED
            for reachable in oval.edge_reachable
        )

    @staticmethod
    def _oval_fill_color(
        oval: PredictionOval,
    ) -> tuple[int, int, int] | None:
        """Tint only a region whose complete boundary passed reachability."""
        return GREEN if oval.fully_reachable else None

    def _draw_predictions(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
        camera: ViewCamera,
    ) -> None:
        if not sim.guidance or sim.hit:
            return
        selected_horizon = sim.guidance.selected_horizon_s
        surface_size = surface.get_size()
        if (
            self._prediction_fill is None
            or self._prediction_fill_size != surface_size
        ):
            self._prediction_fill = pygame.Surface(surface_size, pygame.SRCALPHA)
            self._prediction_fill_size = surface_size
        fill_layer = self._prediction_fill
        fill_layer.fill((0, 0, 0, 0))
        projected_ovals = [
            (
                oval,
                [
                    self._project_cached(point, camera, *surface_size)
                    for point in oval.edge_points
                ],
            )
            for oval in sim.guidance.ovals
        ]
        shared_polygon_projected = (
            [
                self._project_cached(point, camera, *surface_size)
                for point in sim.shared_overlap.polygon_world
            ]
            if sim.shared_overlap is not None
            else []
        )

        # A faint fill makes the nested prediction regions readable against
        # both sky and ground while leaving the synthetic target unobscured.
        # Only a fully reachable oval is filled: a red *edge* does not imply
        # that every point inside it is unreachable.
        for oval, projected_edge in projected_ovals:
            fill_color = self._oval_fill_color(oval)
            if (
                fill_color is not None
                and len(projected_edge) >= 3
                and all(point is not None for point in projected_edge)
            ):
                polygon = [
                    point[:2]
                    for point in projected_edge
                    if point is not None
                ]
                xs = [point[0] for point in polygon]
                ys = [point[1] for point in polygon]
                # SDL's software polygon rasterizer becomes needlessly costly
                # for a huge off-screen 5 s envelope. Its thick border remains
                # exact; skip only the cosmetic fill when it spans >2 screens.
                if (
                    max(xs) - min(xs) <= surface_size[0] * 2
                    and max(ys) - min(ys) <= surface_size[1] * 2
                ):
                    pygame.draw.polygon(fill_layer, (*fill_color, 14), polygon)
        if (
            len(shared_polygon_projected) >= 3
            and all(point is not None for point in shared_polygon_projected)
        ):
            pygame.draw.polygon(
                fill_layer,
                (*CYAN, 34),
                [
                    point[:2]
                    for point in shared_polygon_projected
                    if point is not None
                ],
            )
        surface.blit(fill_layer, (0, 0))

        for oval, projected_edge in projected_ovals:
            selected = (
                selected_horizon is not None
                and abs(oval.horizon_s - selected_horizon) < 0.02
                and sim.guidance.mode == "WEIGHTED OVAL"
            )
            draw_color = self._oval_reachability_color(oval)
            border_width = 5 if selected else 3
            if projected_edge:
                edge_count = len(projected_edge)
                segment_colors = list(self._oval_edge_colors(oval))
                if not segment_colors:
                    segment_colors = [draw_color] * edge_count

                # Batch contiguous same-color arcs instead of issuing 96 draw
                # calls per oval. Mixed borders stay exact and render quickly.
                if len(set(segment_colors)) == 1 and all(
                    point is not None for point in projected_edge
                ):
                    pygame.draw.lines(
                        surface,
                        segment_colors[0],
                        True,
                        [point[:2] for point in projected_edge if point],
                        border_width,
                    )
                else:
                    start = next(
                        (
                            index
                            for index in range(edge_count)
                            if segment_colors[index]
                            != segment_colors[index - 1]
                        ),
                        0,
                    )
                    active_color = segment_colors[start]
                    active_points: list[tuple[int, int]] = []
                    for offset in range(edge_count):
                        index = (start + offset) % edge_count
                        following = (index + 1) % edge_count
                        first = projected_edge[index]
                        second = projected_edge[following]
                        color = segment_colors[index]
                        if (
                            color != active_color
                            or first is None
                            or second is None
                        ):
                            if len(active_points) > 1:
                                pygame.draw.lines(
                                    surface,
                                    active_color,
                                    False,
                                    active_points,
                                    border_width,
                                )
                            active_points = []
                            active_color = color
                        if first is not None and second is not None:
                            if not active_points:
                                active_points.append(first[:2])
                            active_points.append(second[:2])
                    if len(active_points) > 1:
                        pygame.draw.lines(
                            surface,
                            active_color,
                            False,
                            active_points,
                            border_width,
                        )
            for point, reachable in zip(oval.extremes, oval.reachable):
                projected = self._project_cached(
                    point,
                    camera,
                    *surface_size,
                )
                if projected:
                    pygame.draw.circle(
                        surface, GREEN if reachable else RED, projected[:2], 4
                    )
            label_anchor = (
                oval.center
                + oval.plane_x * (oval.radius_x * 0.68)
                + oval.plane_y * (oval.radius_y * 0.68)
            )
            label_projected = self._project_cached(
                label_anchor,
                camera,
                *surface_size,
            )
            if label_projected:
                if oval.invalid_reason:
                    label = (
                        f"+{oval.horizon_s:.0f}s  UNBOUNDED / "
                        f"{oval.invalid_reason}"
                    )
                elif selected:
                    label = (
                        f"SELECTED +{oval.horizon_s:.0f}s  "
                        f"EDGE {oval.edge_reachable_count}/{oval.edge_total}"
                    )
                elif oval.fully_reachable:
                    label = (
                        f"+{oval.horizon_s:.0f}s  GREEN "
                        f"{oval.edge_reachable_count}/{oval.edge_total}"
                    )
                elif oval.edge_reachable_count:
                    label = (
                        f"+{oval.horizon_s:.0f}s  MIXED "
                        f"{oval.edge_reachable_count}/{oval.edge_total} GREEN "
                        f"| CARDINAL {oval.cardinal_reachable_count}/4"
                    )
                else:
                    label = (
                        f"+{oval.horizon_s:.0f}s  RED / BLOCKED "
                        f"{oval.edge_reachable_count}/{oval.edge_total} "
                        f"| CARDINAL {oval.cardinal_reachable_count}/4"
                    )
                text = self.font_tiny.render(label, True, draw_color)
                chip = pygame.Surface(
                    (text.get_width() + 8, text.get_height() + 4),
                    pygame.SRCALPHA,
                )
                chip.fill((3, 13, 20, 210))
                pygame.draw.rect(chip, (*draw_color, 190), chip.get_rect(), 1)
                chip.blit(text, (4, 2))
                surface.blit(
                    chip,
                    (label_projected[0] + 5, label_projected[1] - 8),
                )

        if (
            len(shared_polygon_projected) >= 3
            and all(point is not None for point in shared_polygon_projected)
        ):
            pygame.draw.lines(
                surface,
                CYAN,
                True,
                [
                    point[:2]
                    for point in shared_polygon_projected
                    if point is not None
                ],
                2,
            )

        if (
            sim.shared_overlap is not None
            and sim.shared_overlap_pair is not None
        ):
            secondary_index = next(
                (
                    index
                    for index in sim.shared_overlap_pair
                    if index != sim.active_contact_index
                ),
                sim.shared_overlap_pair[1],
            )
            secondary_contact = sim.contacts[secondary_index]
            secondary_guidance = secondary_contact.guidance
            if secondary_guidance is not None:
                secondary_oval = next(
                    (
                        oval
                        for oval in secondary_guidance.ovals
                        if abs(
                            oval.horizon_s
                            - sim.shared_overlap.horizon_s
                        )
                        < 1e-6
                    ),
                    None,
                )
                if secondary_oval is not None:
                    secondary_edge = [
                        self._project_cached(
                            point,
                            camera,
                            *surface_size,
                        )
                        for point in secondary_oval.edge_points
                    ]
                    secondary_colors = [
                        GREEN if reachable else RED
                        for reachable in secondary_oval.edge_reachable
                    ]
                    if (
                        secondary_edge
                        and len(set(secondary_colors)) == 1
                        and all(
                            point is not None
                            for point in secondary_edge
                        )
                    ):
                        pygame.draw.lines(
                            surface,
                            secondary_colors[0],
                            True,
                            [
                                point[:2]
                                for point in secondary_edge
                                if point is not None
                            ],
                            2,
                        )
                    else:
                        for index, first in enumerate(secondary_edge):
                            second = secondary_edge[
                                (index + 1) % len(secondary_edge)
                            ]
                            if first is None or second is None:
                                continue
                            color = (
                                secondary_colors[index]
                                if index < len(secondary_colors)
                                else RED
                            )
                            pygame.draw.line(
                                surface,
                                color,
                                first[:2],
                                second[:2],
                                2,
                            )
                    secondary_center = self._project_cached(
                        secondary_oval.center,
                        camera,
                        *surface_size,
                    )
                    if secondary_center is not None:
                        label = self.font_tiny.render(
                            f"PAIR {secondary_contact.track_id} "
                            f"+{secondary_oval.horizon_s:.0f}s",
                            True,
                            CYAN,
                        )
                        surface.blit(
                            label,
                            (
                                secondary_center[0] + 9,
                                secondary_center[1] + 9,
                            ),
                        )

        ballistic_points = [
            sim.track.position
            + sim.track.velocity * (step * 0.5)
            - sim.guidance.ovals[0].plane_normal
            * (
                sim.track.velocity
                * (step * 0.5)
            ).dot(sim.guidance.ovals[0].plane_normal)
            for step in range(1, 11)
        ] if sim.track.position else []
        for point in ballistic_points:
            projected = self._project_cached(
                point,
                camera,
                *surface_size,
            )
            if projected:
                pygame.draw.circle(surface, AMBER, projected[:2], 2)

        displayed_aim_point = (
            sim.shared_overlap.aim_point
            if sim.shared_overlap is not None
            else sim.guidance.aim_point
        )
        aim = (
            self._project_cached(
                displayed_aim_point,
                camera,
                *surface_size,
            )
            if self.aim_aids_visible
            else None
        )
        if aim:
            x, y = aim[:2]
            diamond = ((x, y - 9), (x + 9, y), (x, y + 9), (x - 9, y))
            pygame.draw.polygon(surface, WHITE, diamond, 2)
            surface.blit(
                self.font_tiny.render(
                    (
                        "SHARED OVERLAP AIM"
                        if sim.shared_overlap is not None
                        else "AIM"
                    ),
                    True,
                    WHITE,
                ),
                (x + 12, y - 7),
            )

        legend = self.font_tiny.render(
            "OVAL EDGE  GREEN=REACHABLE  RED=BLOCKED  CYAN=SHARED OVERLAP",
            True,
            WHITE,
        )
        legend_chip = pygame.Surface(
            (legend.get_width() + 12, legend.get_height() + 7),
            pygame.SRCALPHA,
        )
        legend_chip.fill((3, 13, 20, 218))
        pygame.draw.line(
            legend_chip,
            GREEN,
            (5, legend_chip.get_height() - 2),
            (legend_chip.get_width() // 2, legend_chip.get_height() - 2),
            2,
        )
        pygame.draw.line(
            legend_chip,
            RED,
            (legend_chip.get_width() // 2, legend_chip.get_height() - 2),
            (legend_chip.get_width() - 5, legend_chip.get_height() - 2),
            2,
        )
        legend_chip.blit(legend, (6, 2))
        legend_x = 342 if sim.control_mode is not ControlMode.AUTO else 12
        surface.blit(legend_chip, (legend_x, 126))

    def _draw_detection_brackets(
        self,
        surface: pygame.Surface,
        bounds: tuple[int, int, int, int] | None,
        sim: InterceptionSimulation,
    ) -> None:
        if (
            sim.config.detector_backend == "yolo"
            and sim.visual_locked
            and sim.detection.visible
        ):
            width, height = surface.get_size()
            display_focal = width / (
                2.0
                * math.tan(
                    math.radians(self.presentation_fov_deg) * 0.5
                )
            )
            focal_scale = display_focal / sim.config.camera.focal_px
            center_x = width * 0.5 + (
                sim.detection.center_px[0]
                - sim.config.camera.width_px * 0.5
            ) * focal_scale
            center_y = height * 0.5 + (
                sim.detection.center_px[1]
                - sim.config.camera.height_px * 0.5
            ) * focal_scale
            half_width = sim.detection.width_px * focal_scale * 0.5
            half_height = sim.detection.height_px * focal_scale * 0.5
            bounds = (
                int(center_x - half_width),
                int(center_y - half_height),
                int(center_x + half_width),
                int(center_y + half_height),
            )
        if (
            not bounds
            or self.view_mode != 0
            or self.free_look_active
            or not sim.visual_locked
        ):
            return
        left, top, right, bottom = bounds
        center_x, center_y = (left + right) // 2, (top + bottom) // 2
        size = max(24, int(max(right - left, bottom - top) * 0.8) + 12)
        box = pygame.Rect(center_x - size, center_y - size, size * 2, size * 2)
        color = GREEN if sim.identity_confirmed else AMBER
        corner = min(18, size)
        segments = (
            ((box.left, box.top + corner), (box.left, box.top), (box.left + corner, box.top)),
            ((box.right - corner, box.top), (box.right, box.top), (box.right, box.top + corner)),
            ((box.left, box.bottom - corner), (box.left, box.bottom), (box.left + corner, box.bottom)),
            ((box.right - corner, box.bottom), (box.right, box.bottom), (box.right, box.bottom - corner)),
        )
        for segment in segments:
            pygame.draw.lines(surface, color, False, segment, 2)
        model = sim.target.spec.name if sim.identity_confirmed else "UNKNOWN / QUERYING"
        readout = self.font_small.render(
            f"{model}  {max(sim.detection.width_px, sim.detection.height_px):.1f}px",
            True,
            color,
        )
        surface.blit(readout, (box.left, box.top - 20))

    def _draw_explosion(
        self, surface: pygame.Surface, sim: InterceptionSimulation, camera: ViewCamera
    ) -> None:
        age = sim.explosion_age_s
        if age > 2.5 or not sim.hit:
            return
        origin = (sim.interceptor.position + sim.target.position) * 0.5
        for index in range(34):
            azimuth = index * 2.399963
            elevation = ((index * 37) % 19 - 9) / 9.0
            direction = Vec3(
                math.cos(azimuth),
                elevation * 0.8,
                math.sin(azimuth),
            ).normalized()
            speed = 4.0 + (index % 7) * 1.15
            particle = origin + direction * speed * age + Vec3(0, 2.4 * age - 2.8 * age * age, 0)
            projected = self._project_cached(
                particle,
                camera,
                *surface.get_size(),
            )
            if projected:
                fade = clamp(1.0 - age / 2.5, 0.0, 1.0)
                color = (
                    int(255 * fade),
                    int((80 + (index % 4) * 35) * fade),
                    int(35 * fade),
                )
                pygame.draw.circle(surface, color, projected[:2], max(1, int(4 * fade)))

    def _draw_manual_command(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
        camera: ViewCamera,
    ) -> None:
        controlled = sim.controlled_vehicle
        request = sim.manual_command.acceleration
        if controlled is None or request.length() < 0.05 or sim.hit:
            return
        maximum = max(
            controlled.spec.max_accel,
            controlled.spec.lateral_accel,
            controlled.spec.brake_accel,
            0.001,
        )
        length = 1.5 + 3.0 * clamp(request.length() / maximum, 0.0, 1.0)
        start = controlled.position
        end = start + request.normalized() * length
        self._draw_polyline_3d(surface, [start, end], camera, CYAN, 3)
        projected = self._project_cached(
            end,
            camera,
            *surface.get_size(),
        )
        if projected:
            pygame.draw.circle(surface, CYAN, projected[:2], 5, 2)
            surface.blit(
                self.font_tiny.render("PLAYER CMD", True, CYAN),
                (projected[0] + 7, projected[1] - 7),
            )

    def _draw_camera_occlusion(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
    ) -> None:
        if (
            not sim.sensor_occluded
            or self.view_mode != 0
            or sim.control_mode is not ControlMode.AUTO
        ):
            return
        width, height = surface.get_size()
        camera_area = pygame.Rect(0, 54, width, max(1, height - 232))
        blackout = pygame.Surface(camera_area.size, pygame.SRCALPHA)
        blackout.fill((1, 7, 11, 238))
        for y in range(0, camera_area.height, 9):
            pygame.draw.line(
                blackout,
                (26, 47, 54, 115),
                (0, y),
                (camera_area.width, y),
            )
        message = self.font_large.render("CAMERA FEED OCCLUDED", True, RED)
        detail = self.font_small.render(
            "VISUAL LOCK INVALID // GUIDANCE OFF // SEARCH SCAN ACTIVE",
            True,
            AMBER,
        )
        blackout.blit(
            message,
            (
                camera_area.width // 2 - message.get_width() // 2,
                camera_area.height // 2 - 30,
            ),
        )
        blackout.blit(
            detail,
            (
                camera_area.width // 2 - detail.get_width() // 2,
                camera_area.height // 2 + 14,
            ),
        )
        surface.blit(blackout, camera_area.topleft)

    def draw_world(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
    ) -> ViewCamera:
        size = surface.get_size()
        if self._background is None or self._background_size != size:
            self._background = self._build_background(size)
            self._background_size = size
        surface.blit(self._background, (0, 0))
        camera = self.get_view_camera(sim)
        usable_height = size[1] - 190
        self._draw_ground_grid(surface, camera, usable_height)
        self._draw_trails(surface, camera)
        self._draw_predictions(surface, sim, camera)
        self._draw_manual_command(surface, sim, camera)

        drone_draws = []
        camera_vehicle = (
            sim.controlled_vehicle
            if self.view_mode == 0 and sim.controlled_vehicle is not None
            else sim.interceptor if self.view_mode == 0 else None
        )
        if sim.interceptor is not camera_vehicle:
            drone_draws.append(
                (
                    camera_coordinates(
                        sim.interceptor.position, camera.position, camera.forward
                    ).z,
                    sim.interceptor,
                    (
                        f"PLAYER // OUR // {sim.interceptor.spec.code}"
                        if sim.control_mode is ControlMode.INTERCEPTOR
                        else f"OUR // {sim.interceptor.spec.code}"
                    ),
                    False,
                )
            )
        for index, contact in enumerate(sim.contacts):
            target = contact.vehicle
            if target is camera_vehicle:
                continue
            active = index == sim.active_contact_index
            if sim.control_mode is ControlMode.TARGET and active:
                label = f"PLAYER // {contact.track_id} // {target.spec.code}"
            elif active:
                label = f"PRIORITY // {contact.track_id} // {target.spec.code}"
            else:
                label = f"CONTACT // {contact.track_id}"
            drone_draws.append(
                (
                    camera_coordinates(
                        target.position,
                        camera.position,
                        camera.forward,
                    ).z,
                    target,
                    label,
                    contact.visual_locked,
                )
            )
        target_bounds = None
        for _, state, label, highlighted in sorted(drone_draws, reverse=True, key=lambda item: item[0]):
            bounds = self._draw_drone(surface, state, camera, label, highlighted)
            if state is sim.target:
                target_bounds = bounds

        self._draw_explosion(surface, sim, camera)
        self._draw_detection_brackets(surface, target_bounds, sim)
        self._draw_camera_occlusion(surface, sim)
        self._draw_reticle(surface, sim)
        return camera

    def _draw_reticle(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
    ) -> None:
        if (
            self.view_mode != 0
            or self.free_look_active
            or sim.control_mode is not ControlMode.AUTO
            or not self.aim_aids_visible
        ):
            return
        width, height = surface.get_size()
        center = (width // 2, (height - 180) // 2)
        color = (109, 162, 171)
        pygame.draw.circle(surface, color, center, 18, 1)
        pygame.draw.line(surface, color, (center[0] - 31, center[1]), (center[0] - 10, center[1]), 1)
        pygame.draw.line(surface, color, (center[0] + 10, center[1]), (center[0] + 31, center[1]), 1)
        pygame.draw.line(surface, color, (center[0], center[1] - 31), (center[0], center[1] - 10), 1)
        pygame.draw.line(surface, color, (center[0], center[1] + 10), (center[0], center[1] + 31), 1)
        label = self.font_tiny.render("SENSOR BORESIGHT", True, color)
        surface.blit(label, (center[0] + 35, center[1] - 7))

    def _panel(
        self,
        surface: pygame.Surface,
        rect: pygame.Rect,
        title: str,
        rows: list[tuple],
    ) -> None:
        panel = pygame.Surface(rect.size, pygame.SRCALPHA)
        panel.fill(PANEL)
        pygame.draw.rect(panel, (64, 143, 155, 220), panel.get_rect(), 1)
        pygame.draw.line(panel, CYAN, (0, 31), (rect.width, 31), 1)
        panel.blit(self.font_bold.render(title, True, CYAN), (12, 8))
        y = 39
        for row in rows:
            label, value = row[0], row[1]
            color = row[2] if len(row) > 2 else None
            label_rendered = self.font_tiny.render(label, True, MUTED)
            panel.blit(label_rendered, (12, y))
            rendered = self.font_small.render(value, True, color or WHITE)
            available_width = rect.width - 32 - label_rendered.get_width()
            if rendered.get_width() > available_width:
                rendered = self.font_tiny.render(value, True, color or WHITE)
            panel.blit(rendered, (rect.width - 12 - rendered.get_width(), y - 1))
            y += 18
            if y > rect.height - 15:
                break
        surface.blit(panel, rect)

    def _draw_control_hud(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
    ) -> None:
        if sim.control_mode is ControlMode.AUTO:
            label = self.font_bold.render("TAB  TAKE CONTROL", True, CYAN)
            chip = pygame.Surface((label.get_width() + 24, 34), pygame.SRCALPHA)
            chip.fill(PANEL)
            pygame.draw.rect(chip, (64, 143, 155), chip.get_rect(), 1)
            chip.blit(label, (12, 8))
            surface.blit(chip, (12, 84))
            return

        controlled = sim.controlled_vehicle
        if controlled is None:
            return
        rect = pygame.Rect(12, 84, 318, 164)
        panel = pygame.Surface(rect.size, pygame.SRCALPHA)
        panel.fill(PANEL)
        pygame.draw.rect(panel, CYAN, panel.get_rect(), 1)
        panel.blit(
            self.font_bold.render(sim.control_mode.value, True, CYAN),
            (12, 8),
        )
        panel.blit(
            self.font_tiny.render(
                f"{controlled.spec.name} // {controlled.spec.flight_model.upper()}",
                True,
                WHITE,
            ),
            (12, 29),
        )

        controls = sim.manual_input
        key_states = (
            ("W", controls.forward > 0.0),
            ("A", controls.turn < 0.0),
            ("S", controls.forward < 0.0),
            ("D", controls.turn > 0.0),
            ("Q", controls.vertical < 0.0),
            ("E", controls.vertical > 0.0),
            ("SHIFT", controls.full_thrust),
            ("CTRL", controls.brake),
        )
        x = 12
        for key, active in key_states:
            key_width = 48 if len(key) > 1 else 26
            box = pygame.Rect(x, 48, key_width, 22)
            pygame.draw.rect(
                panel,
                (20, 64, 67) if active else (10, 30, 39),
                box,
            )
            pygame.draw.rect(panel, GREEN if active else MUTED, box, 1)
            text = self.font_tiny.render(
                key,
                True,
                GREEN if active else MUTED,
            )
            panel.blit(
                text,
                (box.centerx - text.get_width() // 2, box.y + 4),
            )
            x += key_width + 5

        flight_model = controlled.spec.flight_model
        powered = controlled.engine_enabled
        wingborne = controlled.spec.lift_efficiency > 0.0
        authority = (
            ("FWD", GREEN if powered else RED),
            (
                "REV",
                GREEN
                if powered and flight_model in ("multirotor", "vectored_vtol")
                else AMBER if flight_model == "fixed_wing" else RED,
            ),
            (
                "TURN",
                GREEN
                if powered and flight_model in ("multirotor", "vectored_vtol")
                else AMBER
                if flight_model == "fixed_wing"
                or (flight_model == "rocket" and controlled.rcs_remaining_s > 0.0)
                or (flight_model == "vectored_vtol" and wingborne)
                else RED,
            ),
            (
                "VERT",
                GREEN
                if powered and flight_model in ("multirotor", "vectored_vtol")
                else AMBER
                if wingborne
                or (flight_model == "rocket" and controlled.rcs_remaining_s > 0.0)
                else RED,
            ),
            (
                "BRAKE",
                RED
                if flight_model == "rocket"
                else AMBER if flight_model == "fixed_wing" else GREEN,
            ),
        )
        x = 12
        for label, color in authority:
            rendered = self.font_tiny.render(f"{label} >", True, color)
            panel.blit(rendered, (x, 79))
            x += rendered.get_width() + 9

        requested = sim.manual_command.acceleration.length()
        actual = controlled.acceleration.length()
        panel.blit(
            self.font_tiny.render(
                f"REQUEST / ACTUAL A   {requested:5.2f} / {actual:5.2f} m/s2",
                True,
                WHITE,
            ),
            (12, 102),
        )
        brake_text = (
            "ON"
            if controlled.airbrake
            else "N/A" if flight_model == "rocket" else "OFF"
        )
        engine_detail = (
            f"{controlled.rocket_status} {controlled.main_burn_remaining_s:.1f}s "
            f"RCS {controlled.rcs_remaining_s:.1f}s"
            if flight_model == "rocket"
            else (
                f"ENGINE REQ/ACT {sim.manual_command.requested_engine*100:3.0f}"
                f"/{controlled.engine_output*100:3.0f}% "
                f"{'ON' if controlled.engine_enabled else 'CUT'}  BRAKE {brake_text}"
            )
        )
        panel.blit(
            self.font_tiny.render(
                engine_detail,
                True,
                RED
                if not controlled.engine_enabled
                else AMBER if controlled.airbrake else MUTED,
            ),
            (12, 121),
        )
        if sim.control_mode is ControlMode.INTERCEPTOR:
            advisory = (
                f"GUIDANCE ADVISORY {sim.guidance_advisory_command.length():.2f} m/s2"
                if sim.guidance is not None
                else "GUIDANCE ADVISORY: SENSOR SEARCH"
            )
        else:
            advisory = "OUR VEHICLE REMAINS UNDER AUTO GUIDANCE"
        if sim.manual_command.floor_protection:
            advisory = "ALTITUDE FLOOR ACTIVE // DOWN LIMITED"
        panel.blit(self.font_tiny.render(advisory, True, AMBER), (12, 141))
        surface.blit(panel, rect)

    def _toolbar_button(
        self,
        surface: pygame.Surface,
        key: str,
        rect: pygame.Rect,
        label: str,
        active: bool = False,
    ) -> None:
        self.click_regions[key] = rect
        pygame.draw.rect(
            surface,
            (14, 45, 54) if active else (6, 24, 34),
            rect,
        )
        pygame.draw.rect(surface, CYAN if active else (58, 104, 115), rect, 1)
        rendered = self.font_tiny.render(label, True, CYAN if active else WHITE)
        surface.blit(
            rendered,
            (
                rect.centerx - rendered.get_width() // 2,
                rect.centery - rendered.get_height() // 2,
            ),
        )

    def _draw_toolbar(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
    ) -> None:
        width = surface.get_width()
        labels = (
            ("minimap", "MAP [M]", self.minimap_visible),
            ("settings", "SETTINGS", self.settings_visible),
            ("verification", "VERIFY", self.verification_visible),
            ("capture_check", "CHECK 2s [G]", sim.prediction_check is not None),
            ("info", "INFO [F1]", False),
            ("analysis", "ANALYSIS [F2]", False),
            ("help", "KEYS [H]", False),
        )
        button_width = 88
        gap = 5
        total_width = len(labels) * button_width + (len(labels) - 1) * gap
        start_x = max(340, width - total_width - 14)
        y = 82
        for index, (key, label, active) in enumerate(labels):
            self._toolbar_button(
                surface,
                key,
                pygame.Rect(
                    start_x + index * (button_width + gap),
                    y,
                    button_width,
                    28,
                ),
                label,
                active,
            )

    def _draw_minimap(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
    ) -> None:
        if not self.minimap_visible:
            return
        width = surface.get_width()
        rect = pygame.Rect(width - 300, 120, 284, 220)
        panel = pygame.Surface(rect.size, pygame.SRCALPHA)
        panel.fill((4, 16, 23, 242))
        pygame.draw.rect(panel, CYAN, panel.get_rect(), 1)
        panel.blit(self.font_bold.render("TOP-DOWN MAP // WORLD X/Z", True, CYAN), (10, 8))

        map_rect = pygame.Rect(10, 34, rect.width - 20, rect.height - 46)
        pygame.draw.rect(panel, (5, 24, 29), map_rect)
        pygame.draw.rect(panel, (49, 91, 99), map_rect, 1)

        own = sim.interceptor.position
        estimated = (
            sim.track.position
            if sim.track.position is not None
            else sim.last_estimated_position
        )
        points = [own]
        points.extend(self.interceptor_trail[-80:])
        points.extend(self.estimated_target_trail[-80:])
        if estimated is not None:
            points.append(estimated)
        for contact in sim.contacts:
            contact_estimate = (
                contact.track.position
                if contact.track.position is not None
                else contact.last_estimated_position
            )
            if contact_estimate is not None:
                points.append(contact_estimate)
        min_x = min(point.x for point in points)
        max_x = max(point.x for point in points)
        min_z = min(point.z for point in points)
        max_z = max(point.z for point in points)
        span = max(100.0, max_x - min_x + 40.0, max_z - min_z + 40.0)
        center_x = (min_x + max_x) * 0.5
        center_z = (min_z + max_z) * 0.5
        scale = min(map_rect.width, map_rect.height) / span

        def map_point(point: Vec3) -> tuple[int, int]:
            return (
                int(map_rect.centerx + (point.x - center_x) * scale),
                int(map_rect.centery - (point.z - center_z) * scale),
            )

        grid_step = 50.0
        half_lines = int(span / grid_step) + 2
        for index in range(-half_lines, half_lines + 1):
            world_x = math.floor(center_x / grid_step) * grid_step + index * grid_step
            x = map_point(Vec3(world_x, 0, center_z))[0]
            pygame.draw.line(panel, (21, 51, 56), (x, map_rect.top), (x, map_rect.bottom))
            world_z = math.floor(center_z / grid_step) * grid_step + index * grid_step
            y = map_point(Vec3(center_x, 0, world_z))[1]
            pygame.draw.line(panel, (21, 51, 56), (map_rect.left, y), (map_rect.right, y))

        own_px = map_point(own)
        fov_forward = Vec3(
            sim.camera_forward.x,
            0.0,
            sim.camera_forward.z,
        ).normalized(Vec3(0, 0, 1))
        fov_angle = math.radians(sim.config.camera.horizontal_fov_deg * 0.5)
        fov_length = span * 0.58
        base_yaw = math.atan2(fov_forward.x, fov_forward.z)
        for sign in (-1.0, 1.0):
            yaw = base_yaw + sign * fov_angle
            end = own + Vec3(math.sin(yaw), 0, math.cos(yaw)) * fov_length
            pygame.draw.line(panel, (58, 143, 153), own_px, map_point(end), 1)

        if len(self.interceptor_trail) > 1:
            pygame.draw.lines(
                panel,
                (34, 123, 133),
                False,
                [map_point(point) for point in self.interceptor_trail[-80:]],
                1,
            )
        if len(self.estimated_target_trail) > 1:
            pygame.draw.lines(
                panel,
                (122, 84, 76),
                False,
                [map_point(point) for point in self.estimated_target_trail[-80:]],
                1,
            )

        pygame.draw.circle(panel, CYAN, own_px, 5)
        own_heading = own + sim.interceptor.forward_direction() * 12.0
        pygame.draw.line(panel, CYAN, own_px, map_point(own_heading), 2)
        panel.blit(self.font_tiny.render("OUR", True, CYAN), (own_px[0] + 7, own_px[1] - 7))

        if estimated is not None:
            altitude_delta = estimated.y - own.y
            brightness = clamp(0.55 + altitude_delta / 100.0, 0.28, 1.0)
            base_color = sim.target.spec.color
            target_color = tuple(int(channel * brightness) for channel in base_color)
            target_px = map_point(estimated)
            pygame.draw.circle(panel, target_color, target_px, 6)
            symbol = "^" if altitude_delta > 1.0 else "v" if altitude_delta < -1.0 else "="
            panel.blit(
                self.font_tiny.render(
                    f"TGT {symbol} {altitude_delta:+.0f}m",
                    True,
                    target_color,
                ),
                (target_px[0] + 8, target_px[1] - 7),
            )
        for index, contact in enumerate(sim.contacts):
            if index == sim.active_contact_index:
                continue
            contact_estimate = (
                contact.track.position
                if contact.track.position is not None
                else contact.last_estimated_position
            )
            if contact_estimate is None:
                continue
            altitude_delta = contact_estimate.y - own.y
            brightness = clamp(0.55 + altitude_delta / 100.0, 0.28, 1.0)
            contact_color = tuple(
                int(channel * brightness)
                for channel in contact.vehicle.spec.color
            )
            contact_px = map_point(contact_estimate)
            pygame.draw.circle(panel, contact_color, contact_px, 4, 1)
            panel.blit(
                self.font_tiny.render(
                    contact.track_id,
                    True,
                    contact_color,
                ),
                (contact_px[0] + 6, contact_px[1] - 6),
            )

        panel.blit(
            self.font_tiny.render(
                "BRIGHTER = ABOVE  //  DARKER = BELOW  //  WEDGE = SENSOR FOV",
                True,
                MUTED,
            ),
            (10, rect.height - 15),
        )
        surface.blit(panel, rect)

    def _draw_settings(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
        time_scale: float,
    ) -> None:
        if not self.settings_visible:
            return
        rect = pygame.Rect(340, 120, 330, 210)
        panel = pygame.Surface(rect.size, pygame.SRCALPHA)
        panel.fill((4, 16, 25, 244))
        pygame.draw.rect(panel, CYAN, panel.get_rect(), 1)
        panel.blit(self.font_bold.render("PRESENTATION SETTINGS", True, CYAN), (12, 10))
        rows = (
            ("TIME SCALE", f"{time_scale:g}x"),
            ("MINIMAP", "ON" if self.minimap_visible else "OFF"),
            ("AIM AIDS", "ON" if self.aim_aids_visible else "OFF"),
            (
                "TERMINAL",
                "ENTER 1s OVAL"
                if sim.terminal_mode == "ONE_SECOND_ENVELOPE"
                else "TTC <= 1.0s",
            ),
        )
        y = 48
        for label, value in rows:
            panel.blit(self.font_tiny.render(label, True, MUTED), (12, y + 6))
            panel.blit(self.font_small.render(value, True, WHITE), (126, y + 4))
            y += 33
        button_specs = (
            ("time_down", pygame.Rect(238, 43, 34, 25), "-"),
            ("time_up", pygame.Rect(282, 43, 34, 25), "+"),
            ("minimap", pygame.Rect(238, 76, 78, 25), "TOGGLE"),
            ("aim_aids", pygame.Rect(238, 109, 78, 25), "TOGGLE"),
            ("terminal_mode", pygame.Rect(238, 142, 78, 25), "CHANGE"),
            ("panels_reset", pygame.Rect(12, 178, 130, 23), "RESET PANELS"),
        )
        for key, local_rect, label in button_specs:
            pygame.draw.rect(panel, (10, 36, 46), local_rect)
            pygame.draw.rect(panel, (59, 117, 127), local_rect, 1)
            rendered = self.font_tiny.render(label, True, WHITE)
            panel.blit(
                rendered,
                (
                    local_rect.centerx - rendered.get_width() // 2,
                    local_rect.centery - rendered.get_height() // 2,
                ),
            )
            self.click_regions[key] = local_rect.move(rect.x, rect.y)
        surface.blit(panel, rect)

    def _draw_verification(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
    ) -> None:
        if not self.verification_visible:
            return
        width, height = surface.get_size()
        rect = pygame.Rect(width // 2 - 215, max(345, height - 410), 430, 190)
        panel = pygame.Surface(rect.size, pygame.SRCALPHA)
        panel.fill((4, 16, 25, 245))
        pygame.draw.rect(panel, AMBER, panel.get_rect(), 1)
        panel.blit(
            self.font_bold.render(
                "SIMULATION-TRUTH VERIFICATION ONLY",
                True,
                AMBER,
            ),
            (12, 9),
        )
        truth = sim.target.position
        estimate = sim.track.position
        error = truth.distance_to(estimate) if estimate is not None else math.nan
        panel.blit(
            self.font_tiny.render(
                f"TRUTH XYZ {truth.x:+.1f}/{truth.y:+.1f}/{truth.z:+.1f} m",
                True,
                MUTED,
            ),
            (12, 34),
        )
        panel.blit(
            self.font_tiny.render(
                (
                    f"EST. XYZ  {estimate.x:+.1f}/{estimate.y:+.1f}/{estimate.z:+.1f} m"
                    f"  ERROR {error:.2f} m"
                    if estimate is not None
                    else "EST. XYZ  NO CURRENT VISUAL TRACK"
                ),
                True,
                WHITE,
            ),
            (12, 51),
        )

        inset_rect = pygame.Rect(12, 74, 242, 101)
        pygame.draw.rect(panel, (2, 10, 16), inset_rect)
        pygame.draw.rect(panel, (57, 93, 101), inset_rect, 1)
        check = sim.prediction_check
        if check is None:
            panel.blit(
                self.font_tiny.render(
                    "Press G or CAPTURE to record the current +2s sensor frame.",
                    True,
                    MUTED,
                ),
                (20, 114),
            )
            status = "NO RECORDED CHECK"
            status_color = MUTED
        else:
            inset = panel.subsurface(inset_rect)
            recorded_camera = ViewCamera(
                check.camera_position,
                check.camera_forward,
                "RECORDED SENSOR FRAME",
            )
            self._draw_polyline_3d(
                inset,
                self._oval_points(check.oval),
                recorded_camera,
                GREEN if check.result_inside else AMBER,
                2,
                self.DEFAULT_PRESENTATION_FOV_DEG,
            )
            marker_position = (
                check.projected_truth
                if check.evaluated and check.projected_truth is not None
                else sim.target.position
            )
            projected = self._project_cached(
                marker_position,
                recorded_camera,
                inset_rect.width,
                inset_rect.height,
                self.DEFAULT_PRESENTATION_FOV_DEG,
            )
            if projected:
                pygame.draw.circle(
                    inset,
                    GREEN if check.result_inside else RED,
                    projected[:2],
                    5,
                    2,
                )
            if check.evaluated:
                status = "INSIDE" if check.result_inside else "OUTSIDE"
                status_color = GREEN if check.result_inside else RED
            else:
                remaining = max(
                    0.0,
                    check.capture_time_s + check.oval.horizon_s - sim.time_s,
                )
                status = f"RECORDED FRAME // CHECK IN {remaining:.2f}s"
                status_color = AMBER
        panel.blit(
            self.font_small.render(
                f"RESULT: {status}" if check is not None and check.evaluated else status,
                True,
                status_color,
            ),
            (266, 82),
        )
        explanatory = (
            "RECORDED CAMERA FIXED.",
            "LIVE VIEW MAY MOVE.",
            "TRUTH CHECK AT T+2.",
        )
        for index, line in enumerate(explanatory):
            panel.blit(
                self.font_tiny.render(line, True, MUTED),
                (266, 110 + index * 17),
            )

        capture_local = pygame.Rect(rect.width - 144, rect.height - 27, 92, 22)
        clear_local = pygame.Rect(rect.width - 47, rect.height - 27, 42, 22)
        self.click_regions["capture_check"] = capture_local.move(rect.x, rect.y)
        self.click_regions["clear_check"] = clear_local.move(rect.x, rect.y)
        for button_rect, label in ((capture_local, "CAPTURE"), (clear_local, "CLEAR")):
            pygame.draw.rect(panel, (10, 36, 46), button_rect)
            pygame.draw.rect(panel, AMBER, button_rect, 1)
            rendered = self.font_tiny.render(label, True, WHITE)
            panel.blit(
                rendered,
                (
                    button_rect.centerx - rendered.get_width() // 2,
                    button_rect.centery - rendered.get_height() // 2,
                ),
            )
        surface.blit(panel, rect)

    def draw_hud(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
        time_scale: float,
        fps: float,
    ) -> None:
        self.click_regions = {}
        width, height = surface.get_size()
        top = pygame.Surface((width, 54), pygame.SRCALPHA)
        top.fill((3, 12, 20, 235))
        top.blit(self.font_title.render("ZENITH", True, WHITE), (18, 9))
        top.blit(
            self.font_small.render("VISION-ONLY INTERCEPTION // PROTOTYPE 02", True, CYAN),
            (132, 19),
        )
        status_color = GREEN if sim.hit else (CYAN if sim.identity_confirmed else AMBER)
        status_text = self.font_bold.render(sim.status, True, status_color)
        top.blit(status_text, (width - status_text.get_width() - 20, 10))
        details = self.font_tiny.render(
            f"T+ {sim.time_s:06.2f}s   TIME x{time_scale:g}   {fps:4.0f} FPS   "
            f"CONTACTS {sim.visible_contact_count}/{len(sim.contacts)}   "
            f"PRIORITY {sim.active_contact.track_id}   "
            f"SENSOR {sim.config.camera.width_px}x{sim.config.camera.height_px}",
            True,
            MUTED,
        )
        top.blit(details, (width - details.get_width() - 20, 33))
        surface.blit(top, (0, 0))

        surface.blit(
            self.font_tiny.render(
                f"VIEW: {self.get_view_camera(sim).name}   ZOOM x{self.zoom_multiplier:.2f} [WHEEL]   [V] SWITCH   [F3] SPECTATOR   [RMB] LOOK   [F1] INFO   [H] KEYS",
                True,
                MUTED,
            ),
            (16, 62),
        )
        self._draw_control_hud(surface, sim)
        self._draw_toolbar(surface, sim)
        if sim.paused:
            pause = self.font_large.render("SIMULATION PAUSED", True, AMBER)
            surface.blit(pause, (width // 2 - pause.get_width() // 2, 82))

        if sim.hit:
            success = self.font_large.render(sim.success_message, True, GREEN)
            box = pygame.Surface((success.get_width() + 36, 58), pygame.SRCALPHA)
            box.fill((5, 28, 25, 220))
            pygame.draw.rect(box, GREEN, box.get_rect(), 2)
            box.blit(success, (18, 10))
            surface.blit(box, (width // 2 - box.get_width() // 2, 105))

        margin, gap = 10, 8
        panel_width = (width - margin * 2 - gap * 3) // 4
        panel_height_expanded = 190

        target_name = (
            f"{sim.active_contact_label} {sim.target.spec.name}"
            if sim.identity_confirmed
            else f"{sim.active_contact_label} ???"
        )
        target_axial_accel = (
            sim.track.acceleration.dot(sim.track.velocity.normalized())
            if sim.track.velocity.length() > 0.2
            else 0.0
        )
        target_rows = [
            ("MODEL", target_name, sim.target.spec.color if sim.identity_confirmed else AMBER),
            ("PROPULSION", sim.target.spec.flight_model.replace("_", " ").upper() if sim.identity_confirmed else "QUERYING"),
            ("MAX SPEED", f"{sim.target.spec.max_speed:.1f} m/s" if sim.identity_confirmed else "QUERYING"),
            ("ACCEL / BRAKE", f"{sim.target.spec.max_accel:.1f} / {sim.target.spec.brake_accel:.1f} m/s²" if sim.identity_confirmed else "QUERYING"),
            ("SIZE W/H/L", sim.target.spec.size_label if sim.identity_confirmed else "QUERYING"),
            ("SPEED / AXIAL A", f"{sim.track.velocity.length():.2f} / {target_axial_accel:+.2f}" if sim.visual_locked and sim.track.sample_count > 2 else ("NO VISUAL LOCK" if sim.identity_confirmed and not sim.visual_locked else "ANGULAR ONLY")),
            (
                "EST. POSITION XYZ",
                (
                    f"{sim.track.position.x:+.0f}/{sim.track.position.y:+.0f}/"
                    f"{sim.track.position.z:+.0f} m"
                    if sim.track.position is not None
                    else "---"
                ),
                WHITE if sim.visual_locked else AMBER,
            ),
            (
                "SCENARIO",
                (
                    f"TRICKY: {sim.evader_decision}"
                    if sim.config.scenario == "tricky"
                    else dict(SCENARIOS)[sim.config.scenario]
                ),
                AMBER if sim.config.scenario == "tricky" else WHITE,
            ),
        ]

        estimate = sim.range_estimate
        if sim.track.position:
            relative_camera = camera_coordinates(
                sim.track.position, sim.interceptor.position, sim.camera_forward
            )
        else:
            relative_camera = Vec3()
        apparent_range_span = (
            min(sim.detection.width_px, sim.detection.height_px)
            if sim.config.detector_backend == "yolo"
            and sim.target.spec.flight_model == "rocket"
            else max(sim.detection.width_px, sim.detection.height_px)
        )
        detector_value = (
            (
                f"YOLO {sim.detector_metrics.inference_ms:.1f}ms "
                f"{sim.detector_metrics.detection_count} BOX"
            )
            if sim.config.detector_backend == "yolo"
            else "SYNTHETIC BOX + POSE"
        )
        calc_rows = [
            (
                "DETECTOR",
                detector_value if sim.detector_error is None else "YOLO ERROR",
                GREEN
                if sim.config.detector_backend == "yolo"
                and sim.detector_error is None
                else RED
                if sim.detector_error is not None
                else CYAN,
            ),
            ("FORMULA", "Z = f × S / p", CYAN),
            ("FOCAL LENGTH", f"{sim.config.camera.focal_px:.1f} px"),
            (
                "LAST RANGE p / CONF"
                if sim.hit
                else "RANGE p / CONF",
                (
                    f"{apparent_range_span:.2f} px / "
                    f"{sim.detection.confidence*100:.0f}%"
                ),
            ),
            ("RANGE AT IMPACT" if sim.hit else "RANGE EST.", f"{estimate.distance_m:.2f} ± {estimate.sigma_m:.2f} m" if estimate else "WAITING FOR SIZE"),
            ("LAST Dx  LEFT/RIGHT" if sim.hit else "Dx  LEFT/RIGHT", f"{relative_camera.x:+.2f} m" if estimate else "---"),
            ("LAST Dy  UP/DOWN" if sim.hit else "Dy  UP/DOWN", f"{relative_camera.y:+.2f} m" if estimate else "---"),
            ("LAST Dz  LINE OF SIGHT" if sim.hit else "Dz  LINE OF SIGHT", f"{relative_camera.z:+.2f} m" if estimate else "---"),
        ]

        own = sim.interceptor
        own_rows = [
            ("MODEL", own.spec.name, own.spec.color),
            ("PROPULSION", own.spec.flight_model.replace("_", " ").upper()),
            ("POSITION XYZ", f"{own.position.x:+.0f}/{own.position.y:+.0f}/{own.position.z:+.0f} m"),
            ("VELOCITY XYZ", f"{own.velocity.x:+.1f}/{own.velocity.y:+.1f}/{own.velocity.z:+.1f}"),
            ("TOTAL SPEED", f"{own.velocity.length():.2f} / {own.spec.max_speed:.0f} m/s"),
            ("ACCELERATION", f"{own.acceleration.length():.2f} m/s²"),
            (
                "ENGINE / LIFT",
                f"{own.rocket_status if own.spec.flight_model == 'rocket' else ('ON' if own.engine_enabled else 'CUT')} "
                f"{own.engine_output*100:.0f}% / "
                f"{own.lift_acceleration.length():.1f} m/s2",
                RED if not own.engine_enabled else WHITE,
            ),
        ]
        if own.spec.flight_model == "rocket":
            own_rows.append(
                (
                    "BOOSTER / RCS",
                    f"{own.main_burn_remaining_s:.1f}s / {own.rcs_remaining_s:.1f}s",
                    AMBER if own.engine_enabled else MUTED,
                )
            )

        if sim.hit:
            relative_velocity = sim.target.velocity - own.velocity
            right, up, forward = basis_from_forward(sim.camera_forward)
            relative_v = Vec3(
                relative_velocity.dot(right),
                relative_velocity.dot(up),
                relative_velocity.dot(forward),
            )
        elif sim.track.position:
            right, up, forward = basis_from_forward(sim.camera_forward)
            relative_velocity = sim.track.velocity - own.velocity
            relative_v = Vec3(
                relative_velocity.dot(right),
                relative_velocity.dot(up),
                relative_velocity.dot(forward),
            )
        else:
            relative_v = Vec3()
        guidance = sim.guidance
        if sim.hit:
            relative_rows = [
                ("REL. Vx / Vy", f"{relative_v.x:+.2f} / {relative_v.y:+.2f} m/s"),
                ("REL. Vz", f"{relative_v.z:+.2f} m/s"),
                ("CONTACT TIME", f"T+ {sim.hit_time_s:.3f} s" if sim.hit_time_s else "---", GREEN),
                ("GUIDANCE", "DISABLED AFTER CONTACT", GREEN),
                ("IMPACT TEST", "CONTINUOUS SEGMENT", WHITE),
                ("CRASH PHYSICS", "GRAVITY + TUMBLE", WHITE),
                ("VERIFY", "CONTACT CONFIRMED", GREEN),
            ]
        else:
            relative_xy = (
                f"{relative_v.x:+.2f} / {relative_v.y:+.2f} m/s"
                if sim.visual_locked
                else "---"
            )
            relative_z = f"{relative_v.z:+.2f} m/s" if sim.visual_locked else "---"
            displayed_guidance_mode = (
                sim.multi_guidance_mode
                if len(sim.contacts) > 1
                else guidance.mode
                if guidance
                else "BEARING PURSUIT"
                if sim.visual_locked
                else "NO VISUAL LOCK"
            )
            relative_rows = [
                ("REL. Vx / Vy", relative_xy),
                ("REL. Vz", relative_z),
                ("CLOSING SPEED", f"{guidance.closing_speed:+.2f} m/s" if guidance else "---"),
                ("TIME TO CONTACT", f"{guidance.time_to_contact_s:.2f} s" if guidance and math.isfinite(guidance.time_to_contact_s) else "---"),
                ("GUIDANCE", displayed_guidance_mode),
                (
                    "OVAL / EDGE",
                    (
                        f"SHARED {sim.shared_overlap.horizon_s:.0f}s / "
                        f"{sim.contacts[sim.shared_overlap_pair[0]].track_id}+"
                        f"{sim.contacts[sim.shared_overlap_pair[1]].track_id}"
                        if (
                            sim.shared_overlap is not None
                            and sim.shared_overlap_pair is not None
                        )
                        else
                        f"{guidance.selected_horizon_s:.1f}s / "
                        f"{guidance.reachable_count}/{guidance.reachable_total}"
                        if guidance and guidance.selected_horizon_s
                        else "SEARCHING" if not sim.visual_locked else "---"
                    ),
                ),
                (
                    "TRACK SIGMA",
                    (
                        f"{sim.track.position_sigma_m:.2f}m / "
                        f"{sim.track.velocity_sigma_mps:.2f}m/s"
                        if sim.visual_locked
                        else "---"
                    ),
                    MUTED,
                ),
                (
                    "PRIORITY SCORE",
                    f"{sim.active_contact.priority_score:.1f} / IMAGE TRACKS",
                    CYAN,
                ),
            ]

        panels = (
            ("target", "PRIORITY TARGET", target_rows),
            ("calculations", "CALCULATIONS", calc_rows),
            ("own", "OUR VEHICLE", own_rows),
            ("relative", "RELATIVE", relative_rows),
        )
        for index, (key, title, rows) in enumerate(panels):
            expanded = self.panel_expanded[key]
            panel_height = panel_height_expanded if expanded else 32
            panel_y = height - panel_height - 10
            rect = pygame.Rect(
                margin + index * (panel_width + gap),
                panel_y,
                panel_width,
                panel_height,
            )
            self._panel(
                surface,
                rect,
                f"{'v' if expanded else '>'} {title}",
                rows if expanded else [],
            )
            self.click_regions[f"panel:{key}"] = pygame.Rect(
                rect.x,
                rect.y,
                rect.width,
                32,
            )

        self._draw_minimap(surface, sim)
        self._draw_settings(surface, sim, time_scale)
        self._draw_verification(surface, sim)

    def draw_analysis(self, surface: pygame.Surface, sim: InterceptionSimulation) -> None:
        width, height = surface.get_size()
        overlay = pygame.Surface((width, height), pygame.SRCALPHA)
        overlay.fill((2, 8, 14, 238))
        margin = 42
        pygame.draw.rect(overlay, (59, 136, 151), (margin, margin, width - margin * 2, height - margin * 2), 1)
        overlay.blit(self.font_title.render("ENGINEERING ANALYSIS // MONOCULAR RANGE", True, WHITE), (margin + 24, margin + 18))
        overlay.blit(self.font_small.render("[F2] close analysis", True, MUTED), (width - margin - 175, margin + 25))
        overlay.blit(
            self.font.render("Pinhole model:  range Z = focal length f × known target span S ÷ apparent pixel span p", True, CYAN),
            (margin + 24, margin + 58),
        )

        graph = pygame.Rect(margin + 34, margin + 110, int((width - margin * 2) * 0.56), height - margin * 2 - 168)
        pygame.draw.rect(overlay, (10, 24, 34), graph)
        pygame.draw.rect(overlay, (64, 116, 128), graph, 1)
        spec = sim.target.spec
        camera = sim.config.camera
        focal = camera.focal_px
        yolo_range = sim.config.detector_backend == "yolo"
        if yolo_range:
            reference_size = (
                max(spec.dimensions.x, spec.dimensions.y)
                if spec.flight_model == "rocket"
                else max(spec.dimensions.x, spec.dimensions.z)
                * YOLO_DRONE_BOX_CALIBRATION
            )
        else:
            reference_size = spec.dimensions.x
        apparent_range_span = (
            min(sim.detection.width_px, sim.detection.height_px)
            if yolo_range and spec.flight_model == "rocket"
            else max(sim.detection.width_px, sim.detection.height_px)
        )
        pixels = list(range(2, 201, 2))
        ranges = [focal * reference_size / value for value in pixels]
        max_range = min(1600.0, ranges[0])

        def graph_point(pixel: float, distance: float) -> tuple[int, int]:
            x = graph.left + int((pixel - 2) / 198 * graph.width)
            y = graph.bottom - int(clamp(distance / max_range, 0, 1) * graph.height)
            return x, y

        for fraction in (0.25, 0.5, 0.75):
            y = graph.bottom - int(graph.height * fraction)
            pygame.draw.line(overlay, (31, 58, 68), (graph.left, y), (graph.right, y), 1)
            label = self.font_tiny.render(f"{max_range*fraction:.0f}m", True, MUTED)
            overlay.blit(label, (graph.left + 4, y + 3))
        curve = [graph_point(pixel, distance) for pixel, distance in zip(pixels, ranges)]
        pygame.draw.lines(overlay, CYAN, False, curve, 2)

        # +/- half-pixel edge quantisation envelope.
        upper = [
            graph_point(pixel, focal * reference_size / max(0.5, pixel - 0.5))
            for pixel in pixels
        ]
        lower = [
            graph_point(pixel, focal * reference_size / (pixel + 0.5))
            for pixel in reversed(pixels)
        ]
        band = upper + lower
        band_surface = pygame.Surface((width, height), pygame.SRCALPHA)
        pygame.draw.polygon(band_surface, (72, 224, 238, 35), band)
        overlay.blit(band_surface, (0, 0))
        overlay.blit(self.font_bold.render("RANGE vs APPARENT TARGET SIZE", True, WHITE), (graph.left, graph.top - 25))
        overlay.blit(self.font_tiny.render("apparent pixels →", True, MUTED), (graph.centerx - 45, graph.bottom + 10))
        for pixel in (2, 10, 25, 50, 100, 200):
            x, _ = graph_point(pixel, 0)
            pygame.draw.line(overlay, (64, 116, 128), (x, graph.bottom), (x, graph.bottom + 5), 1)
            overlay.blit(self.font_tiny.render(str(pixel), True, MUTED), (x - 8, graph.bottom + 7))

        right_x = graph.right + 34
        right_width = width - margin - 28 - right_x
        overlay.blit(self.font_bold.render("WHY SMALL TARGETS ARE DIFFICULT", True, AMBER), (right_x, graph.top))
        range_method_lines = (
            [
                "YOLO supplies no 3D pose. The calculation uses",
                "a known planform/cross-section span and declares",
                "orientation/confidence uncertainty in the HUD.",
            ]
            if yolo_range
            else [
                "Pose-compensated fitting uses the known 3D",
                "dimensions. A width-only estimate is also logged",
                "so rotation sensitivity remains verifiable.",
            ]
        )
        explanation = [
            "A ±0.5 px box-edge error produces approximately:",
            "    relative range error ≈ ±0.5 / p",
            "At 4 px:  ±12.5%    At 20 px: ±2.5%",
            "At 100 px: ±0.5%",
            "",
            *range_method_lines,
        ]
        y = graph.top + 30
        for line in explanation:
            overlay.blit(self.font_small.render(line, True, WHITE if line else MUTED), (right_x, y))
            y += 21

        overlay.blit(self.font_bold.render("MINIMUM HORIZONTAL CAMERA RESOLUTION", True, AMBER), (right_x, y + 12))
        y += 44
        overlay.blit(
            self.font_tiny.render(
                f"12 px analysis requirement // {camera.horizontal_fov_deg:.0f} DEG horizontal FOV",
                True,
                MUTED,
            ),
            (right_x, y),
        )
        y += 24
        for distance in (100, 250, 500, 1000):
            resolution = minimum_horizontal_resolution(
                distance,
                reference_size,
                camera.horizontal_fov_deg,
                12,
            )
            color = GREEN if resolution <= 3840 else (AMBER if resolution <= 7680 else RED)
            line = f"{distance:4d} m  →  {resolution:5d} horizontal pixels"
            overlay.blit(self.font.render(line, True, color), (right_x, y))
            y += 25

        y += 18
        overlay.blit(self.font_bold.render("CURRENT FRAME VERIFICATION", True, AMBER), (right_x, y))
        y += 30
        current_lines = [
            (
                f"Target: {spec.name} "
                f"({reference_size:.2f} m "
                f"{'known image span' if yolo_range else 'reference width'})"
            ),
            f"Sensor focal length: {focal:.2f} px",
            f"Ranging pixel span p: {apparent_range_span:.3f} px",
            f"Estimated range: {sim.range_estimate.distance_m:.3f} m" if sim.range_estimate else "Estimated range: waiting for identity",
            f"Ground-truth range: {sim.true_range_m:.3f} m [verification only]",
        ]
        for line in current_lines:
            overlay.blit(self.font_small.render(line, True, WHITE), (right_x, y))
            y += 22

        surface.blit(overlay, (0, 0))

    def draw_info(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
        page: int,
    ) -> None:
        """Four-page in-application reference for the whole prototype."""
        page = page % INFO_PAGE_COUNT
        width, height = surface.get_size()
        overlay = pygame.Surface((width, height), pygame.SRCALPHA)
        overlay.fill((2, 8, 14, 242))
        card = pygame.Rect(width // 2 - 470, height // 2 - 310, 940, 620)
        pygame.draw.rect(overlay, (7, 22, 32), card)
        pygame.draw.rect(overlay, CYAN, card, 1)

        page_titles = (
            "SENSOR, COORDINATES, AND READOUTS",
            "PREDICTION OVALS AND GUIDANCE",
            "GRAVITY, AERODYNAMICS, AND CONTROL",
            "LOCK LOSS, CAMERAS, AND VERIFICATION",
        )
        title = f"FULL SYSTEM INFO {page + 1}/{INFO_PAGE_COUNT} // {page_titles[page]}"
        overlay.blit(
            self.font_title.render(title, True, WHITE),
            (card.x + 26, card.y + 20),
        )
        navigation = self.font_tiny.render(
            "[F1] CLOSE   [LEFT/RIGHT or PAGE UP/DOWN] CHANGE PAGE",
            True,
            MUTED,
        )
        overlay.blit(
            navigation,
            (card.right - navigation.get_width() - 24, card.y + 54),
        )
        pygame.draw.line(
            overlay,
            (42, 92, 103),
            (card.x + 24, card.y + 76),
            (card.right - 24, card.y + 76),
            1,
        )

        estimated_range = (
            f"{sim.range_estimate.distance_m:.2f} +/- "
            f"{sim.range_estimate.sigma_m:.2f} m"
            if sim.range_estimate is not None
            else "unavailable without a current known-model detection"
        )
        current_aim = (
            sim.shared_overlap.aim_point
            if sim.shared_overlap is not None
            else sim.guidance.aim_point
            if sim.guidance is not None
            else None
        )
        current_guidance = (
            f"{sim.multi_guidance_mode if len(sim.contacts) > 1 else sim.guidance.mode}; "
            f"aim ({current_aim.x:+.1f}, "
            f"{current_aim.y:+.1f}, "
            f"{current_aim.z:+.1f})"
            if current_aim is not None and sim.guidance is not None
            else "disabled: no guidance-quality visual observation"
        )
        oval_rows = []
        if sim.guidance is not None:
            for oval in sim.guidance.ovals:
                selected = (
                    sim.guidance.mode == "WEIGHTED OVAL"
                    and sim.guidance.selected_horizon_s is not None
                    and abs(
                        sim.guidance.selected_horizon_s - oval.horizon_s
                    )
                    < 0.02
                )
                oval_rows.append(
                    f"{oval.horizon_s:.0f}s: radii "
                    f"{oval.radius_x:.1f}/{oval.radius_y:.1f} m, "
                    f"edge {oval.edge_reachable_count}/{oval.edge_total}, "
                    f"cardinal {oval.cardinal_reachable_count}/4"
                    f"{' [SELECTED]' if selected else ''}"
                )
        else:
            oval_rows.append("No live ovals: visual guidance is unavailable.")

        if page == 0:
            sections = (
                (
                    "SENSOR PIPELINE",
                    [
                        (
                            f"Active detector: YOLO CUSTOM WEIGHTS; last inference {sim.detector_metrics.inference_ms:.1f} ms on {sim.detector_metrics.device}."
                            if sim.config.detector_backend == "yolo"
                            else "Active detector: SYNTHETIC BOX + POSE; select YOLO CUSTOM on setup to use trained pixel inference."
                        ),
                        f"{len(sim.contacts)} contacts run independent boxes, signal timers, ranges, tracks, ovals, and guidance; current state: {sim.multi_guidance_mode}.",
                        "Only after visual detection does the simulated signal query reveal the requested vehicle model and its real dimensions.",
                        "YOLO receives a clean BGR sensor frame without HUD, labels, ovals, actor coordinates, or dataset annotations.",
                        "Range uses pinhole known-size ranging; YOLO uses a known planform/cross-section span and declares larger pose ambiguity.",
                        f"Current model: {sim.target.spec.name if sim.identity_confirmed else 'UNKNOWN / QUERYING'}",
                        f"Current camera estimate: {estimated_range}",
                    ],
                ),
                (
                    "CAMERA-RELATIVE AXES",
                    [
                        "X is target left/right in the image.",
                        "Y is target up/down in the image.",
                        "Z is the optical line from our camera toward the target.",
                        "World coordinates exist for simulation and collision, but target truth never enters range or guidance.",
                    ],
                ),
                (
                    "THE FOUR BOTTOM PANELS",
                    [
                        "PRIORITY TARGET: selected contact ID, signal-resolved dimensions, and maneuver limits.",
                        "CALCULATIONS: detector, measured pixels, confidence, range uncertainty, and Dx/Dy/Dz; expanded by default.",
                        "OUR VEHICLE: integrated position, velocity, acceleration, engine/lift, and rocket fuel where applicable.",
                        "RELATIVE: image-track relative velocity, closing time, guidance, full-edge reachability, and track uncertainty.",
                    ],
                ),
                (
                    "WHAT IS VERIFIABLE",
                    [
                        f"Sensor: {sim.config.camera.width_px} x {sim.config.camera.height_px}, focal length {sim.config.camera.focal_px:.1f} px.",
                        f"Contacts visible/identified: {sim.visible_contact_count}/{sim.identified_contact_count} of {len(sim.contacts)}.",
                        f"Detected span: {max(sim.detection.width_px, sim.detection.height_px):.2f} px.",
                        f"True range: {sim.true_range_m:.2f} m [verification only].",
                        "F2 opens the resolution/error graph. F5 exports every sampled estimate and its marked truth comparison.",
                    ],
                ),
            )
        elif page == 1:
            sections = (
                (
                    "WHAT EACH OVAL MEANS",
                    [
                        "At 60 Hz the current camera pose is frozen and the 1, 2, 3, and 5 second ellipses bound the pixels the target could occupy.",
                        "Maneuver authority, perspective, and bounded track uncertainty inflate an outer containment border.",
                        "Each pixel ellipse is back-projected onto the plane through the estimated target, perpendicular to the current sensor axis.",
                        "A possible camera-plane crossing is labeled UNBOUNDED instead of drawing a false finite ellipse.",
                    ],
                ),
                (
                    "COLOR AND COMPLETE-EDGE RULE",
                    [
                        "Each GREEN segment is an edge point our vehicle can reach by that horizon; each RED segment is one it cannot.",
                        "A mixed red/green border is partial. Only a 96/96 green loop is selectable and receives a faint green interior tint.",
                        "RED never fills an oval: a blocked edge does not mean its center or every interior point is blocked.",
                        "Four large cardinal markers remain as a simple 4/4 explanation, but they do not decide the border color.",
                        "AMBER dots are the unchanged-velocity trajectory, not a partially valid oval.",
                    ],
                ),
                (
                    "LIVE OVAL CHECK",
                    oval_rows,
                ),
                (
                    "HOW THE AIM IS CHOSEN",
                    [
                        "Multi-contact mode compares matching horizons, aims at one cyan overlap centroid, then commits on smaller-oval entry.",
                        "Single-target mode checks 5s, 3s, 2s, then 1s and biases the largest green oval toward likely motion.",
                        "If none pass, it reports NO GUARANTEED OVAL and follows the transparent unchanged-motion point.",
                        "In single-target mode, entering the one-second oval switches to camera-derived terminal collision lead.",
                        f"Current guidance: {current_guidance}",
                    ],
                ),
            )
        elif page == 2:
            sections = (
                (
                    "GRAVITY AND FORCE MODEL",
                    [
                        "Gravity is always 9.81 m/s2 downward for every airborne vehicle.",
                        "Aegis-Q4 and Smart Evader spawn level at half maximum speed; forward thrust tilts their noses down, not up.",
                        "Rotorcraft hover only because their powered thrust explicitly counters gravity; cutting the engine removes that support.",
                        "Drag grows with speed. Airbrakes add continuous opposing acceleration instead of deleting speed instantly.",
                        "Autonomous fixed wings schedule throttle and closing speed early; they never use reverse thrust or an automatic speed-triggered brake.",
                    ],
                ),
                (
                    "WINGS AND ROCKETS",
                    [
                        "Fixed wings generate lift perpendicular to their flight path. Lift grows with airflow, weakens below stall speed, and vanishes in a vertical fall.",
                        "An engine-off wing therefore glides and loses altitude more slowly while moving forward, then sinks faster as drag removes airspeed.",
                        "Rockets use one automatic finite main burn plus a separate limited RCS budget; burnout is permanent.",
                        "They receive gravity and drag but no wing lift, reverse thrust, throttle, restart, or airbrake.",
                        "The model is deliberately simplified and exposes its stall speed and lift efficiency rather than claiming CFD fidelity.",
                    ],
                ),
                (
                    "LIVE FORCE READOUT",
                    [
                        f"OUR {sim.interceptor.spec.code}: engine {'ON' if sim.interceptor.engine_enabled else 'CUT'}, output {sim.interceptor.engine_output*100:.0f}%, lift {sim.interceptor.lift_acceleration.length():.2f}, drag {sim.interceptor.drag_acceleration.length():.2f} m/s2.",
                        f"TARGET {sim.target.spec.code}: engine {'ON' if sim.target.engine_enabled else 'CUT'}, output {sim.target.engine_output*100:.0f}%, lift {sim.target.lift_acceleration.length():.2f}, drag {sim.target.drag_acceleration.length():.2f} m/s2.",
                        f"OUR stall/lift factor: {sim.interceptor.spec.stall_speed:.1f} m/s / {sim.interceptor.spec.lift_efficiency:.2f}.",
                        f"TARGET stall/lift factor: {sim.target.spec.stall_speed:.1f} m/s / {sim.target.spec.lift_efficiency:.2f}.",
                    ],
                ),
                (
                    "PLAYER AUTHORITY",
                    [
                        "Tab cycles AUTO -> OUR VEHICLE -> TARGET -> AUTO.",
                        "W/S request forward/reverse or deceleration; A/D turn; Q/E change and then hold altitude; Shift requests full authority; Ctrl uses an available airbrake.",
                        "Commands still pass through thrust direction, stall/lift, turn rate, speed, drag, gravity, and the two-metre floor.",
                        "When our drone is manual, the oval solution remains visible only as an advisory.",
                        "TRICKY AI assumes the target knows our true approach, then chooses deterministic jinks, traps, climbs, and sprints within that target's physical limits.",
                    ],
                ),
            )
        else:
            sections = (
                (
                    "WHEN VISUAL LOCK IS LOST",
                    [
                        "Range, target track, guidance, and ovals become invalid immediately.",
                        "The last filtered image-bearing motion is extrapolated for at most 1.5 seconds.",
                        "After that, the camera widens a horizontal scan while staying within +/-35 degrees of the world horizon.",
                        "Autonomous body search turns horizontally, holds altitude, and regulates about half of the selected vehicle's maximum speed.",
                        "Only a new detector result can reacquire; target truth is never used to point the search.",
                    ],
                ),
                (
                    "CAMERA AND SPECTATOR MODES",
                    [
                        "V cycles onboard sensor, chase, and spectator/tactical views. The fixed boresight appears only onboard.",
                        "Aegis-Q4 and Smart Evader publish a fixed 6-degree upward mount and 24-degree body limit: no target-following gimbal exists.",
                        "F3 jumps directly to the spectator view, which watches both vehicles and is not attached to either pilot.",
                        "Right mouse captures unlimited presentation free-look; Shift+right mouse rolls; the wheel zooms; C centers and resets zoom.",
                        "Presentation free-look and zoom never change the 90-degree sensor, oval plane, detection, or guidance calculation.",
                    ],
                ),
                (
                    "PROOF BOUNDARY",
                    [
                        "Defense guidance never reads target truth; truth only renders the synthetic image, resolves contact, and supports verification.",
                        "Multi-contact pairing uses same-horizon image ellipses, mutual-center containment, a reachable shared centroid, and sticky nested-oval entry.",
                        "TRICKY AI is the declared exception: only its target autopilot knows our approach; it cannot feed our sensor or guidance.",
                        "The detector/pose output, signal lookup, meshes, and aerodynamics are deterministic presentation adapters, not trained/deployed hardware.",
                        "CHECK 2s records an old camera frame and evaluates truth at T+2.",
                    ],
                ),
                (
                    "FAST DEMO KEYS",
                    [
                        "F1 info | H controls | F2 analysis | F3 spectator | F4 settings | F5 CSV",
                        "M minimap | G check 2s | O camera occlusion | Space pause | +/- time",
                        "Tab authority | X drone engine | W/S A/D Q/E flight | Shift full | Ctrl brake",
                        f"Current status: {sim.status}",
                    ],
                ),
            )

        column_width = 412
        column_x = (card.x + 28, card.x + 492)
        for section_index, (heading, lines) in enumerate(sections):
            column = section_index // 2
            section_in_column = section_index % 2
            x = column_x[column]
            y = card.y + (96 if section_in_column == 0 else 345)
            overlay.blit(self.font_bold.render(heading, True, AMBER), (x, y))
            y += 27
            for raw_line in lines:
                words = raw_line.split()
                wrapped: list[str] = []
                current = ""
                for word in words:
                    candidate = f"{current} {word}".strip()
                    if (
                        current
                        and self.font_tiny.size(candidate)[0] > column_width
                    ):
                        wrapped.append(current)
                        current = word
                    else:
                        current = candidate
                if current:
                    wrapped.append(current)
                for line in wrapped:
                    overlay.blit(
                        self.font_tiny.render(line, True, WHITE),
                        (x, y),
                    )
                    y += 17
                y += 5

        surface.blit(overlay, (0, 0))

    def draw_help(self, surface: pygame.Surface) -> None:
        width, height = surface.get_size()
        overlay = pygame.Surface((width, height), pygame.SRCALPHA)
        overlay.fill((2, 8, 14, 230))
        card = pygame.Rect(width // 2 - 470, height // 2 - 300, 940, 600)
        pygame.draw.rect(overlay, (7, 22, 32), card)
        pygame.draw.rect(overlay, CYAN, card, 1)
        overlay.blit(
            self.font_title.render("PRESENTATION + FLIGHT CONTROLS", True, WHITE),
            (card.x + 28, card.y + 24),
        )
        presentation_items = [
            ("SPACE", "pause / continue the 60 Hz simulation"),
            ("+ / -", "increase / decrease time scale"),
            ("V", "cycle onboard, chase, and spectator cameras"),
            ("RMB HOLD", "capture mouse for unlimited free look"),
            ("SHIFT+RMB", "roll the presentation camera"),
            ("WHEEL", "zoom presentation view without changing the sensor"),
            ("C", "center free look and reset presentation zoom"),
            ("F1", "open the full four-page system reference"),
            ("F2", "open range-error and resolution analysis"),
            ("F3", "jump directly to spectator / tactical view"),
            ("F4", "toggle presentation settings"),
            ("F5", "export verification telemetry to CSV"),
            ("M / G", "toggle minimap / record a +2 s prediction check"),
            ("R", "restart the current simulation"),
            ("N", "return to setup and place new drones"),
            ("O", "force camera occlusion (lock-loss proof)"),
            ("H", "close this help panel"),
            ("ESC", "close an overlay, then exit"),
        ]
        flight_items = [
            ("TAB", "AUTO -> OUR VEHICLE -> TARGET -> AUTO"),
            ("W / S", "forward thrust / reverse or decelerate"),
            ("A / D", "turn left / right within vehicle limits"),
            ("Q / E", "descend / climb; release to hold altitude"),
            ("SHIFT", "request full available maneuver authority"),
            ("CTRL", "airbrake (unavailable on rockets)"),
            ("X", "cut / restart a drone engine; solid rockets are fixed-burn"),
            ("RMB", "mouse look suppresses flight-key input"),
            ("GREEN", "direct authority"),
            ("AMBER", "turn / braking limited"),
            ("RED", "requested action unavailable"),
        ]
        overlay.blit(
            self.font_bold.render("CAMERA / DEMO", True, AMBER),
            (card.x + 36, card.y + 76),
        )
        overlay.blit(
            self.font_bold.render("PLAYER AUTHORITY", True, AMBER),
            (card.x + 490, card.y + 76),
        )
        y = card.y + 108
        for key, description in presentation_items:
            overlay.blit(self.font_bold.render(key, True, CYAN), (card.x + 36, y))
            overlay.blit(self.font_tiny.render(description, True, WHITE), (card.x + 138, y + 2))
            y += 22
        y = card.y + 108
        for key, description in flight_items:
            color = GREEN if key == "GREEN" else AMBER if key == "AMBER" else RED if key == "RED" else CYAN
            overlay.blit(self.font_bold.render(key, True, color), (card.x + 490, y))
            overlay.blit(self.font_tiny.render(description, True, WHITE), (card.x + 586, y + 2))
            y += 30

        separator_y = card.bottom - 88
        pygame.draw.line(
            overlay,
            (42, 92, 103),
            (card.x + 28, separator_y),
            (card.right - 28, separator_y),
            1,
        )
        notes = [
            "Every oval shares the target plane perpendicular to the sensor.",
            "Its border contains the modeled projected maneuvers plus bounded track uncertainty.",
            "Green/red segments show each edge result; only a fully green 96/96 oval is selectable.",
            "True range is simulation verification only, never guidance input.",
        ]
        y = separator_y + 12
        for note in notes:
            overlay.blit(
                self.font_tiny.render(note, True, MUTED),
                (card.x + 36, y),
            )
            y += 19
        surface.blit(overlay, (0, 0))
