"""Pygame software 3D renderer, HUD, and engineering-analysis overlay."""

from __future__ import annotations

from dataclasses import dataclass
import math

import pygame

from .camera import minimum_horizontal_resolution, model_vertices
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
PANEL = (5, 16, 25, 208)


@dataclass(slots=True)
class ViewCamera:
    position: Vec3
    forward: Vec3
    name: str


class WorldRenderer:
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
        self.view_mode = 0
        self.interceptor_trail: list[Vec3] = []
        self.target_trail: list[Vec3] = []
        self._trail_timer = 0.0

    def cycle_view(self) -> None:
        self.view_mode = (self.view_mode + 1) % 3

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
        self.interceptor_trail = self.interceptor_trail[-180:]
        self.target_trail = self.target_trail[-180:]

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
        if self.view_mode == 0:
            return ViewCamera(sim.interceptor.position, sim.camera_forward, "ONBOARD / GIMBAL")
        if self.view_mode == 1:
            position = sim.interceptor.position - line * 14.0 + Vec3(0, 6.5, 0)
            focus = sim.interceptor.position + line * min(45.0, sim.true_range_m * 0.55)
            return ViewCamera(position, (focus - position).normalized(line), "CHASE")
        midpoint = (sim.interceptor.position + sim.target.position) * 0.5
        separation = clamp(sim.true_range_m, 35.0, 280.0)
        position = midpoint + Vec3(-separation * 0.54, separation * 0.40, -separation * 0.58)
        return ViewCamera(
            position,
            (midpoint - position).normalized(line),
            "TACTICAL OVERVIEW",
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
        if relative.z <= 0.08:
            return None
        focal = width / (2.0 * math.tan(math.radians(fov_deg) * 0.5))
        x = int(width * 0.5 + focal * relative.x / relative.z)
        y = int(height * 0.5 - focal * relative.y / relative.z)
        return x, y, relative.z

    def _draw_ground_grid(
        self, surface: pygame.Surface, camera: ViewCamera, usable_height: int
    ) -> None:
        width, height = surface.get_size()
        grid_color = (48, 90, 83)
        major_color = (55, 111, 100)
        for x in range(-400, 401, 25):
            points = []
            for z in range(-100, 901, 20):
                projected = self._project(Vec3(x, 0, z), camera, width, height)
                if projected and -100 < projected[0] < width + 100 and projected[1] < usable_height:
                    points.append(projected[:2])
            if len(points) >= 2:
                pygame.draw.lines(
                    surface, major_color if x == 0 else grid_color, False, points, 1
                )
        for z in range(-100, 901, 25):
            points = []
            for x in range(-400, 401, 20):
                projected = self._project(Vec3(x, 0, z), camera, width, height)
                if projected and -100 < projected[0] < width + 100 and projected[1] < usable_height:
                    points.append(projected[:2])
            if len(points) >= 2:
                pygame.draw.lines(
                    surface, major_color if z == 0 else grid_color, False, points, 1
                )

    def _draw_polyline_3d(
        self,
        surface: pygame.Surface,
        points: list[Vec3],
        camera: ViewCamera,
        color: tuple[int, int, int],
        width_px: int = 1,
    ) -> None:
        width, height = surface.get_size()
        segment: list[tuple[int, int]] = []
        for point in points:
            projected = self._project(point, camera, width, height)
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
        projected = [self._project(vertex, camera, width, height) for vertex in vertices]
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
        points: list[Vec3] = []
        for index in range(count + 1):
            angle = math.tau * index / count
            cosine, sine = math.cos(angle), math.sin(angle)
            x_vector = (
                oval.extremes[0] - oval.center
                if cosine >= 0
                else oval.center - oval.extremes[1]
            )
            y_vector = (
                oval.extremes[2] - oval.center
                if sine >= 0
                else oval.center - oval.extremes[3]
            )
            points.append(oval.center + x_vector * cosine + y_vector * sine)
        return points

    def _draw_predictions(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
        camera: ViewCamera,
    ) -> None:
        if not sim.guidance or sim.hit:
            return
        colors = ((54, 117, 133), (54, 139, 152), (58, 169, 180), CYAN)
        selected_horizon = sim.guidance.selected_horizon_s
        for oval, color in zip(sim.guidance.ovals, colors):
            selected = (
                selected_horizon is not None
                and abs(oval.horizon_s - selected_horizon) < 0.02
                and sim.guidance.mode == "OVAL CENTER"
            )
            draw_color = WHITE if selected else color
            self._draw_polyline_3d(
                surface,
                self._oval_points(oval),
                camera,
                draw_color,
                2 if selected else 1,
            )
            for point, reachable in zip(oval.extremes, oval.reachable):
                projected = self._project(point, camera, *surface.get_size())
                if projected:
                    pygame.draw.circle(
                        surface, GREEN if reachable else RED, projected[:2], 3, 1
                    )
            center_projected = self._project(oval.center, camera, *surface.get_size())
            if center_projected:
                text = self.font_tiny.render(f"+{oval.horizon_s:.0f}s", True, draw_color)
                surface.blit(text, (center_projected[0] + 5, center_projected[1] + 2))

        ballistic_points = [
            sim.track.position + sim.track.velocity * (step * 0.5)
            for step in range(1, 11)
        ] if sim.track.position else []
        for point in ballistic_points:
            projected = self._project(point, camera, *surface.get_size())
            if projected:
                pygame.draw.circle(surface, AMBER, projected[:2], 2)

        aim = self._project(sim.guidance.aim_point, camera, *surface.get_size())
        if aim:
            x, y = aim[:2]
            pygame.draw.line(surface, WHITE, (x - 8, y), (x + 8, y), 1)
            pygame.draw.line(surface, WHITE, (x, y - 8), (x, y + 8), 1)
            surface.blit(self.font_tiny.render("AIM", True, WHITE), (x + 10, y - 7))

    def _draw_detection_brackets(
        self,
        surface: pygame.Surface,
        bounds: tuple[int, int, int, int] | None,
        sim: InterceptionSimulation,
    ) -> None:
        if not bounds or self.view_mode != 0:
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
            projected = self._project(particle, camera, *surface.get_size())
            if projected:
                fade = clamp(1.0 - age / 2.5, 0.0, 1.0)
                color = (
                    int(255 * fade),
                    int((80 + (index % 4) * 35) * fade),
                    int(35 * fade),
                )
                pygame.draw.circle(surface, color, projected[:2], max(1, int(4 * fade)))

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

        drone_draws = []
        if self.view_mode != 0:
            drone_draws.append(
                (
                    camera_coordinates(
                        sim.interceptor.position, camera.position, camera.forward
                    ).z,
                    sim.interceptor,
                    f"OUR // {sim.interceptor.spec.code}",
                    False,
                )
            )
        drone_draws.append(
            (
                camera_coordinates(sim.target.position, camera.position, camera.forward).z,
                sim.target,
                "TARGET",
                True,
            )
        )
        target_bounds = None
        for _, state, label, highlighted in sorted(drone_draws, reverse=True, key=lambda item: item[0]):
            bounds = self._draw_drone(surface, state, camera, label, highlighted)
            if state is sim.target:
                target_bounds = bounds

        self._draw_explosion(surface, sim, camera)
        self._draw_detection_brackets(surface, target_bounds, sim)
        self._draw_reticle(surface)
        return camera

    def _draw_reticle(self, surface: pygame.Surface) -> None:
        width, height = surface.get_size()
        center = (width // 2, (height - 180) // 2)
        color = (109, 162, 171)
        pygame.draw.circle(surface, color, center, 18, 1)
        pygame.draw.line(surface, color, (center[0] - 31, center[1]), (center[0] - 10, center[1]), 1)
        pygame.draw.line(surface, color, (center[0] + 10, center[1]), (center[0] + 31, center[1]), 1)
        pygame.draw.line(surface, color, (center[0], center[1] - 31), (center[0], center[1] - 10), 1)
        pygame.draw.line(surface, color, (center[0], center[1] + 10), (center[0], center[1] + 31), 1)

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
            panel.blit(self.font_tiny.render(label, True, MUTED), (12, y))
            rendered = self.font_small.render(value, True, color or WHITE)
            panel.blit(rendered, (rect.width - 12 - rendered.get_width(), y - 1))
            y += 18
            if y > rect.height - 15:
                break
        surface.blit(panel, rect)

    def draw_hud(
        self,
        surface: pygame.Surface,
        sim: InterceptionSimulation,
        time_scale: float,
        fps: float,
    ) -> None:
        width, height = surface.get_size()
        top = pygame.Surface((width, 54), pygame.SRCALPHA)
        top.fill((3, 12, 20, 205))
        top.blit(self.font_title.render("ZENITH", True, WHITE), (18, 9))
        top.blit(
            self.font_small.render("VISION-ONLY INTERCEPTION // PROTOTYPE 01", True, CYAN),
            (132, 19),
        )
        status_color = GREEN if sim.hit else (CYAN if sim.identity_confirmed else AMBER)
        status_text = self.font_bold.render(sim.status, True, status_color)
        top.blit(status_text, (width - status_text.get_width() - 20, 10))
        details = self.font_tiny.render(
            f"T+ {sim.time_s:06.2f}s   TIME x{time_scale:g}   {fps:4.0f} FPS   SENSOR {sim.config.camera.width_px}x{sim.config.camera.height_px}",
            True,
            MUTED,
        )
        top.blit(details, (width - details.get_width() - 20, 33))
        surface.blit(top, (0, 0))

        surface.blit(
            self.font_tiny.render(
                f"VIEW: {self.get_view_camera(sim).name}   [V] SWITCH   [SPACE] PAUSE   [A] ANALYSIS   [H] HELP",
                True,
                MUTED,
            ),
            (16, 62),
        )
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

        panel_y = height - 178
        margin, gap = 10, 8
        panel_width = (width - margin * 2 - gap * 3) // 4
        panel_height = 168

        target_name = sim.target.spec.name if sim.identity_confirmed else "???"
        target_axial_accel = (
            sim.track.acceleration.dot(sim.track.velocity.normalized())
            if sim.track.velocity.length() > 0.2
            else 0.0
        )
        target_rows = [
            ("MODEL", target_name, sim.target.spec.color if sim.identity_confirmed else AMBER),
            ("MAX SPEED", f"{sim.target.spec.max_speed:.1f} m/s" if sim.identity_confirmed else "QUERYING"),
            ("ACCEL / BRAKE", f"{sim.target.spec.max_accel:.1f} / {sim.target.spec.brake_accel:.1f} m/s²" if sim.identity_confirmed else "QUERYING"),
            ("SIZE W/H/L", sim.target.spec.size_label if sim.identity_confirmed else "QUERYING"),
            ("BOUND. VOLUME", f"{sim.target.spec.bounding_volume:.4f} m³" if sim.identity_confirmed else "QUERYING"),
            ("SPEED / AXIAL A", f"{sim.track.velocity.length():.2f} / {target_axial_accel:+.2f}" if sim.track.sample_count > 2 else "ANGULAR ONLY"),
            ("SCENARIO", dict(SCENARIOS)[sim.config.scenario]),
        ]

        estimate = sim.range_estimate
        if sim.track.position:
            relative_camera = camera_coordinates(
                sim.track.position, sim.interceptor.position, sim.camera_forward
            )
        else:
            relative_camera = Vec3()
        calc_rows = [
            ("FORMULA", "Z = f × S / p", CYAN),
            ("FOCAL LENGTH", f"{sim.config.camera.focal_px:.1f} px"),
            ("LAST APPARENT / CONF" if sim.hit else "APPARENT / CONF", f"{max(sim.detection.width_px, sim.detection.height_px):.2f} px / {sim.detection.confidence*100:.0f}%"),
            ("RANGE AT IMPACT" if sim.hit else "RANGE EST.", f"{estimate.distance_m:.2f} ± {estimate.sigma_m:.2f} m" if estimate else "WAITING FOR SIZE"),
            ("LAST Dx  LEFT/RIGHT" if sim.hit else "Dx  LEFT/RIGHT", f"{relative_camera.x:+.2f} m" if estimate else "---"),
            ("LAST Dy  UP/DOWN" if sim.hit else "Dy  UP/DOWN", f"{relative_camera.y:+.2f} m" if estimate else "---"),
            ("LAST Dz  LINE OF SIGHT" if sim.hit else "Dz  LINE OF SIGHT", f"{relative_camera.z:+.2f} m" if estimate else "---"),
        ]

        own = sim.interceptor
        own_rows = [
            ("MODEL", own.spec.name, own.spec.color),
            ("POSITION XYZ", f"{own.position.x:+.0f}/{own.position.y:+.0f}/{own.position.z:+.0f} m"),
            ("Vx", f"{own.velocity.x:+.2f} m/s"),
            ("Vy", f"{own.velocity.y:+.2f} m/s"),
            ("Vz", f"{own.velocity.z:+.2f} m/s"),
            ("TOTAL SPEED", f"{own.velocity.length():.2f} / {own.spec.max_speed:.0f} m/s"),
            ("ACCELERATION", f"{own.acceleration.length():.2f} m/s²"),
        ]

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
        error = (
            estimate.distance_m - sim.true_range_m if estimate is not None else None
        )
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
            relative_rows = [
                ("REL. Vx / Vy", f"{relative_v.x:+.2f} / {relative_v.y:+.2f} m/s"),
                ("REL. Vz", f"{relative_v.z:+.2f} m/s"),
                ("CLOSING SPEED", f"{guidance.closing_speed:+.2f} m/s" if guidance else "---"),
                ("TIME TO CONTACT", f"{guidance.time_to_contact_s:.2f} s" if guidance and math.isfinite(guidance.time_to_contact_s) else "---"),
                ("GUIDANCE", guidance.mode if guidance else "BEARING PURSUIT"),
                ("OVAL / EDGES", f"{guidance.selected_horizon_s:.1f}s / {guidance.reachable_count}/4" if guidance and guidance.selected_horizon_s else "---"),
                ("VERIFY TRUE / ERR", f"{sim.true_range_m:.2f} / {error:+.2f} m" if error is not None else f"{sim.true_range_m:.2f} / ---", MUTED),
            ]

        panels = (
            ("TARGET VEHICLE", target_rows),
            ("CALCULATIONS", calc_rows),
            ("OUR DRONE", own_rows),
            ("RELATIVE", relative_rows),
        )
        for index, (title, rows) in enumerate(panels):
            rect = pygame.Rect(
                margin + index * (panel_width + gap),
                panel_y,
                panel_width,
                panel_height,
            )
            self._panel(surface, rect, title, rows)

    def draw_analysis(self, surface: pygame.Surface, sim: InterceptionSimulation) -> None:
        width, height = surface.get_size()
        overlay = pygame.Surface((width, height), pygame.SRCALPHA)
        overlay.fill((2, 8, 14, 238))
        margin = 42
        pygame.draw.rect(overlay, (59, 136, 151), (margin, margin, width - margin * 2, height - margin * 2), 1)
        overlay.blit(self.font_title.render("ENGINEERING ANALYSIS // MONOCULAR RANGE", True, WHITE), (margin + 24, margin + 18))
        overlay.blit(self.font_small.render("[A] close analysis", True, MUTED), (width - margin - 160, margin + 25))
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
        pixels = list(range(2, 201, 2))
        ranges = [focal * spec.dimensions.x / value for value in pixels]
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
            graph_point(pixel, focal * spec.dimensions.x / max(0.5, pixel - 0.5))
            for pixel in pixels
        ]
        lower = [
            graph_point(pixel, focal * spec.dimensions.x / (pixel + 0.5))
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
        explanation = [
            "A ±0.5 px box-edge error produces approximately:",
            "    relative range error ≈ ±0.5 / p",
            "At 4 px:  ±12.5%    At 20 px: ±2.5%",
            "At 100 px: ±0.5%",
            "",
            "Pose-compensated fitting uses the known 3D",
            "dimensions. A width-only estimate is also logged",
            "so rotation sensitivity remains verifiable.",
        ]
        y = graph.top + 30
        for line in explanation:
            overlay.blit(self.font_small.render(line, True, WHITE if line else MUTED), (right_x, y))
            y += 21

        overlay.blit(self.font_bold.render("MINIMUM HORIZONTAL CAMERA RESOLUTION", True, AMBER), (right_x, y + 12))
        y += 44
        overlay.blit(self.font_tiny.render("12 px target requirement // 75° horizontal FOV", True, MUTED), (right_x, y))
        y += 24
        for distance in (100, 250, 500, 1000):
            resolution = minimum_horizontal_resolution(
                distance, spec.dimensions.x, camera.horizontal_fov_deg, 12
            )
            color = GREEN if resolution <= 3840 else (AMBER if resolution <= 7680 else RED)
            line = f"{distance:4d} m  →  {resolution:5d} horizontal pixels"
            overlay.blit(self.font.render(line, True, color), (right_x, y))
            y += 25

        y += 18
        overlay.blit(self.font_bold.render("CURRENT FRAME VERIFICATION", True, AMBER), (right_x, y))
        y += 30
        current_lines = [
            f"Target: {spec.name} ({spec.dimensions.x:.2f} m reference width)",
            f"Sensor focal length: {focal:.2f} px",
            f"Measured bounding span: {max(sim.detection.width_px, sim.detection.height_px):.3f} px",
            f"Estimated range: {sim.range_estimate.distance_m:.3f} m" if sim.range_estimate else "Estimated range: waiting for identity",
            f"Ground-truth range: {sim.true_range_m:.3f} m [verification only]",
        ]
        for line in current_lines:
            overlay.blit(self.font_small.render(line, True, WHITE), (right_x, y))
            y += 22

        surface.blit(overlay, (0, 0))

    def draw_help(self, surface: pygame.Surface) -> None:
        width, height = surface.get_size()
        overlay = pygame.Surface((width, height), pygame.SRCALPHA)
        overlay.fill((2, 8, 14, 230))
        card = pygame.Rect(width // 2 - 330, height // 2 - 315, 660, 630)
        pygame.draw.rect(overlay, (7, 22, 32), card)
        pygame.draw.rect(overlay, CYAN, card, 1)
        overlay.blit(self.font_title.render("PRESENTATION CONTROLS", True, WHITE), (card.x + 28, card.y + 24))
        items = [
            ("SPACE", "pause / continue the 60 Hz simulation"),
            ("+ / -", "increase / decrease time scale"),
            ("V", "cycle onboard, chase, and tactical cameras"),
            ("A", "open range-error and resolution analysis"),
            ("R", "restart the current simulation"),
            ("N", "return to setup and place new drones"),
            ("E", "export verification telemetry to a CSV file"),
            ("H", "close this help panel"),
            ("ESC", "close an overlay, then exit"),
        ]
        y = card.y + 86
        for key, description in items:
            overlay.blit(self.font_bold.render(key, True, CYAN), (card.x + 36, y))
            overlay.blit(self.font.render(description, True, WHITE), (card.x + 160, y))
            y += 38
        y += 8
        notes = [
            "Green extreme point = our drone can reach it before the target.",
            "Red extreme point = unreachable. White oval = selected solution.",
            "The bottom-right true range is simulation verification, never guidance input.",
        ]
        for note in notes:
            overlay.blit(self.font_small.render(note, True, MUTED), (card.x + 36, y))
            y += 25
        surface.blit(overlay, (0, 0))
