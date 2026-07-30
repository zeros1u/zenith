"""INTERCEPTRON interactive desktop application entry point."""

from __future__ import annotations

import argparse
import ctypes
import csv
from datetime import datetime
import json
import os
from pathlib import Path

import pygame

from zenith.camera import CameraModel
from zenith.controls import ControlMode, ManualControlInput
from zenith.math3d import Vec3
from zenith.models import INTERCEPTOR_SPECS, TARGET_SPECS
from zenith.physics import DroneState
from zenith.rendering import (
    AMBER,
    BG_TOP,
    CYAN,
    GREEN,
    INFO_PAGE_COUNT,
    MUTED,
    RED,
    WHITE,
    ViewCamera,
    WorldRenderer,
)
from zenith.simulation import InterceptionSimulation, SCENARIOS, SimulationConfig


DISPLAY_OPTIONS = (
    (1050, 700),
    (1152, 720),
    (1280, 800),
)
WINDOW_SIZE = DISPLAY_OPTIONS[0]
MINIMUM_WINDOW_SIZE = (1000, 680)
FIXED_STEP = 1.0 / 60.0
TIME_SCALES = (0.25, 0.5, 1.0, 2.0, 4.0)
DEFAULT_TIME_SCALE_INDEX = 1
SENSOR_OPTIONS = (
    (1280, 720),
    (1920, 1080),
    (3840, 2160),
)


def enable_windows_dpi_awareness() -> bool:
    """Keep requested client pixels physical when Windows display scaling is >100%."""
    if os.name != "nt":
        return False
    try:
        # Per-monitor V2 must be set before pygame creates its first window.
        ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
        return True
    except (AttributeError, OSError):
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
            return True
        except (AttributeError, OSError):
            try:
                ctypes.windll.user32.SetProcessDPIAware()
                return True
            except (AttributeError, OSError):
                return False


class NumericField:
    def __init__(self, label: str, value: float) -> None:
        self.label = label
        self.text = f"{value:g}"
        self.active = False
        self.rect = pygame.Rect(0, 0, 120, 42)

    def value(self) -> float:
        return float(self.text)

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type != pygame.KEYDOWN or not self.active:
            return
        if event.key == pygame.K_BACKSPACE:
            self.text = self.text[:-1]
        elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_TAB):
            self.active = False
        elif event.unicode and event.unicode in "0123456789.-" and len(self.text) < 10:
            if event.unicode == "-" and self.text:
                return
            if event.unicode == "." and "." in self.text:
                return
            self.text += event.unicode


class SetupScreen:
    def __init__(self, size: tuple[int, int]) -> None:
        pygame.font.init()
        self.title = pygame.font.SysFont("arial", 42, bold=True)
        self.subtitle = pygame.font.SysFont("consolas", 16)
        self.font = pygame.font.SysFont("consolas", 15)
        self.bold = pygame.font.SysFont("consolas", 16, bold=True)
        self.small = pygame.font.SysFont("consolas", 13)
        self.interceptor_index = 3
        self.target_index = 0
        self.scenario_index = 2
        self.sensor_index = 1
        self.enemy_count = 1
        self.display_index = min(
            range(len(DISPLAY_OPTIONS)),
            key=lambda index: abs(DISPLAY_OPTIONS[index][0] - size[0])
            + abs(DISPLAY_OPTIONS[index][1] - size[1]),
        )
        self.display_request: tuple[int, int] | None = None
        self.own_fields = [
            NumericField("X", 0),
            NumericField("Y", 28),
            NumericField("Z", 0),
        ]
        self.target_fields = [
            NumericField("X", 10),
            NumericField("Y", 36),
            NumericField("Z", 240),
        ]
        self.buttons: dict[str, pygame.Rect] = {}
        self.error = ""
        self.size = size
        self.preview_renderer = WorldRenderer()

    def _button(
        self,
        surface: pygame.Surface,
        key: str,
        rect: pygame.Rect,
        text: str,
        selected: bool = False,
        accent: tuple[int, int, int] = CYAN,
    ) -> None:
        self.buttons[key] = rect
        color = accent if selected else (54, 85, 97)
        fill = (14, 38, 49) if selected else (9, 27, 37)
        pygame.draw.rect(surface, fill, rect)
        pygame.draw.rect(surface, color, rect, 2 if selected else 1)
        font = self.bold
        if font.size(text)[0] > rect.width - 12:
            font = self.small
        rendered = font.render(text, True, accent if selected else WHITE)
        surface.blit(
            rendered,
            (
                rect.centerx - rendered.get_width() // 2,
                rect.centery - rendered.get_height() // 2,
            ),
        )

    def _draw_model_card(
        self,
        surface: pygame.Surface,
        rect: pygame.Rect,
        heading: str,
        spec_index: int,
        prefix: str,
    ) -> None:
        catalogue = INTERCEPTOR_SPECS if prefix == "own" else TARGET_SPECS
        spec = catalogue[spec_index]
        pygame.draw.rect(surface, (7, 24, 35), rect)
        pygame.draw.rect(surface, spec.color, rect, 1)
        surface.blit(self.small.render(heading, True, MUTED), (rect.x + 18, rect.y + 14))
        surface.blit(self.bold.render(spec.name, True, spec.color), (rect.x + 18, rect.y + 38))
        surface.blit(self.small.render(spec.notes, True, WHITE), (rect.x + 18, rect.y + 65))
        values = [
            f"SIZE       {spec.size_label}",
            f"MAX SPEED  {spec.max_speed:.0f} m/s",
            f"ACCEL      {spec.max_accel:.0f} m/s²",
            (
                f"BOOST/RCS  {spec.main_burn_duration_s:.0f}s / "
                f"{spec.rcs_duration_s:.0f}s"
                if spec.flight_model == "rocket"
                else f"AIRBRAKE   {spec.brake_accel:.0f} m/s²"
            ),
        ]
        y = rect.y + 94
        for line in values:
            surface.blit(self.font.render(line, True, WHITE), (rect.x + 18, y))
            y += 24

        preview = pygame.Surface((210, 128), pygame.SRCALPHA)
        pygame.draw.circle(preview, (20, 53, 65, 180), (105, 65), 58, 1)
        preview_distance = max(spec.dimensions.as_tuple()) * 1.55
        preview_state = DroneState(
            spec,
            Vec3(0, 0, preview_distance),
            Vec3(),
            orientation=Vec3(0.18, 0.72, -0.08),
        )
        self.preview_renderer._draw_drone(
            preview,
            preview_state,
            ViewCamera(Vec3(), Vec3(0, 0, 1), "MODEL PREVIEW"),
            "",
            False,
        )
        surface.blit(preview, (rect.right - 260, rect.y + 56))
        self._button(
            surface,
            f"{prefix}_prev",
            pygame.Rect(rect.right - 90, rect.y + 18, 31, 31),
            "<",
        )
        self._button(
            surface,
            f"{prefix}_next",
            pygame.Rect(rect.right - 49, rect.y + 18, 31, 31),
            ">",
        )

    def _draw_fields(
        self,
        surface: pygame.Surface,
        fields: list[NumericField],
        x: int,
        y: int,
        heading: str,
    ) -> None:
        surface.blit(self.small.render(heading, True, MUTED), (x, y))
        y += 22
        for index, field in enumerate(fields):
            field.rect = pygame.Rect(x + index * 139, y, 124, 43)
            pygame.draw.rect(surface, (5, 18, 27), field.rect)
            pygame.draw.rect(surface, CYAN if field.active else (49, 88, 100), field.rect, 2 if field.active else 1)
            surface.blit(self.small.render(field.label, True, MUTED), (field.rect.x + 8, field.rect.y + 4))
            value = self.bold.render(field.text or "_", True, WHITE)
            surface.blit(value, (field.rect.x + 30, field.rect.y + 17))
            unit = self.small.render("m", True, MUTED)
            surface.blit(unit, (field.rect.right - 20, field.rect.y + 21))

    def draw(self, surface: pygame.Surface) -> None:
        self.size = surface.get_size()
        width, height = self.size
        surface.fill(BG_TOP)
        for y in range(height):
            amount = y / max(1, height)
            color = (7, int(17 + amount * 14), int(31 + amount * 21))
            pygame.draw.line(surface, color, (0, y), (width, y))
        pygame.draw.circle(surface, (19, 64, 76), (width - 120, 80), 160, 1)
        pygame.draw.circle(surface, (17, 52, 63), (width - 120, 80), 110, 1)

        surface.blit(self.title.render("INTERCEPTRON", True, WHITE), (48, 30))
        display_x = width - 424
        surface.blit(
            self.small.render("WINDOW SIZE // CAMERA SENSOR IS SEPARATE", True, MUTED),
            (display_x, 20),
        )
        for index, resolution in enumerate(DISPLAY_OPTIONS):
            self._button(
                surface,
                f"display_{index}",
                pygame.Rect(display_x + index * 126, 43, 116, 34),
                f"{resolution[0]} × {resolution[1]}",
                index == self.display_index,
            )
        surface.blit(
            self.subtitle.render(
                "VISION-ONLY INTERCEPTION // SIMULATION CONFIGURATION",
                True,
                CYAN,
            ),
            (50, 82),
        )
        pipeline = "SYNTHETIC DETECTOR [YOLO/DINO ADAPTER INTERFACE]  >  SIGNAL LOOKUP  >  PINHOLE RANGE  >  OVALS  >  MANEUVER"
        surface.blit(self.small.render(pipeline, True, MUTED), (50, 113))

        content_width = min(1260, width - 90)
        left = (width - content_width) // 2
        card_gap = 22
        card_width = (content_width - card_gap) // 2
        card_y = 142
        card_height = 190
        self._draw_model_card(
            surface,
            pygame.Rect(left, card_y, card_width, card_height),
            "OUR VEHICLE / INTERCEPTOR",
            self.interceptor_index,
            "own",
        )
        self._draw_model_card(
            surface,
            pygame.Rect(left + card_width + card_gap, card_y, card_width, card_height),
            "TARGET VEHICLE / INTRUDER",
            self.target_index,
            "target",
        )

        fields_y = card_y + card_height + 21
        self._draw_fields(surface, self.own_fields, left + 4, fields_y, "INTERCEPTOR START COORDINATES // WORLD X Y Z")
        self._draw_fields(
            surface,
            self.target_fields,
            left + card_width + card_gap + 4,
            fields_y,
            "TARGET START COORDINATES // WORLD X Y Z",
        )

        scenario_y = fields_y + 92
        surface.blit(self.small.render("TARGET BEHAVIOR", True, MUTED), (left + 4, scenario_y))
        button_y = scenario_y + 24
        scenario_count = len(SCENARIOS)
        scenario_width = (content_width - (scenario_count - 1) * 10) // scenario_count
        for index, (_, name) in enumerate(SCENARIOS):
            self._button(
                surface,
                f"scenario_{index}",
                pygame.Rect(left + index * (scenario_width + 10), button_y, scenario_width, 42),
                name,
                index == self.scenario_index,
                TARGET_SPECS[self.target_index].color,
            )

        options_y = button_y + 68
        surface.blit(self.small.render("MONOCULAR SENSOR RESOLUTION // 90 DEG HFOV", True, MUTED), (left + 4, options_y))
        for index, resolution in enumerate(SENSOR_OPTIONS):
            self._button(
                surface,
                f"sensor_{index}",
                pygame.Rect(left + index * 166, options_y + 23, 154, 38),
                f"{resolution[0]} × {resolution[1]}",
                index == self.sensor_index,
            )

        contacts_x = left + 500
        surface.blit(
            self.small.render("ENEMY CONTACTS", True, MUTED),
            (contacts_x, options_y),
        )
        for count in (1, 2, 3):
            self._button(
                surface,
                f"enemy_count_{count}",
                pygame.Rect(
                    contacts_x + (count - 1) * 47,
                    options_y + 23,
                    40,
                    38,
                ),
                str(count),
                count == self.enemy_count,
                TARGET_SPECS[self.target_index].color,
            )

        launch_rect = pygame.Rect(left + content_width - 310, options_y + 16, 310, 54)
        self._button(surface, "launch", launch_rect, "START 60 Hz SIMULATION", True, GREEN)
        if self.error:
            error_text = self.font.render(self.error, True, RED)
            surface.blit(error_text, (launch_rect.right - error_text.get_width(), launch_rect.bottom + 8))

        footer_y = height - 38
        pygame.draw.line(surface, (39, 76, 89), (left, footer_y - 10), (left + content_width, footer_y - 10))
        footer = (
            "Six drones + two rockets selectable on both sides • place both vehicles • no radar/lidar/rangefinder • "
            "ground truth is verification-only"
        )
        surface.blit(self.small.render(footer, True, MUTED), (left, footer_y))

    def _all_fields(self) -> list[NumericField]:
        return self.own_fields + self.target_fields

    def handle_event(self, event: pygame.event.Event) -> SimulationConfig | None:
        for field in self._all_fields():
            field.handle_event(event)
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return None

        for field in self._all_fields():
            field.active = field.rect.collidepoint(event.pos)
        for key, rect in self.buttons.items():
            if not rect.collidepoint(event.pos):
                continue
            if key == "own_prev":
                self.interceptor_index = (
                    self.interceptor_index - 1
                ) % len(INTERCEPTOR_SPECS)
            elif key == "own_next":
                self.interceptor_index = (
                    self.interceptor_index + 1
                ) % len(INTERCEPTOR_SPECS)
            elif key == "target_prev":
                self.target_index = (self.target_index - 1) % len(TARGET_SPECS)
                self._sync_target_scenario()
            elif key == "target_next":
                self.target_index = (self.target_index + 1) % len(TARGET_SPECS)
                self._sync_target_scenario()
            elif key.startswith("scenario_"):
                self.scenario_index = int(key.split("_")[1])
            elif key.startswith("sensor_"):
                self.sensor_index = int(key.split("_")[1])
            elif key.startswith("enemy_count_"):
                self.enemy_count = int(key.rsplit("_", 1)[1])
            elif key.startswith("display_"):
                self.display_index = int(key.split("_")[1])
                self.display_request = DISPLAY_OPTIONS[self.display_index]
            elif key == "launch":
                return self.build_config()
        return None

    def consume_display_request(self) -> tuple[int, int] | None:
        request = self.display_request
        self.display_request = None
        return request

    def _sync_target_scenario(self) -> None:
        is_rocket = TARGET_SPECS[self.target_index].vehicle_type == "rocket"
        rocket_index = next(
            index for index, (key, _) in enumerate(SCENARIOS) if key == "rocket_attack"
        )
        if is_rocket:
            self.scenario_index = rocket_index
        elif self.scenario_index == rocket_index:
            self.scenario_index = 2

    def build_config(self) -> SimulationConfig | None:
        try:
            own = Vec3(*(field.value() for field in self.own_fields))
            target = Vec3(*(field.value() for field in self.target_fields))
        except ValueError:
            self.error = "Coordinates must be valid numbers."
            return None
        if own.distance_to(target) < 8.0:
            self.error = "Place the drones at least 8 m apart."
            return None
        if own.y < 2.0 or target.y < 2.0:
            self.error = "Both drones must start at least 2 m above ground."
            return None
        resolution = SENSOR_OPTIONS[self.sensor_index]
        enemy_count = (
            1
            if TARGET_SPECS[self.target_index].vehicle_type == "rocket"
            else self.enemy_count
        )
        self.error = ""
        return SimulationConfig(
            interceptor_code=INTERCEPTOR_SPECS[self.interceptor_index].code,
            target_code=TARGET_SPECS[self.target_index].code,
            interceptor_position=own,
            target_position=target,
            scenario=SCENARIOS[self.scenario_index][0],
            camera=CameraModel(resolution[0], resolution[1], 90.0),
            enemy_count=enemy_count,
        )


def run_headless(args: argparse.Namespace) -> int:
    target_code = {
        "rocket_attack": "SR1",
        "tricky": "SEV",
    }.get(args.scenario, "FX1")
    config = SimulationConfig(scenario=args.scenario, target_code=target_code)
    simulation = InterceptionSimulation(config)
    steps = args.headless_steps if args.headless_steps is not None else 600
    for _ in range(steps):
        simulation.step(FIXED_STEP)
        if simulation.finished:
            break

    if args.screenshot:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        pygame.init()
        surface = pygame.Surface(WINDOW_SIZE)
        renderer = WorldRenderer()
        if simulation.hit:
            renderer.view_mode = 1
        elif args.scenario == "rocket_attack":
            renderer.view_mode = 1
        renderer.draw_world(surface, simulation)
        renderer.draw_hud(surface, simulation, 1.0, 60.0)
        output = Path(args.screenshot).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        pygame.image.save(surface, output)

    result = {
        "scenario": args.scenario,
        "steps": steps,
        "simulation_time_s": round(simulation.time_s, 3),
        "identity_confirmed": simulation.identity_confirmed,
        "hit": simulation.hit,
        "hit_time_s": round(simulation.hit_time_s, 3) if simulation.hit_time_s else None,
        "final_true_range_m": round(simulation.true_range_m, 3),
        "telemetry_samples": len(simulation.telemetry),
        "screenshot": str(Path(args.screenshot).resolve()) if args.screenshot else None,
    }
    print(json.dumps(result, indent=2))
    return 0 if simulation.identity_confirmed else 1


def export_telemetry(simulation: InterceptionSimulation) -> Path:
    output_dir = Path("exports")
    output_dir.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = output_dir / f"interceptron_{simulation.config.scenario}_{stamp}.csv"
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_s",
                "true_range_m_verification_only",
                "camera_estimated_range_m",
                "apparent_target_span_px",
                "range_error_m",
                "closing_speed_m_s",
                "estimated_target_x_m",
                "estimated_target_y_m",
                "estimated_target_z_m",
                "guidance_mode",
                "selected_horizon_s",
                "edge_reachable",
                "edge_total",
                "interceptor_booster_remaining_s",
                "interceptor_rcs_remaining_s",
                "target_booster_remaining_s",
                "target_rcs_remaining_s",
                "active_contact_id",
                "visible_contacts",
                "total_contacts",
                "priority_score_camera_tracks",
                "target_type",
                "target_model",
                "multi_guidance_mode",
                "shared_pair",
                "shared_horizon_s",
                "interceptor_model",
                "scenario",
                "target_ai_state",
                "target_ai_threat_range_m_truth_assumption",
                "target_ai_closing_speed_m_s_truth_assumption",
            ]
        )
        for sample in simulation.telemetry:
            writer.writerow(
                [
                    f"{sample.time_s:.6f}",
                    f"{sample.true_range_m:.6f}",
                    "" if sample.estimated_range_m is None else f"{sample.estimated_range_m:.6f}",
                    f"{sample.apparent_px:.6f}",
                    "" if sample.range_error_m is None else f"{sample.range_error_m:.6f}",
                    f"{sample.closing_speed:.6f}",
                    (
                        ""
                        if sample.estimated_target_position is None
                        else f"{sample.estimated_target_position.x:.6f}"
                    ),
                    (
                        ""
                        if sample.estimated_target_position is None
                        else f"{sample.estimated_target_position.y:.6f}"
                    ),
                    (
                        ""
                        if sample.estimated_target_position is None
                        else f"{sample.estimated_target_position.z:.6f}"
                    ),
                    sample.guidance_mode,
                    (
                        ""
                        if sample.selected_horizon_s is None
                        else f"{sample.selected_horizon_s:.6f}"
                    ),
                    sample.edge_reachable,
                    sample.edge_total,
                    f"{sample.interceptor_burn_remaining_s:.6f}",
                    f"{sample.interceptor_rcs_remaining_s:.6f}",
                    f"{sample.target_burn_remaining_s:.6f}",
                    f"{sample.target_rcs_remaining_s:.6f}",
                    sample.active_contact_id,
                    sample.visible_contacts,
                    sample.total_contacts,
                    f"{sample.priority_score:.6f}",
                    sample.target_type,
                    sample.target_model,
                    sample.multi_guidance_mode,
                    sample.shared_pair or "",
                    (
                        ""
                        if sample.shared_horizon_s is None
                        else f"{sample.shared_horizon_s:.6f}"
                    ),
                    simulation.interceptor.spec.name,
                    simulation.config.scenario,
                    sample.target_ai_state,
                    (
                        ""
                        if sample.target_ai_threat_range_m is None
                        else f"{sample.target_ai_threat_range_m:.6f}"
                    ),
                    (
                        ""
                        if sample.target_ai_closing_speed_mps is None
                        else f"{sample.target_ai_closing_speed_mps:.6f}"
                    ),
                ]
            )
    return output


def run_interactive() -> int:
    enable_windows_dpi_awareness()
    pygame.init()
    pygame.display.set_caption("INTERCEPTRON — Vision-only Interception")
    screen = pygame.display.set_mode(WINDOW_SIZE, pygame.RESIZABLE)
    clock = pygame.time.Clock()
    setup = SetupScreen(screen.get_size())
    renderer = WorldRenderer()
    simulation: InterceptionSimulation | None = None
    current_config: SimulationConfig | None = None
    app_state = "setup"
    accumulator = 0.0
    time_scales = TIME_SCALES
    time_scale_index = DEFAULT_TIME_SCALE_INDEX
    analysis_open = False
    help_open = False
    info_open = False
    info_page = 0
    cinematic_hit_seen = False
    right_mouse_dragging = False
    window_focused = True
    running = True

    def set_mouse_capture(captured: bool) -> None:
        nonlocal right_mouse_dragging
        right_mouse_dragging = captured
        pygame.event.set_grab(captured)
        pygame.mouse.set_visible(not captured)
        if captured:
            pygame.mouse.set_pos(
                (screen.get_width() // 2, screen.get_height() // 2)
            )
        pygame.mouse.get_rel()

    def read_manual_input(suppressed: bool) -> ManualControlInput:
        if suppressed:
            return ManualControlInput()
        keys = pygame.key.get_pressed()
        return ManualControlInput(
            forward=float(keys[pygame.K_w]) - float(keys[pygame.K_s]),
            turn=float(keys[pygame.K_d]) - float(keys[pygame.K_a]),
            vertical=float(keys[pygame.K_e]) - float(keys[pygame.K_q]),
            full_thrust=bool(
                keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
            ),
            brake=bool(keys[pygame.K_LCTRL] or keys[pygame.K_RCTRL]),
        )

    while running:
        real_dt = min(clock.tick(120) / 1000.0, 0.1)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                set_mouse_capture(False)
                running = False
                continue
            if event.type == pygame.VIDEORESIZE:
                requested = (
                    max(MINIMUM_WINDOW_SIZE[0], event.w),
                    max(MINIMUM_WINDOW_SIZE[1], event.h),
                )
                if requested != screen.get_size():
                    screen = pygame.display.set_mode(requested, pygame.RESIZABLE)
                setup.size = screen.get_size()
                continue
            if event.type == pygame.WINDOWFOCUSLOST:
                window_focused = False
                set_mouse_capture(False)
                if simulation is not None:
                    simulation.clear_manual_input()
            elif event.type == pygame.WINDOWFOCUSGAINED:
                window_focused = True
            if app_state == "setup":
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False
                    continue
                config = setup.handle_event(event)
                requested_display = setup.consume_display_request()
                if requested_display is not None:
                    screen = pygame.display.set_mode(
                        requested_display, pygame.RESIZABLE
                    )
                    setup.size = requested_display
                if config is not None:
                    current_config = config
                    simulation = InterceptionSimulation(config)
                    renderer = WorldRenderer()
                    app_state = "simulation"
                    accumulator = 0.0
                    cinematic_hit_seen = False
                continue

            if simulation is None:
                continue
            if (
                event.type == pygame.MOUSEWHEEL
                and not analysis_open
                and not help_open
                and not info_open
                and not renderer.settings_visible
            ):
                renderer.adjust_zoom(event.y)
                continue
            if (
                event.type == pygame.MOUSEBUTTONDOWN
                and event.button == 1
                and not analysis_open
                and not help_open
                and not info_open
            ):
                action = renderer.handle_left_click(event.pos, simulation)
                if action is not None:
                    set_mouse_capture(False)
                    if action == "time_down":
                        time_scale_index = max(0, time_scale_index - 1)
                    elif action == "time_up":
                        time_scale_index = min(
                            len(time_scales) - 1,
                            time_scale_index + 1,
                        )
                    elif action == "info":
                        info_open = True
                        analysis_open = False
                        help_open = False
                    elif action == "analysis":
                        analysis_open = True
                        info_open = False
                        help_open = False
                    elif action == "help":
                        help_open = True
                        info_open = False
                        analysis_open = False
                    continue
            if (
                event.type == pygame.MOUSEBUTTONDOWN
                and event.button == 3
                and not analysis_open
                and not help_open
                and not info_open
                and not renderer.settings_visible
            ):
                set_mouse_capture(True)
                continue
            if event.type == pygame.MOUSEBUTTONUP and event.button == 3:
                set_mouse_capture(False)
                continue
            if (
                event.type == pygame.MOUSEMOTION
                and right_mouse_dragging
                and not analysis_open
                and not help_open
                and not info_open
            ):
                continue
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    set_mouse_capture(False)
                    if analysis_open:
                        analysis_open = False
                    elif help_open:
                        help_open = False
                    elif info_open:
                        info_open = False
                    else:
                        running = False
                elif event.key == pygame.K_SPACE:
                    simulation.paused = not simulation.paused
                    simulation.clear_manual_input()
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    time_scale_index = min(len(time_scales) - 1, time_scale_index + 1)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    time_scale_index = max(0, time_scale_index - 1)
                elif event.key == pygame.K_v:
                    renderer.cycle_view()
                elif event.key == pygame.K_F3:
                    renderer.view_mode = 2
                    renderer.reset_view_offset()
                elif event.key == pygame.K_F4:
                    renderer.settings_visible = not renderer.settings_visible
                elif event.key == pygame.K_m:
                    renderer.minimap_visible = not renderer.minimap_visible
                elif event.key == pygame.K_g:
                    renderer.verification_visible = True
                    simulation.capture_prediction_check()
                elif event.key == pygame.K_c:
                    renderer.reset_view_offset()
                elif event.key == pygame.K_TAB:
                    mode = simulation.cycle_control_mode()
                    renderer.view_mode = (
                        0 if mode is ControlMode.AUTO else 1
                    )
                    renderer.reset_view_offset()
                elif event.key == pygame.K_F2:
                    set_mouse_capture(False)
                    analysis_open = not analysis_open
                    help_open = False
                    info_open = False
                elif event.key == pygame.K_F1:
                    set_mouse_capture(False)
                    info_open = not info_open
                    analysis_open = False
                    help_open = False
                elif info_open and event.key in (
                    pygame.K_RIGHT,
                    pygame.K_PAGEDOWN,
                ):
                    info_page = (info_page + 1) % INFO_PAGE_COUNT
                elif info_open and event.key in (
                    pygame.K_LEFT,
                    pygame.K_PAGEUP,
                ):
                    info_page = (info_page - 1) % INFO_PAGE_COUNT
                elif event.key == pygame.K_h:
                    set_mouse_capture(False)
                    help_open = not help_open
                    analysis_open = False
                    info_open = False
                elif (
                    event.key == pygame.K_x
                    and not analysis_open
                    and not help_open
                    and not info_open
                ):
                    simulation.toggle_controlled_engine()
                elif event.key == pygame.K_r and current_config is not None:
                    set_mouse_capture(False)
                    simulation = InterceptionSimulation(current_config)
                    renderer = WorldRenderer()
                    accumulator = 0.0
                    cinematic_hit_seen = False
                    info_open = False
                elif event.key == pygame.K_n:
                    set_mouse_capture(False)
                    app_state = "setup"
                    simulation = None
                    analysis_open = False
                    help_open = False
                    info_open = False
                elif event.key == pygame.K_F5:
                    exported = export_telemetry(simulation)
                    simulation._event(f"Telemetry exported: {exported.name}")
                elif event.key == pygame.K_o:
                    simulation.toggle_sensor_occlusion()

        if (
            app_state == "simulation"
            and right_mouse_dragging
            and not analysis_open
            and not help_open
            and not info_open
        ):
            relative_x, relative_y = pygame.mouse.get_rel()
            if relative_x or relative_y:
                renderer.rotate_view(
                    relative_x,
                    relative_y,
                    bool(pygame.key.get_mods() & pygame.KMOD_SHIFT),
                )
            pygame.mouse.set_pos(
                (screen.get_width() // 2, screen.get_height() // 2)
            )
            pygame.mouse.get_rel()

        if app_state == "setup":
            setup.draw(screen)
        elif simulation is not None:
            manual_input_suppressed = (
                simulation.paused
                or analysis_open
                or help_open
                or info_open
                or renderer.settings_visible
                or right_mouse_dragging
                or not window_focused
                or simulation.finished
                or simulation.hit
            )
            simulation.set_manual_input(
                read_manual_input(manual_input_suppressed)
            )
            if (
                not simulation.paused
                and not analysis_open
                and not help_open
                and not info_open
            ):
                accumulator += real_dt * time_scales[time_scale_index]
                steps_this_frame = 0
                while accumulator >= FIXED_STEP and steps_this_frame < 16:
                    simulation.step(FIXED_STEP)
                    renderer.update_trails(simulation, FIXED_STEP)
                    if simulation.hit and not cinematic_hit_seen:
                        renderer.view_mode = 1
                        cinematic_hit_seen = True
                    accumulator -= FIXED_STEP
                    steps_this_frame += 1
            renderer.draw_world(screen, simulation)
            renderer.draw_hud(
                screen,
                simulation,
                time_scales[time_scale_index],
                clock.get_fps(),
            )
            if info_open:
                renderer.draw_info(screen, simulation, info_page)
            elif analysis_open:
                renderer.draw_analysis(screen, simulation)
            elif help_open:
                renderer.draw_help(screen)

        pygame.display.flip()

    set_mouse_capture(False)
    pygame.quit()
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="INTERCEPTRON vision-only interception simulation"
    )
    parser.add_argument(
        "--headless-steps",
        type=int,
        help="run a deterministic non-interactive simulation for N fixed 60 Hz steps",
    )
    parser.add_argument(
        "--scenario",
        choices=[key for key, _ in SCENARIOS],
        default="evasive",
    )
    parser.add_argument("--screenshot", help="save the final headless frame as a PNG")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.headless_steps is not None or args.screenshot:
        return run_headless(args)
    return run_interactive()


if __name__ == "__main__":
    raise SystemExit(main())
