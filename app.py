"""ZENITH interactive desktop prototype entry point."""

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
from zenith.math3d import Vec3
from zenith.models import DRONE_SPECS, TARGET_SPECS
from zenith.physics import DroneState
from zenith.rendering import (
    AMBER,
    BG_TOP,
    CYAN,
    GREEN,
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
        catalogue = DRONE_SPECS if prefix == "own" else TARGET_SPECS
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
            f"AIRBRAKE   {spec.brake_accel:.0f} m/s²",
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

        surface.blit(self.title.render("ZENITH", True, WHITE), (48, 30))
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
        pipeline = "VISUAL DETECTION  >  SIGNAL LOOKUP  >  PINHOLE RANGE  >  OVAL REACHABILITY  >  MANEUVER"
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
            "OUR DRONE / INTERCEPTOR",
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
        surface.blit(self.small.render("MONOCULAR SENSOR RESOLUTION // 75° HFOV", True, MUTED), (left + 4, options_y))
        for index, resolution in enumerate(SENSOR_OPTIONS):
            self._button(
                surface,
                f"sensor_{index}",
                pygame.Rect(left + index * 166, options_y + 23, 154, 38),
                f"{resolution[0]} × {resolution[1]}",
                index == self.sensor_index,
            )

        launch_rect = pygame.Rect(left + content_width - 310, options_y + 16, 310, 54)
        self._button(surface, "launch", launch_rect, "START 60 Hz SIMULATION", True, GREEN)
        if self.error:
            error_text = self.font.render(self.error, True, RED)
            surface.blit(error_text, (launch_rect.right - error_text.get_width(), launch_rect.bottom + 8))

        footer_y = height - 38
        pygame.draw.line(surface, (39, 76, 89), (left, footer_y - 10), (left + content_width, footer_y - 10))
        footer = (
            "Five playable drones + two rocket threats • place both vehicles • no radar/lidar/rangefinder • "
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
                self.interceptor_index = (self.interceptor_index - 1) % len(DRONE_SPECS)
            elif key == "own_next":
                self.interceptor_index = (self.interceptor_index + 1) % len(DRONE_SPECS)
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
        self.error = ""
        return SimulationConfig(
            interceptor_code=DRONE_SPECS[self.interceptor_index].code,
            target_code=TARGET_SPECS[self.target_index].code,
            interceptor_position=own,
            target_position=target,
            scenario=SCENARIOS[self.scenario_index][0],
            camera=CameraModel(resolution[0], resolution[1], 75.0),
        )


def run_headless(args: argparse.Namespace) -> int:
    target_code = "SR1" if args.scenario == "rocket_attack" else "FX1"
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
        "version": "prototype-02",
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
    output = output_dir / f"zenith_{simulation.config.scenario}_{stamp}.csv"
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
                "target_type",
                "target_model",
                "interceptor_model",
                "scenario",
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
                    simulation.target.spec.vehicle_type,
                    simulation.target.spec.name,
                    simulation.interceptor.spec.name,
                    simulation.config.scenario,
                ]
            )
    return output


def run_interactive() -> int:
    enable_windows_dpi_awareness()
    pygame.init()
    pygame.display.set_caption("ZENITH — Vision-only Interception")
    screen = pygame.display.set_mode(WINDOW_SIZE, pygame.RESIZABLE)
    clock = pygame.time.Clock()
    setup = SetupScreen(screen.get_size())
    renderer = WorldRenderer()
    simulation: InterceptionSimulation | None = None
    current_config: SimulationConfig | None = None
    app_state = "setup"
    accumulator = 0.0
    time_scales = (0.25, 0.5, 1.0, 2.0, 4.0)
    time_scale_index = 2
    analysis_open = False
    help_open = False
    cinematic_hit_seen = False
    running = True

    while running:
        real_dt = min(clock.tick(120) / 1000.0, 0.1)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
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
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    if analysis_open:
                        analysis_open = False
                    elif help_open:
                        help_open = False
                    else:
                        running = False
                elif event.key == pygame.K_SPACE:
                    simulation.paused = not simulation.paused
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    time_scale_index = min(len(time_scales) - 1, time_scale_index + 1)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    time_scale_index = max(0, time_scale_index - 1)
                elif event.key == pygame.K_v:
                    renderer.cycle_view()
                elif event.key == pygame.K_a:
                    analysis_open = not analysis_open
                    help_open = False
                elif event.key == pygame.K_h:
                    help_open = not help_open
                    analysis_open = False
                elif event.key == pygame.K_r and current_config is not None:
                    simulation = InterceptionSimulation(current_config)
                    renderer = WorldRenderer()
                    accumulator = 0.0
                    cinematic_hit_seen = False
                elif event.key == pygame.K_n:
                    app_state = "setup"
                    simulation = None
                    analysis_open = False
                    help_open = False
                elif event.key == pygame.K_e:
                    exported = export_telemetry(simulation)
                    simulation._event(f"Telemetry exported: {exported.name}")
                elif event.key == pygame.K_o:
                    simulation.toggle_sensor_occlusion()

        if app_state == "setup":
            setup.draw(screen)
        elif simulation is not None:
            if not simulation.paused and not analysis_open and not help_open:
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
            if analysis_open:
                renderer.draw_analysis(screen, simulation)
            elif help_open:
                renderer.draw_help(screen)

        pygame.display.flip()

    pygame.quit()
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ZENITH vision-only interception prototype")
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
