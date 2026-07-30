# ZENITH

ZENITH is a windowed interactive desktop proof-of-concept for vision-only interception. It estimates an intruder's range from a monocular camera, builds conservative maneuver-containment ovals, and outputs the maneuver for a propulsion-constrained interceptor. No radar, lidar, rangefinder, or target ground-truth coordinates are used by the guidance calculation.

The setup offers two explicitly labeled perception backends. `YOLO CUSTOM` rasterizes a clean rigid-camera BGR frame, runs the bundled custom `aerial_target` weights, and passes only its boxes and confidence values into the existing signal, range, tracking, oval, and guidance pipeline. `SYNTHETIC BOX + POSE` remains a dependency-free deterministic fallback. YOLO detects a generic aerial target; it deliberately does not reveal the exact vehicle model, because the project idea obtains dimensions and maneuver limits from the later simulated signal lookup.

The project includes six controllable drones with distinct low-poly quadcopter, swept-wing, delta-wing, blended-wing, and directional UFO-style meshes. `WRAITH-S` is the fastest drone; `TALON-R` turns harder; and `SMART EVADER` combines high acceleration, braking, and turn authority with a separate threat-aware `TRICKY AI` behavior. The setup can launch one, two, or three enemy contacts. Every contact gets an independent detection, signal identity, range/velocity track, oval solution, and camera-derived threat score. Multi-contact guidance compares only equal horizons—2-second with 2-second, 5-second with 5-second—and steers toward the center of one qualifying two-target overlap. Once our physical reach enters either member's smaller nested oval, guidance commits to that contact. Two literal single-stage rockets are selectable on either side. Each ignites automatically, expends a finite nonrestartable booster, uses a separate limited RCS steering budget, and then coasts under drag and gravity.

![DPI-aware windowed setup](artifacts/setup_windowed.png)

![Optional three-contact setup](artifacts/multi_contact_setup.png)

![Three independent tracks with a two-target shared-overlap aim](artifacts/multi_contact_demo.png)

![In-app same-horizon overlap explanation](artifacts/multi_contact_info.png)

![Clean sensor frame used by custom YOLO](artifacts/yolo_sensor_frame.png)

![Setup selecting the bundled custom-YOLO backend](artifacts/yolo_setup.png)

![Custom-YOLO closed loop with measured inference timing](artifacts/yolo_closed_loop.png)

![Generated custom-YOLO training examples and exact offline labels](artifacts/yolo_dataset_preview.jpg)

![Actual custom-YOLO predictions on held-out frames](artifacts/yolo_inference_preview.jpg)

![Custom-YOLO training and validation curves](artifacts/yolo_training_results.png)

![Smart Evader and separate Tricky AI setup](artifacts/smart_evader_setup.png)

![Tricky AI onboard engagement with a blocked red edge and no misleading red fill](artifacts/smart_evader_tricky.png)

![Tricky AI close engagement with a fully reachable green oval](artifacts/smart_evader_tricky_close.png)

![Mid-flight prototype](artifacts/prototype_midflight.png)

![Rolled chase-camera free look](artifacts/camera_freelook.png)

![Visual lock loss and search state](artifacts/lock_loss_search.png)

![Manual target takeover and authority HUD](artifacts/manual_takeover.png)

![Spectator view proving the common target/oval plane](artifacts/spectator_oval_plane.png)

![Expandable HUD and estimated-track minimap](artifacts/expandable_hud.png)

![Recorded-camera two-second prediction check](artifacts/prediction_check.png)

![Full in-app gravity and aerodynamics reference](artifacts/info_page_3.png)

[Full information page 1: sensor/readouts](artifacts/info_page_1.png) ·
[page 2: ovals/guidance](artifacts/info_page_2.png) ·
[page 3: gravity/aerodynamics](artifacts/info_page_3.png) ·
[page 4: lock loss/cameras/verification](artifacts/info_page_4.png)

![Rocket interception](artifacts/rocket_intercept.png)

![Finite-burn rocket used as our interceptor](artifacts/rocket_interceptor.png)

## Run it

On Windows, double-click `run_zenith.bat`. The application is DPI-aware, opens in a resizable 1050 × 700 window, and is not fullscreen even when Windows display scaling is 125% or 150%. The setup screen offers 1050 × 700, 1152 × 720, and 1280 × 800 window sizes independently of the simulated camera resolution.

For the trained YOLO backend, create the isolated environment once:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup_yolo.ps1
```

`run_zenith.bat` automatically uses that environment. Select `DETECTOR: YOLO CUSTOM` on the setup screen. The fallback installation needs only Pygame:

```powershell
python -m pip install -r requirements.txt
python app.py
```

The installed environment needs Python 3.11+ and Pygame 2.6+.

## Recommended first demonstration

1. Keep `TALON-R` as our interceptor and `FALCON-X1` as the target.
2. Select `DETECTOR: YOLO CUSTOM`, `EVASIVE MANEUVERS`, `1920 × 1080`, and start the simulation. For the multi-target demonstration, also select `3` under `ENEMY CONTACTS`.
3. Observe the target label change from `UNKNOWN / QUERYING` to `FALCON-X1`.
4. Point out the 1, 2, 3, and 5 second prediction ovals. The four large dots are the cardinal summary, but the border color tests all 96 displayed edge directions. Green segments are reachable and red segments are blocked; only a completely green `96/96` edge can be selected and receive a faint green fill. Red never fills the center.
   In multi-contact mode, the cyan region is the intersection of a same-horizon pair, and the white diamond is its shared centroid. Once the interceptor enters one contact's smaller nested oval, the HUD changes from `SHARED` to `COMMITTED`.
5. Press `F2` to show the camera-resolution and range-error analysis.
6. Press `V` to cycle views or `F3` to jump directly to the independent spectator/tactical view. Hold the right mouse button to capture the pointer for unlimited free look; hold `Shift` while moving it to roll the presentation camera. Use the mouse wheel to zoom without changing the sensor. Release the button to restore the pointer, or press `C` to center the view and reset zoom.
7. Press `O` to obscure the camera: lock, guidance, range, and ovals disappear immediately. Press `O` again and watch the search pattern genuinely reacquire the target.
8. Click the bottom headers to expand/collapse individual panels. Press `M` for the estimated-track minimap and `G` to record a two-second prediction check in the old virtual camera frame.
9. Press `Tab` to take over our vehicle, then press it again to take over the target. Use `W/S`, `A/D`, `Q/E`, `Shift`, and `Ctrl`; the authority HUD proves which requests each propulsion model can execute. `X` cuts/restarts a drone engine, while a solid rocket correctly rejects that request.
10. Start a new simulation with `SMART EVADER` as the target and select `TRICKY AI`. Its Target panel and event log expose deterministic decisions such as jinks, climbs, speed shifts, brake traps, and escape boosts.
11. Select `SKYFALL-R1` or `LANCE-M2` as **our interceptor**, and show the automatic booster/RCS timers. Rockets remain selectable as incoming targets too.
12. Press `F5` to export the run's verification telemetry, including the current target-AI decision.

## Controls

| Key | Action |
|---|---|
| `Space` | Pause or continue |
| `+` / `-` | Change time scale from 0.25× to 4× |
| `V` | Cycle onboard, chase, and spectator/tactical views |
| `F3` | Jump directly to spectator/tactical view |
| `F4` | Toggle presentation settings |
| Hold right mouse | Capture the pointer for unlimited free look |
| `Shift` + right mouse | Roll the presentation camera |
| Mouse wheel | Zoom the presentation camera; the simulated sensor remains 90° |
| `C` | Center free look and reset presentation zoom |
| `Tab` | Cycle `AUTO -> PLAYER / OUR VEHICLE -> PLAYER / TARGET -> AUTO` |
| `W` / `S` | Forward thrust / reverse or decelerate |
| `A` / `D` | Turn left / right within propulsion limits |
| `Q` / `E` | Descend / climb; release to hold the assisted altitude |
| `Shift` | Request full available maneuver authority |
| `Ctrl` | Airbrake where the selected vehicle supports one |
| `X` | Cut/restart a drone engine; solid rocket boosters cannot be cut or restarted |
| `M` | Toggle the top-down estimated-track minimap |
| `G` | Record a two-second prediction check in the saved camera frame |
| `F1` | Toggle the four-page full system information reference |
| `F2` | Toggle engineering analysis |
| `H` | Toggle help |
| `O` | Toggle camera occlusion for lock-loss/reacquisition proof |
| `F5` | Export verification telemetry as CSV |
| `R` | Restart the current setup |
| `N` | Start a new setup |
| `Esc` | Close an overlay, then exit |

## What the four panels mean

- `PRIORITY TARGET`: the selected contact ID plus drone or rocket specifications obtained after visual detection and simulated signal lookup.
- `CALCULATIONS`: detector adapter, focal length, apparent target size, estimated range, uncertainty, and camera-relative `Dx/Dy/Dz`. This panel starts expanded.
- `OUR VEHICLE`: propulsion model, world position, velocity, acceleration, engine/lift, and rocket booster/RCS timers where applicable.
- `RELATIVE`: camera-relative velocity, closing speed, contact time, same-horizon shared/committed guidance state, complete-edge result, track uncertainty, and image-track priority score.

Each panel header is independently clickable. Target truth and error appear only in the separately expanded `SIMULATION-TRUTH VERIFICATION` window.

`X` is camera-left/right, `Y` is camera-up/down, and `Z` is the line from the camera toward the target.

## System pipeline

```mermaid
flowchart LR
    A[Monocular image] --> B[Generic drone detection]
    B --> C[Bounding box and bearing]
    B --> D[Signal lookup]
    D --> E[Known dimensions and limits]
    C --> F[Pinhole range and uncertainty]
    E --> F
    F --> G[Filtered position and velocity track]
    C --> L{Visual lock valid?}
    L -- No --> M[Disable guidance and extrapolate image bearing]
    M --> B
    L -- Yes --> F
    E --> H[1, 2, 3, 5 s maneuver ovals]
    G --> H
    H --> I[96-direction complete-edge reachability]
    I --> J[Weighted oval / transparent fallback / terminal pursuit]
    J --> K[Acceleration command]
```

The defense guidance never reads the simulation's exact target position. In `YOLO CUSTOM` mode, scene truth stops at the software sensor renderer: the model receives only a clean BGR pixel array, and runtime code cannot access the offline dataset annotations. A one-time association seed binds generic image tracks to the simulated actors that answer the exercise's later signal query; seed coordinates never enter range, tracking, ovals, priority, or guidance. In fallback mode, truth instead stops at the explicitly labeled synthetic detector adapter. Physical collision and marked verification data necessarily use truth in both modes. The guidance path reads detector boxes/bearings/confidence, signal-resolved model data, successive camera-derived position measurements, and our drone's own integrated state. Target velocity and acceleration are estimated from those measurements rather than copied from target state. YOLO supplies no hidden target pose: drones use a known planform span with the box major axis, while long rockets use their known cross-section with the box minor axis; the full bounding diameter contributes only to the declared uncertainty/audit value. The optional `TRICKY AI` target controller is a separately declared adversary test harness: it may observe our approach to choose its own bounded maneuver, but cannot feed truth back into detection or guidance. The rigid sensor cannot turn toward truth, preserve lock through a hidden gimbal, or see behind the airframe. A lost image detection invalidates guidance; search uses only the final filtered image-bearing history.

At every 60 Hz update the current sensor pose is frozen and each mathematical ellipse bounds the pixels the identified target could occupy after 1, 2, 3, or 5 seconds. The bound includes propulsion support, perspective at the closest permitted future depth, and the track's position/velocity uncertainty. It is then back-projected onto the plane through the estimated target, perpendicular to the current optical axis. If the future set could cross the camera plane, the horizon is labeled `UNBOUNDED / CAMERA CROSSING` instead of drawing a dishonest finite oval.

The four large marked points remain the easy cardinal explanation. They do not prove the diagonal edge: the actual decision checks all 96 directions used to render the border. Each reachable edge segment is green and each blocked segment is red, so partial reachability is visibly mixed; the label still reports `n/96`. Only a completely green border gets a faint green interior tint. Red is never used as an interior fill because failure at the outer border does not imply that the center is unreachable. Guidance chooses only the largest completely green oval and biases its aim from the center toward measured likely motion while remaining within 65% of the ellipse. With no green oval it explicitly reports `NO GUARANTEED OVAL` and follows the unchanged-motion prediction. Reaching the one-second region activates terminal collision lead.

With several contacts, all target ovals are still calculated independently. At most one pair is used for a maneuver. For each pair the solver compares matching horizons only and projects both ellipses into the frozen camera's common normalized image plane, which remains valid when the targets are at different depths. A pair qualifies only when each ellipse center lies inside the other ellipse and the shared centroid is physically reachable by that horizon. The largest qualifying horizon wins; cyan shows its polygonal intersection. As soon as the interceptor can enter either member's smaller nested oval, that target becomes a sticky committed lock. Losing its real visual detection cancels the commitment and resumes overlap search.

Mouse free-look and wheel zoom change only the presentation renderer, not the sensor axis, detection, or guidance. The zoom is a bounded 24°–100° presentation field of view; the sensor remains fixed at its configured 90°. While the right button is held, the pointer is captured and hidden so window edges cannot stop rotation; release, focus loss, overlays, or simulation exit restore it. Vehicle key input is suppressed during mouse capture so `Shift` can roll without also commanding thrust.

Manual takeover is an advisory proof mode rather than a physics bypass. Inputs pass through the same thrust direction, airbrake, turn rate, speed, drag, gravity, wing lift/stall, fuel, tilt, and altitude constraints used by autonomy. Aegis-Q4 and Smart Evader begin level with zero vertical velocity at half maximum speed; the corrected vectored attitude model visibly tilts the nose down for forward thrust while respecting their camera-preserving body limit. A powered rotorcraft must spend thrust to hover; an engine-off wing glides before sinking. Fixed-wing autonomy avoids overspeed by scheduling throttle and closing speed early; a negative request cuts propulsion and lets drag slow the aircraft instead of inventing reverse thrust or an automatic speed-triggered brake. A rocket automatically burns at full thrust, cannot reverse/throttle/restart, steers only while RCS remains, and falls after burnout. During our-vehicle takeover, the normal oval solution remains visible as an advisory.

`F1` opens a four-page in-app reference. The fixed `SENSOR BORESIGHT` appears only in onboard sensor view; the separate diamond is the actual world-space guidance aim. `M` opens an X/Z minimap with the sensor FOV wedge and altitude-coded target marker. `G` records a two-second prediction in its original virtual camera frame so moving or rotating the live camera cannot invalidate the comparison.

## Verification

Run all seven deterministic behavior tests:

```powershell
python verify_prototype.py --duration 30 --csv artifacts\verification.csv
python -m unittest discover -v
python benchmark_performance.py
python benchmark_performance.py --enemies 3
```

The trained backend can be exercised separately through the isolated environment:

```powershell
.\.venv-yolo\Scripts\python.exe app.py --headless-steps 1800 --detector yolo
.\.venv-yolo\Scripts\python.exe verify_prototype.py --duration 30 --detector yolo
```

The verifier reports identification, interception result, hit time, minimum separation, and range-estimation MAE/RMSE. The automated suite additionally checks all 96 edge directions, green-only fill semantics, diagonal failure despite `4/4` cardinals, weighted-point limits, camera-crossing invalidation, detector-pose isolation, subpixel range continuity, frame-stable rendered oval radii, level Aegis/Smart starts, correct vectored pitch direction, saved-frame two-second containment, every drone as a baseline interceptor, half-speed no-lock cruise, onboard-only boresight, finite rocket burnout/RCS, both rockets as interceptors and targets, visual lock loss/reacquisition, gravity/aerodynamics, and every default interception scenario.

The checked-in [synthetic verification report](artifacts/verification.csv) and
[custom-YOLO verification report](artifacts/yolo_verification.csv) both record
successful physical contact in all seven scenarios. The YOLO CSV intentionally
shows larger monocular range error at tiny apparent sizes; model inference
misses are never replaced with synthetic boxes.

The benchmark runs the same evasive simulation and complete 1050 × 700 software-rendering path repeatedly without opening a window. Its numbers are machine-dependent; it does not reduce the fixed 60 Hz physics rate or skip any of the 96 displayed edge checks.

## Project structure

```text
app.py                    Desktop UI, setup screen, controls, CSV export
verify_prototype.py       Deterministic seven-scenario verification
benchmark_performance.py  Repeatable physics/guidance/render benchmark
zenith/
  camera.py               Pinhole camera, detector output, range estimator
  controls.py             Manual authority modes and assisted flight requests
  guidance.py             Tracking, prediction ovals, reachability, commands
  math3d.py               Vector, camera-basis, and transform mathematics
  models.py               Six drones and two controllable rocket profiles
  meshes.py               Bundled procedural drone and rocket polygon meshes
  physics.py              60 Hz gravity, lift/stall, thrust, drag, and impacts
  rendering.py            3D scene, HUD, spectator, analysis, full info display
  vision.py               Clean sensor renderer, YOLO inference, box association
  simulation.py           Detection → signal → guidance state machine
tools/
  generate_yolo_dataset.py Deterministic labeled sensor-frame generation
  train_yolo.py            GPU training, validation, and weight export
  create_yolo_preview.py    Held-out model-prediction evidence montage
models/zenith_yolo.pt      Bundled custom aerial-target detector weights
models/zenith_yolo_metrics.json Held-out metrics and model SHA-256
models/README.md            Model card, intended use, and limitations
tests/test_core.py        Automated numerical and scenario checks
docs/YOLO_INTEGRATION.md  Training, runtime boundary, metrics, limitations
docs/TECHNICAL_REPORT.md  Equations, assumptions, degradation analysis
docs/PRESENTATION_GUIDE.md Suggested presentation and likely questions
docs/VERIFICATION_AUDIT.md Requirement-to-code-and-test evidence matrix
```

## Prototype boundary

The bundled YOLO weights are trained on software-rendered ZENITH imagery, not real outdoor footage. This proves real model inference and a replaceable pixel boundary; it does not prove field readiness. Deployment on a physical drone would require representative real camera data, domain adaptation, calibrated optics, adverse-weather and lighting evaluation, measured false-positive/false-negative rates, hardware timing, redundant safety systems, and regulatory review. At long range the target may occupy only two or three pixels, which is genuinely insufficient for reliable recognition; YOLO mode honestly remains in search until the image contains enough information.

The aerodynamic layer is intentionally a verifiable presentation model, not computational fluid dynamics: gravity is exact, while lift uses exposed stall-speed and lift-efficiency parameters plus the current flight-path direction and airspeed. A production vehicle would require measured lift/drag curves, mass, wing area, wind, control-surface dynamics, and hardware validation.

## Optional external model sources

The bundled meshes are original project code, so the prototype has no asset-license dependency. If higher-detail assets are wanted later:

- [NASA 3D Resources](https://science.nasa.gov/3d-resources/) provides downloadable rocket and spacecraft assets under NASA's media-usage guidance.
- [NASA Atlas V 401 glTF](https://science.nasa.gov/resource/atlas-v-401-3d-model/) is an official 2.08 MB rocket model.
- [Kira's Drone on Sketchfab](https://sketchfab.com/3d-models/drone-eac2b4bc20f54b3ba8c3ddbcdf03c8d6) is downloadable under CC Attribution, but its 275k triangles require simplification before use in this software renderer.
- [Khronos glTF Sample Assets](https://github.com/KhronosGroup/glTF-Sample-Assets) is a useful reference for a future standards-based importer; individual asset licenses must still be checked.
