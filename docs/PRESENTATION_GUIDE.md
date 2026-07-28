# Presentation guide

## Suggested 6-minute structure

### 1. State the problem — 30 seconds

“The interceptor has only one monocular camera. It has no radar, lidar, or rangefinder. Once a generic drone is visible, the simulated signal lookup supplies the model dimensions and maneuver limits given by the exercise.”

### 2. Explain the range calculation — 60 seconds

Point to `CALCULATIONS`:

```text
range = focal length in pixels × known physical span / apparent pixel span
```

Explain that the image center gives bearing, while known size gives scale. `Dx`, `Dy`, and `Dz` are reconstructed in the camera frame. Mention that target rotation is compensated using the known 3D box and pose, not a fixed width alone.

### 3. Explain your oval idea — 90 seconds

Point to the 1, 2, 3, and 5 second ovals:

- each oval represents positions allowed by the target's known acceleration and speed;
- every oval, its four extremes, its center, the projected unchanged path, and the target lie in one plane perpendicular to our sensor camera view;
- the border is inflated from propulsion-specific acceleration support, so the target's projected position cannot escape it under the stated limits;
- the four extreme points are calculated separately and tested for our reachability;
- a green border means all 4 extremes are reachable; if even one required point fails, the border is red and reports the exact `n/4` count;
- an unreachable oval remains visible and red instead of disappearing;
- if all four extremes are reachable, the center is a safe guidance objective;
- if no complete oval works, the system follows the unchanged-trajectory prediction;
- near contact, large ovals are ignored and terminal pursuit matches the target's motion.

### 4. Prove it is camera-driven — 60 seconds

Point out:

- identity begins as `UNKNOWN / QUERYING`;
- pixel size, focal length, estimated range, and uncertainty are visible;
- `TRUE / ERROR` is labeled verification-only;
- the exact target coordinate never enters the guidance function;
- pressing `F5` exports the camera estimate and truth comparison for every sampled frame.

Press `O` once. Point out that the detection brackets, metric readout, ovals, and maneuver guidance all disappear immediately. The gimbal scans from the last seen bearing without using target truth. Press `O` again; only a new image detection can reacquire lock and restart guidance.

Hold the right mouse button to capture the pointer and inspect the scene without being stopped by a window edge. Hold `Shift` while moving the mouse to roll the horizon and view the prediction plane obliquely. Explain that this is a presentation-camera offset: it does not rotate the actual sensor or change guidance, and vehicle keys are suppressed during capture. Release the right button to restore the pointer, then press `C` to return to the centered sensor presentation.

Press `F3` for the independent spectator/tactical camera. This is the clearest view for proving that the target and all four ovals occupy the same plane. Press `F1` at any time for the four-page explanation of every coordinate, color, force, guidance state, camera, and verification boundary.

### 5. Demonstrate robustness — 60 seconds

Use `N` to show the selectable behaviors and threats:

- weave tests continuous side motion;
- evasive tests changing acceleration direction;
- airbrake tests nonlinear speed changes;
- rotating target tests apparent-size changes.
- rocket attack selects a fast incoming target with rocket-specific physics and status.

The six-scenario verifier can be shown from a terminal:

```powershell
python verify_prototype.py
```

### 6. Show analysis and finish — 60 seconds

Press `F2`. Explain that range error grows inversely with pixel count. Show the minimum-resolution table, which changes with the selected target size. Finish with the successful collision, explosion, and falling physics.

## Likely questions

### “Is the true target position used?”

Only to render what the camera would see, detect physical collision, and calculate the marked verification error. Guidance receives the detected image box, resolved specifications, camera estimate, and our drone's integrated state.

### “Can one camera really calculate range?”

Only when scale is known. The exercise gives the physical target dimensions, so the pinhole relationship supplies scale. For an unknown object, one frame cannot uniquely determine absolute range.

### “Why does the estimate jump at long distance?”

The target occupies only a few pixels. A one-pixel width change can represent tens of metres. The displayed uncertainty and analysis graph make this limitation measurable; the filtered track prevents that noise from becoming raw maneuver commands.

### “Why are the prediction regions ovals?”

The target's bounded acceleration spreads its future projected position in the plane perpendicular to the current camera axis. Directional positive/negative acceleration supports give component bounds. The displayed ellipse circumscribes those bounds, making it a conservative containment border recomputed at 60 Hz.

### “Does checking four points prove the whole oval is reachable?”

The four checks prove whether our drone can reach the four marked extremes used by this software's decision rule. Separately, the oval's conservative support construction proves containment of the target's projected acceleration-bounded set. A production system would add full 3D range uncertainty and formal actuator/wind uncertainty.

### “Can an aircraft accelerate in any direction in this simulation?”

No. Multirotors must slew and tilt their thrust vector. Fixed-wing engines push only along the nose; turns come from bounded aerodynamic lateral force. Rockets have forward thrust and limited steering but cannot command reverse thrust. The guidance request is always filtered through that vehicle-specific physics layer.

### “Where is gravity, and why does a wing fall differently?”

Gravity is continuously applied at `9.81 m/s²` to every airborne vehicle. A powered multirotor spends upward thrust to cancel it. A wing creates lift perpendicular to its flight path; that lift depends on airspeed, weakens below the model's visible stall speed, and vanishes during a vertical fall. With its engine cut, a fast wing therefore glides and sinks slowly at first, then falls faster as drag removes speed. Rockets have gravity and drag but no wing lift. Take over a vehicle with `Tab` and press `X` to cut or restart its engine.

### “What happens when the target rotates?”

A naive fixed-width calculation is biased. ZENITH projects all eight corners of the known 3D dimensions using a pose estimate and fits both horizontal and vertical spans.

### “Where are YOLO or DINO?”

The current milestone uses a deterministic synthetic detector with the same bounding-box output a generic detector would provide. Identity comes from signal lookup, as required by the project idea. YOLO/DINO remain planned adapters; the camera, tracking, guidance, UI, and verification layers do not depend on a specific detector.

### “Are the vehicles still boxes?”

No. The renderer now uses original low-poly meshes with rotors, arms, fuselages, wings, fins, nose cones, and exhaust sections. They remain lightweight enough for the custom renderer, and `mesh_id` provides the future boundary for imported Blender or glTF assets.

### “Why no instant boosters?”

The prototype uses continuous acceleration, drag, and airbrake forces because they are easier to verify physically. A booster would use the same acceleration command interface but with a time-limited higher bound.

### “Can you control both vehicles?”

Yes. `Tab` cycles from autonomy to our interceptor, then to the target, then back to autonomy. `W/S` requests forward/reverse or braking, `A/D` turns, `Q/E` changes altitude, `Shift` requests full authority, and `Ctrl` uses an available airbrake. The colored authority row proves that a multirotor, fixed wing, and rocket do not receive the same capabilities. The player command still passes through the exact propulsion physics used by autonomous guidance.

## A clean live-demo sequence

1. Start `run_zenith.bat`.
2. Select `TALON-R`, `FALCON-X1`, `EVASIVE MANEUVERS`, and 1920×1080.
3. Start and wait for signal confirmation.
4. Press `V` once for chase view, hold the right mouse button to look around, and use `Shift` while captured to show camera roll. Release it and press `C` to center.
5. Press `F3` for spectator view. Point out that the target, every oval center/border, and every amber unchanged-path dot occupy the same oblique plane. Red now means the oval fails the required four-point rule even if only one point is unreachable.
6. Press `Tab` for our-drone control. Show the guidance advisory beside your own command, then use `W/A/Q` with `Shift`. Press `X` to cut the engine and show gravity/lift, then restart it. Press `Tab` again and control the target; point out the chase camera and authority HUD. Press `Tab` a third time to restore autonomy.
7. Press `F1`; show the oval and aerodynamics information pages. Close it and press `F2` to discuss range degradation and resolution.
8. Close analysis with `F2`, return onboard with `V`, then press `O` to prove guidance loss. Point out that search follows the last image motion while holding altitude and staying near the horizon. Press `O` again to demonstrate genuine reacquisition.
9. Press `-` for slow motion and let the collision complete.
10. Start a new run with `SKYFALL-R1` and `ROCKET ATTACK`. Take over the rocket to show that reverse, airbrake, and wing lift are unavailable.
11. Press `F5` to export the proof data.
