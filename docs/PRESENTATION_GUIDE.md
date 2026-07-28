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
- every oval lies exactly in the plane perpendicular to our current camera view;
- the border is inflated from propulsion-specific acceleration support, so the target's projected position cannot escape it under the stated limits;
- the four extreme points are calculated separately and tested for our reachability;
- green means our drone can arrive before the target; red means it cannot;
- if all four extremes are reachable, the center is a safe guidance objective;
- if no complete oval works, the system follows the unchanged-trajectory prediction;
- near contact, large ovals are ignored and terminal pursuit matches the target's motion.

### 4. Prove it is camera-driven — 60 seconds

Point out:

- identity begins as `UNKNOWN / QUERYING`;
- pixel size, focal length, estimated range, and uncertainty are visible;
- `TRUE / ERROR` is labeled verification-only;
- the exact target coordinate never enters the guidance function;
- pressing `E` exports the camera estimate and truth comparison for every sampled frame.

Press `O` once. Point out that the detection brackets, metric readout, ovals, and maneuver guidance all disappear immediately. The gimbal scans from the last seen bearing without using target truth. Press `O` again; only a new image detection can reacquire lock and restart guidance.

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

Press `A`. Explain that range error grows inversely with pixel count. Show the minimum-resolution table, which changes with the selected target size. Finish with the successful collision, explosion, and falling physics.

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

### “What happens when the target rotates?”

A naive fixed-width calculation is biased. ZENITH projects all eight corners of the known 3D dimensions using a pose estimate and fits both horizontal and vertical spans.

### “Where are YOLO or DINO?”

The current milestone uses a deterministic synthetic detector with the same bounding-box output a generic detector would provide. Identity comes from signal lookup, as required by the project idea. YOLO/DINO remain planned adapters; the camera, tracking, guidance, UI, and verification layers do not depend on a specific detector.

### “Are the vehicles still boxes?”

No. The renderer now uses original low-poly meshes with rotors, arms, fuselages, wings, fins, nose cones, and exhaust sections. They remain lightweight enough for the custom renderer, and `mesh_id` provides the future boundary for imported Blender or glTF assets.

### “Why no instant boosters?”

The prototype uses continuous acceleration, drag, and airbrake forces because they are easier to verify physically. A booster would use the same acceleration command interface but with a time-limited higher bound.

## A clean live-demo sequence

1. Start `run_zenith.bat`.
2. Select `TALON-R`, `FALCON-X1`, `EVASIVE MANEUVERS`, and 1920×1080.
3. Start and wait for signal confirmation.
4. Press `V` once for chase view, then again for tactical overview.
5. Press `A`; discuss range degradation and resolution.
6. Close analysis with `A`, return onboard with `V`, then press `O` to prove guidance loss and `O` again to demonstrate reacquisition.
7. Press `-` for slow motion and let the collision complete.
8. Start a new run with `SKYFALL-R1` and `ROCKET ATTACK`.
9. Press `E` to export the proof data.
