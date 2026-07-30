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

- each oval is the future pixel area seen from the current camera pose if that pose were frozen now;
- the pixel ellipse is back-projected onto one plane through the estimated target, perpendicular to the sensor;
- propulsion support, perspective, and track uncertainty make the border an outer bound under the stated model;
- four large points remain the cardinal explanation, while all 96 rendered edge directions are tested;
- green edge segments pass our reachability check, red segments fail, a mixed edge is only partially reachable, and grey reports an unbounded camera crossing;
- only a completely green loop receives a faint green interior tint; red never fills the center because an unreachable outer edge does not make every interior point unreachable;
- an unreachable oval remains visible and red instead of disappearing;
- the largest green oval uses a confidence-weighted likely-motion point limited to 65% of its radius;
- if no complete oval works, the system follows the unchanged-trajectory prediction;
- reaching the one-second region activates camera-derived terminal collision lead.
- with several contacts, only equal horizons are paired: 2-second with 2-second, never 2-second with 5-second;
- cyan marks the shared image-plane intersection of one qualifying two-target pair, and the white diamond marks its centroid;
- once our reach enters either member's smaller nested oval, the state becomes `COMMITTED` and normal single-target interception finishes the maneuver.

### 4. Prove it is camera-driven — 60 seconds

Point out:

- identity begins as `UNKNOWN / QUERYING`;
- pixel size, focal length, estimated range, and uncertainty are visible;
- estimated target XYZ is visible while truth/error stays in the expanded verification window;
- the exact target coordinate never enters the guidance function;
- pressing `F5` exports the camera estimate and truth comparison for every sampled frame.

Press `O` once. Point out that the detection brackets, metric readout, ovals, and maneuver guidance all disappear immediately. The fixed camera cannot follow the missing target. Autonomy turns the interceptor body using the last image bearing and bearing rate, without target truth. Press `O` again; only a target that physically re-enters the nose-mounted camera FOV can reacquire lock and restart guidance.

Hold the right mouse button to capture the pointer and inspect the scene without being stopped by a window edge. Hold `Shift` while moving the mouse to roll the horizon and view the prediction plane obliquely. Use the wheel to zoom. Explain that these are presentation-camera changes: they do not rotate or zoom the actual 90° sensor or change guidance, and vehicle keys are suppressed during capture. Release the right button to restore the pointer, then press `C` to return to the centered, default-zoom presentation.

Press `F3` for the independent spectator/tactical camera. This is the clearest view for proving that the target and all four ovals occupy the same plane. Press `F1` at any time for the four-page explanation of every coordinate, color, force, guidance state, camera, and verification boundary.

Press `M` for the estimated-track X/Z minimap. Its wedge is the actual sensor FOV; the target marker is brighter above the camera and darker below, with a numeric altitude difference. Press `G` to record the current two-second prediction. The verification inset retains the old virtual camera, so the live vehicle and camera may continue moving before the `INSIDE/OUTSIDE` check.

### 5. Demonstrate robustness — 60 seconds

Use `N` to show the selectable behaviors and threats:

- `ENEMY CONTACTS` selects one, two, or three independently detected and tracked intruders; the HUD shows `SHARED T1+T2 / 5s OVERLAP` while aiming between a qualifying pair and later changes to `COMMITTED` after entering a smaller nested oval;
- weave tests continuous side motion;
- evasive tests changing acceleration direction;
- airbrake tests nonlinear speed changes;
- rotating target tests apparent-size changes;
- tricky AI assumes the enemy knows our approach and reacts with physically bounded jinks, vertical breaks, sprints, speed shifts, and brake traps; as a real stress case it can sometimes escape a less favorable matchup;
- rocket attack selects a fast incoming target with rocket-specific physics and status.

The seven-scenario verifier can be shown from a terminal:

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

No. Connecting four cardinal extremes makes a diamond that excludes diagonal parts of an ellipse. ZENITH keeps the four large dots for explanation but tests all 96 rendered edge directions. A deliberately tested diagonal failure remains red even with `CARDINAL 4/4`.

### “Can an aircraft accelerate in any direction in this simulation?”

No. Multirotors must slew and tilt their thrust vector. Fixed-wing engines push only along the nose; turns come from bounded aerodynamic lateral force. Rockets have forward thrust and limited steering but cannot command reverse thrust. The guidance request is always filtered through that vehicle-specific physics layer.

### “Where is gravity, and why does a wing fall differently?”

Gravity is continuously applied at `9.81 m/s²` to every airborne vehicle. A powered multirotor spends upward thrust to cancel it. A wing creates lift perpendicular to its flight path; that lift weakens below stall and vanishes in a vertical fall. Rockets have gravity and drag but no wing lift. Their solid booster ignites automatically, expires permanently, and uses a separately timed RCS supply; `X` correctly cannot cut or restart it.

### “What happens when the target rotates?”

A naive fixed-width calculation is biased. ZENITH projects all eight corners of the known 3D dimensions using a pose estimate and fits both horizontal and vertical spans.

### “Where are YOLO or DINO?”

No trained recognition model or weight file is bundled. The current milestone uses a deterministic synthetic detector/pose adapter with the same floating-point box, vehicle-center keypoint, bearing, confidence, and pose outputs that real models would provide. Subpixel box motion is preserved instead of rounded into unstable whole-pixel range steps. Identity comes from signal lookup, as required by the project idea. The `ImageDetectorAdapter` protocol is the explicit replacement boundary for YOLO or DINO, but actual inference also needs weights, dependencies, and sensor frames; the project does not falsely label those as present.

### “Why were Aegis-Q4 and Smart Evader tilting upward?”

That was a real sign and initialization bug, now covered by regression tests. Both spawn level at half maximum speed with zero vertical velocity. Forward vectored thrust tilts the nose downward; any later climb is a gradual maneuver toward a higher target, not a spawn impulse. Their cameras are rigidly mounted at a published 6° upward cant, and their 24° body-tilt envelope prevents automatic flight from repeatedly throwing the target across the vertical FOV edge. It is a fixed transform and flight constraint, not a truth-following gimbal.

### “How does guidance work with several enemies?”

Every contact is processed separately. At most one pair affects the command. The solver compares equal horizons only and converts both ellipses to the frozen camera's normalized image plane, so different target depths do not invalidate the overlap. A pair qualifies when each center lies inside the other's ellipse and our vehicle can reach the shared centroid by that horizon. It initially steers toward that cyan overlap. Once our reach enters either target's smaller nested oval, the higher camera-derived threat becomes a sticky committed lock. Exact target truth is never used for pairing or commitment.

### “Why can Wraith-S miss even though it is fastest?”

Maximum speed is not the same as interception agility. Wraith-S reaches `82 m/s`, but Talon-R has more lateral acceleration and a higher turn-rate limit. ZENITH now schedules fixed-wing closing speed before terminal range and cuts throttle to use passive drag; it does not invent reverse thrust or automatically deploy an airbrake based on speed.

### “Are the vehicles still boxes?”

No. The renderer now uses original low-poly meshes with rotors, arms, fuselages, wings, fins, nose cones, and exhaust sections. They remain lightweight enough for the custom renderer, and `mesh_id` provides the future boundary for imported Blender or glTF assets.

### “How do the rocket boosters work?”

`SKYFALL-R1` and `LANCE-M2` can be selected as our interceptor or as the target. They ignite at launch, use full thrust for four or six seconds, cannot throttle/reverse/cut/restart, and then coast. A separate RCS timer powers bounded direction changes. The HUD exposes both remaining timers.

### “Can you control both vehicles?”

Yes. `Tab` cycles from autonomy to our interceptor, then to the target, then back to autonomy. `W/S` requests forward/reverse or braking, `A/D` turns, `Q/E` changes altitude, `Shift` requests full authority, and `Ctrl` uses an available airbrake. The colored authority row proves that a multirotor, fixed wing, and rocket do not receive the same capabilities. The player command still passes through the exact propulsion physics used by autonomous guidance.

### “Does the Smart Evader cheat?”

Only its explicitly labeled `TRICKY AI` target autopilot is allowed to know our true range and closing approach, because that test assumes an informed enemy. It chooses its own deterministic maneuver, then obeys the target's normal acceleration, speed, turn, airbrake, drag, and gravity limits. It can sometimes genuinely escape; the standard scenarios remain the guaranteed demonstration cases. The defense camera, tracker, ovals, and guidance receive no truth from it.

### “Does mouse-wheel zoom improve detection?”

No. It changes only the presentation camera used to inspect the scene. The synthetic defense sensor remains at its configured 90° field of view, so zoom cannot acquire a target, preserve lock, change an oval, or improve the range calculation.

## A clean live-demo sequence

1. Start `run_zenith.bat`.
2. Select `TALON-R`, `FALCON-X1`, `EVASIVE MANEUVERS`, three enemy contacts, and 1920×1080.
3. Start and wait for signal confirmation.
4. Press `V` once for chase view, hold the right mouse button to look around, and use `Shift` while captured to show camera roll. Release it and press `C` to center.
5. Scroll to demonstrate presentation zoom, then press `F3` for spectator view and show the common target plane. Green/red edge segments show exactly which of the 96 directions pass, while only a completely green loop may be selected.
6. Press `M` for the estimated-track map, then `G` for the saved-camera two-second check.
7. Press `Tab` for our-vehicle control. Demonstrate drone gravity/lift with `X`, then restore autonomy.
8. Press `F1` for the oval/aerodynamics reference, then `F2` for range degradation.
9. Press `N`, select `SMART EVADER` plus `TRICKY AI`, and point to its live decision in the Target panel. Explain that target awareness is the declared adversary test assumption and remains isolated from defense guidance.
9. Return onboard, press `O` to prove guidance loss, then restore the image and reacquire.
10. Start a new run with `SKYFALL-R1` or `LANCE-M2` as **our interceptor**. Show automatic burn, RCS steering, and burnout; rockets remain available as incoming threats.
11. Press `F5` to export the proof data.
