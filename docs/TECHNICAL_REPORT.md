# ZENITH technical report

## 1. Purpose and assumptions

ZENITH is a proof-of-concept onboard software system that tells an interceptor which maneuver to execute using one monocular camera. Target model information is given by the exercise and is represented operationally as a signal lookup that starts only after a generic visual drone detection.

The defense guidance algorithm does not read simulated target coordinates. Simulation truth has three isolated defense-side uses:

1. creating the synthetic camera observation;
2. detecting physical contact and applying crash physics;
3. producing the marked `TRUE / ERROR` value and exported verification data.

Our interceptor's position and velocity are propagated from its own commanded acceleration, which represents normal onboard inertial/state estimation. No external target range sensor is modeled.

The optional `TRICKY AI` adversary is an explicit test-harness exception to this truth boundary. That scenario assumes the enemy knows about the interceptor, so only the target's own autopilot reads true relative range and closing speed to choose a maneuver. It cannot write to the detector, target track, oval solver, or interceptor command.

## 2. Coordinate system

Calculations use a camera-relative orthonormal basis:

- `X`: right is positive and left is negative;
- `Y`: up is positive and down is negative;
- `Z`: positive along the camera's optical axis toward the target.

Let the camera forward unit vector be **f**. Then:

1. **r** = normalize(world-up × **f**)
2. **u** = **f** × **r**
3. for a world-relative target vector **q**:
   - `Dx = q · r`
   - `Dy = q · u`
   - `Dz = q · f`

The target and interceptor define the line of sight. A plane perpendicular to that line is spanned by **r** and **u**, allowing the four oval-extreme calculations to be treated as a 2D reachability problem while the drones remain in a 3D world.

The interactive renderer also has an independent presentation-camera offset. Right-button drag changes presentation yaw/pitch, `Shift` plus right-button drag changes screen roll, the mouse wheel changes a bounded 24°–100° presentation field of view, and `C` recenters and resets zoom. These offsets never modify the configured 90° sensor, `camera_forward`, detections, tracking, or guidance; they only allow the already-calculated 3D result to be inspected from another angle.

## 3. Monocular pinhole range

For a known physical target span `S`, focal length `f` expressed in pixels, and measured apparent span `p`:

```text
Z = f S / p
```

For horizontal image width `N` and horizontal field of view `θ`:

```text
f = N / (2 tan(θ/2))
```

The bounding-box center also gives horizontal and vertical bearing:

```text
bearing_x = atan((pixel_x - center_x) / f)
bearing_y = atan((center_y - pixel_y) / f)
```

Combining bearing with estimated range reconstructs the relative 3D target position.

### Rotation compensation

A width-only estimate fails when a rectangular target rotates. For box dimensions `W × H × L`, the approximate horizontal silhouette span under yaw `ψ` is:

```text
Sx(ψ) = |W cos ψ| + |L sin ψ|
```

ZENITH uses all eight known-model corners and a visual pose estimate to calculate horizontal and vertical physical spans. It first evaluates `Z = fS/p` in both axes, then fits the complete projected 3D box. The full fit matters at terminal distance because different corners no longer have equal depth.

The display retains a naive width-only result internally, allowing the compensated method to be compared later when realistic models and a pose detector are connected.

## 4. Why range degrades when the target shrinks

Differentiating the pinhole equation gives:

```text
dZ/dp = -fS/p²
```

For a pixel-span error `δp`:

```text
|δZ| ≈ (fS/p²)|δp|
|δZ|/Z ≈ |δp|/p
```

Therefore, the same half-pixel edge error is minor for a large target and severe for a small one.

| Target span | Approx. relative range error for ±0.5 px |
|---:|---:|
| 4 px | ±12.5% |
| 10 px | ±5.0% |
| 20 px | ±2.5% |
| 50 px | ±1.0% |
| 100 px | ±0.5% |

At 1920 horizontal pixels and 90° HFOV, `f = 960 px`. For the 0.70 m-wide FALCON-X1:

| Distance | Nominal apparent width | ±0.5 px relative error |
|---:|---:|---:|
| 100 m | 6.72 px | ±7.4% |
| 250 m | 2.69 px | ±18.6% |
| 500 m | 1.34 px | ±37.2% |
| 1000 m | 0.67 px | ±74.4% |

This is visible in the application's `F2` analysis screen and in the exported telemetry. A filtered track is essential because direct frame-to-frame differentiation would turn this quantization noise into unusable speed estimates.

## 5. Minimum camera resolution

Rearranging the focal and pinhole equations gives the horizontal sensor resolution needed for a target to occupy at least `p_min` pixels:

```text
N_min = ceil(2 p_min Z tan(θ/2) / S)
```

For `S = 0.70 m`, `θ = 90°`, and a conservative 12 px analysis requirement:

| Engagement distance | Minimum horizontal resolution |
|---:|---:|
| 100 m | 3,429 px |
| 250 m | 8,572 px |
| 500 m | 17,143 px |
| 1000 m | 34,286 px |

These are not arbitrary labels: the application recalculates the table for whichever target model is selected. Narrower optics would reduce the resolution requirement but also reduce the searchable field of view.

## 6. Detection and signal sequence

The implemented state machine is:

1. `SEARCHING`
2. a generic aerial object with at least 3 px apparent span becomes `VISUAL LOCK / SIGNAL QUERY`
3. a simulated lookup resolves the known target specification after 0.65 s for drones or 0.42 s for fast incoming rockets
4. the range estimator, metric track, and oval guidance activate
5. any later loss becomes `TARGET LOST / SEARCHING`, clears guidance immediately, and extrapolates the filtered image-plane bearing rate for no more than 1.5 seconds
6. only a newly visible image detection can reacquire the target and reactivate guidance

Before identification, the system has bearing, pixel size, and angular motion. After target dimensions arrive, it converts those observations to metres and metres per second. This separation makes it clear which calculations depend on known model data.

## 7. Filtered image track

An alpha-beta filter predicts target position from the previous camera-derived velocity and corrects that prediction with the new pinhole measurement:

```text
prediction = position + velocity Δt
residual   = measurement - prediction
position   = prediction + α residual
velocity   = velocity + (β/Δt) residual
```

This reduces range-quantization jitter without secretly substituting ground truth. A track is guidance-valid only while the image detector has visual lock. The sensor optical axis is recomputed from the interceptor airframe orientation every tick; there is no independent target-following gimbal. While detections are valid, ZENITH separately filters horizontal and vertical image-bearing rates. If detection is lost, range and guidance are invalidated immediately. The last image motion is extrapolated for at most 1.5 seconds, after which autonomy requests an expanding horizontal body scan around that predicted direction. Its requested world elevation remains inside `-35 to +35 degrees`. Reacquisition requires the body-mounted camera to see a new visible detection; after a meaningful gap, the metric tracker is restarted instead of pretending its stale prediction is current.

## 8. Prediction ovals

Prediction horizons are `1, 2, 3, and 5 seconds`.

For every 60 Hz observation, the camera pose is frozen and each possible future point is projected into that recorded image, then back-projected onto the current estimated target-depth plane:

```text
N       = camera optical-axis unit vector
D       = (P - camera) · N
Qplane  = camera + (Q - camera) × D / ((Q - camera) · N)
```

The one oval plane passes through the current target `P`, and its normal is exactly the current camera optical axis. Therefore the target, every oval center, every extreme, and every projected unchanged-trajectory point have the same camera depth:

```text
(Q - P) · N = 0
```

Propulsion limits define acceleration support in camera X/Y. The radii are enlarged by bounded track position/velocity uncertainty and by the closest permitted future depth, so approach cannot create a larger pixel displacement than the border. If the possible set crosses the frozen camera plane, the horizon is marked `UNBOUNDED / CAMERA CROSSING`.

Positive and negative support are calculated separately in four camera-plane directions:

- positive X;
- negative X;
- positive Y;
- negative Y.

Those component bounds define a guaranteed rectangle for the projected future position. Directional vehicles use an ellipse whose semiaxes are multiplied by `√2`, which circumscribes that rectangle. The multirotor acceleration ellipsoid is aligned with the camera-plane horizontal/projected-vertical basis and needs no inflation. Thus the border is a conservative containment claim: any allowed projected maneuver remains inside it. Speed limits can only make the actual reachable set smaller.

```text
center offset X = (support(+X) - support(-X)) t² / 4
radius X        = (support(+X) + support(-X)) t² / 4 × containment factor
```

The orange dots show the unchanged-velocity trajectory inside the smallest oval.

Four large cardinal points remain visible, but their connecting diamond does not cover diagonal ellipse points. The actual decision evaluates all 96 directions that render the border. Each passing border segment is green and each failing segment is red; only a complete green `96/96` loop is selectable. Grey reports an invalid/unbounded horizon. Cardinal colors remain a readable summary and amber remains the unchanged-motion trajectory.

## 9. Reachability and maneuver choice

For every rendered target-edge direction and horizon, ZENITH computes whether our interceptor can arrive before the target:

```text
distance_to_point ≤ maximum_distance(v_along, max_accel, max_speed, horizon)
```

The maximum-distance calculation accelerates to maximum speed and then cruises. For wings and rockets, the test also subtracts a turn-time penalty derived from current heading, lateral acceleration, and maximum turn rate. A small reserve accounts for drag and path curvature.

The decision order follows the project idea:

1. test the 5 s oval, then 3 s, 2 s, and 1 s;
2. select the largest oval whose complete edge is reachable;
3. confidence-weight its aim toward measured likely motion, capped at 65% normalized radius;
4. if none pass, report `NO GUARANTEED OVAL` and use the unchanged-motion 1 s point;
5. when one-second reach enters the smallest region, enter terminal collision lead; `0.75 s` TTC is the safety fallback.

Terminal pursuit solves the constant-velocity intercept equation. Against a co-directional drone it controls closing speed while matching target lateral velocity. Against an incoming rocket it keeps the interceptor nose-on instead of asking a fixed-wing craft to reverse and velocity-match the threat.

## 10. Physics

Physics advances at a fixed 60 ticks per simulated second. Guidance supplies a requested acceleration, but the flight-model layer decides what force the vehicle can actually produce:

- gravity supplies `9.81 m/s²` downward acceleration to every airborne class on every tick;
- multirotor and vectored-VTOL craft slew and tilt a powered thrust vector inside a bounded cone, explicitly spending thrust to counter gravity;
- fixed-wing craft add engine thrust only along the nose, turn through bounded lateral aerodynamic acceleration, and generate airflow-dependent lift perpendicular to the flight path;
- rockets ignite a finite full-thrust booster, have no reverse/throttle/restart/wing lift, and steer only while separate RCS propellant remains.

```text
velocity(t+Δt) = clamp(velocity + acceleration Δt, max_speed)
position(t+Δt) = position + velocity Δt
```

Actual propulsion acceleration, gravity, passive quadratic drag, model-specific airbrakes, maximum speed, and ground contact are continuous. Wing lift uses an explicit model stall speed and lift-efficiency factor:

```text
airflow factor = clamp((speed / stall_speed)², 0, 1)
lift            = 9.81 × lift_efficiency × airflow factor
```

Lift points perpendicular to the flight path toward world-up. A horizontal wing therefore sinks more slowly when unpowered, lift weakens below stall, and lift vanishes in a vertical fall. This is a transparent presentation model rather than computational fluid dynamics. `X` cuts/restarts controllable drone engines. A solid rocket correctly rejects that action: `SKYFALL-R1` burns for 4 s with 8 s RCS, while `LANCE-M2` burns for 6 s with 12 s RCS. Both expose `BURNING/BURNOUT` and remaining timers.

Collision uses the closest separation across the entire relative-motion segment for each tick, preventing fast drones from tunneling through each other between frames. Following impact, both vehicles tumble and fall under gravity.

## 11. Player takeover and control authority

`Tab` cycles between autonomous guidance, player control of our interceptor, player control of the target, and autonomy again. Entering a player mode copies the vehicle's current heading and altitude into the assisted controller, so authority transfer does not create an artificial pose or speed jump. The presentation camera follows the controlled vehicle, while the synthetic defense sensor remains independent.

The player does not write position or velocity directly. `W/S`, `A/D`, and `Q/E` create forward, turn, and vertical requests; `Shift` requests the full available bound, `Ctrl` activates a supported airbrake, and `X` cuts or restarts the selected engine. Those requests pass through the normal 60 Hz flight model:

- multirotors may command forward or reverse acceleration and an explicit body heading, but their thrust vector must still slew and tilt;
- fixed-wing vehicles may decelerate but never fly backward under engine power, and lateral input is turn-rate limited;
- rockets automatically use their remaining main burn, accept only bounded RCS steering, and reject reverse, throttle, airbrake, cut, and restart requests;
- a two-metre altitude floor blocks commanded descent into the ground.

The compact authority HUD labels direct capability green, limited turn/brake/glide capability amber, and unavailable capability red. Cutting an engine immediately updates those authority colors. It also reports active keys, requested versus achieved acceleration, requested/actual engine output, engine state, and airbrake state. During player control of our interceptor, autonomous oval guidance continues only as a visible advisory. During player control of the target, our interceptor remains autonomous. Mouse capture suppresses vehicle keys so `Shift + right mouse` rolls only the presentation camera.

`F3` selects an independent spectator camera. The fixed sensor boresight is rendered only onboard; the guidance diamond is a projected world point. Independently clickable windows expose the core panels, presentation settings, estimated-track minimap, and truth-isolated verification. `G` records a +2 s oval and its old virtual camera; truth is projected into that saved frame only at the audit time.

During target loss, player control of our interceptor overrides the automatic body-search turn. Because the camera is fixed to the body, the player must physically turn the interceptor to search. Manual flight cannot manufacture sensor lock.

## 12. Robustness evidence

Seven deterministic behaviors are included:

- steady flight;
- lateral/vertical weave;
- piecewise evasive maneuvers;
- repeated airbraking;
- continuous target rotation;
- threat-aware tricky AI with speed shifts, lateral/vertical breaks, brake traps, and escape acceleration;
- an incoming rocket attack.

`verify_prototype.py` executes all seven and reports identification, hit time, minimum separation, and range error. The tricky row uses SMART EVADER deterministically through the same physically bounded integration as every other target; the rocket-attack row uses SKYFALL-R1. The current checked-in verification report is `artifacts/verification.csv`.

Rotation is handled by known-model pose-compensated physical spans. Maneuvers are handled by the alpha-beta track, model acceleration limits, four-extreme ovals, and terminal velocity matching.

## 13. Known prototype boundaries

- Six original low-poly drone meshes and two original rocket meshes are bundled. The SMART EVADER uses a saucer body, pointed +Z nose, stabilizers, keel, and twin rear drives so its direction remains unambiguous. They are recognizable real-time silhouettes rather than photorealistic Blender assets.
- The generic detector backend deterministically emits the synthetic camera bounding box. It represents the output contract of YOLO/DINO, not a trained neural network.
- The signal lookup is simulated; it is a state and data-flow demonstration, not radio hardware.
- The current oval is a conservative acceleration-support envelope in the 2D camera plane. A production system would also propagate range-axis uncertainty, latency, pose confidence, wind, actuator dynamics, and safety constraints.
- The lift model is not CFD. Production aerodynamics require measured lift/drag curves, mass, wing area, angle of attack, wind, control-surface dynamics, and flight-test validation.
- A monocular camera cannot recover absolute scale for an unknown object. Metric values correctly remain unavailable until known target dimensions are resolved.

These boundaries are deliberately visible rather than hidden, so the project can be presented as a working and testable software proof-of-concept.
