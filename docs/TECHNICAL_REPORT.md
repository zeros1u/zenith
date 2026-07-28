# ZENITH technical report

## 1. Purpose and assumptions

ZENITH is a proof-of-concept onboard software system that tells an interceptor which maneuver to execute using one monocular camera. Target model information is given by the exercise and is represented operationally as a signal lookup that starts only after a generic visual drone detection.

The guidance algorithm does not read simulated target coordinates. Simulation truth has three isolated uses:

1. creating the synthetic camera observation;
2. detecting physical contact and applying crash physics;
3. producing the marked `TRUE / ERROR` value and exported verification data.

Our interceptor's position and velocity are propagated from its own commanded acceleration, which represents normal onboard inertial/state estimation. No external target range sensor is modeled.

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

The interactive renderer also has an independent presentation-camera offset. Right-button drag changes presentation yaw/pitch, `Shift` plus right-button drag changes screen roll, and `C` recenters it. These offsets never modify `camera_forward`, detections, tracking, or guidance; they only allow the already-calculated 3D result to be inspected from another angle.

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

At 1920 horizontal pixels and 75° HFOV, `f ≈ 1251.1 px`. For the 0.70 m-wide FALCON-X1:

| Distance | Nominal apparent width | ±0.5 px relative error |
|---:|---:|---:|
| 100 m | 8.76 px | ±5.7% |
| 250 m | 3.50 px | ±14.3% |
| 500 m | 1.75 px | ±28.5% |
| 1000 m | 0.88 px | ±57.1% |

This is visible in the application's `A` analysis screen and in the exported telemetry. A filtered track is essential because direct frame-to-frame differentiation would turn this quantization noise into unusable speed estimates.

## 5. Minimum camera resolution

Rearranging the focal and pinhole equations gives the horizontal sensor resolution needed for a target to occupy at least `p_min` pixels:

```text
N_min = ceil(2 p_min Z tan(θ/2) / S)
```

For `S = 0.70 m`, `θ = 75°`, and a 12 px generic-detection requirement:

| Engagement distance | Minimum horizontal resolution |
|---:|---:|
| 100 m | 2,631 px |
| 250 m | 6,578 px |
| 500 m | 13,155 px |
| 1000 m | 26,309 px |

These are not arbitrary labels: the application recalculates the table for whichever target model is selected. Narrower optics would reduce the resolution requirement but also reduce the searchable field of view.

## 6. Detection and signal sequence

The implemented state machine is:

1. `SEARCHING`
2. a generic drone with at least 4 px apparent span, or a high-contrast rocket with at least 3 px, becomes `VISUAL LOCK / SIGNAL QUERY`
3. a simulated lookup resolves the known target specification after 0.65 s for drones or 0.42 s for fast incoming rockets
4. the range estimator, metric track, and oval guidance activate
5. any later loss becomes `TARGET LOST / SEARCHING`, clears guidance immediately, and starts an expanding gimbal scan from the last visual bearing
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

This reduces range-quantization jitter without secretly substituting ground truth. A track is guidance-valid only while the image detector has visual lock. If detection is lost, range and guidance are invalidated immediately and the gimbal performs an expanding search around the last observed bearing. Reacquisition must come from a new visible image detection; after a meaningful gap, the metric tracker is restarted instead of pretending its stale prediction is current.

## 8. Prediction ovals

Prediction horizons are `1, 2, 3, and 5 seconds`.

For every horizon, the target's unchanged-velocity point is:

```text
P₀(t) = P + Vt
```

The oval plane passes through this future region and its normal is exactly the current camera optical axis. Therefore every border point `Q` satisfies:

```text
(Q - center) · camera_forward = 0
```

Propulsion limits define acceleration support in the camera-plane X and Y directions. Multirotors use a bounded acceleration ellipsoid with separate horizontal and vertical authority. Wings use forward thrust, bounded braking, and lateral aerodynamic acceleration. Rockets have forward thrust and limited lateral steering, but no reverse engine thrust.

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

The complete border communicates the four reachability tests: green means `4/4`, amber means `1–3/4`, and red means `0/4`. All active prediction horizons remain drawn regardless of reachability. Ovals are hidden only when visual lock is invalid and the system therefore has no current guidance-quality observation.

## 9. Reachability and maneuver choice

For every target extreme and horizon, ZENITH computes whether our interceptor can arrive before the target:

```text
distance_to_point ≤ maximum_distance(v_along, max_accel, max_speed, horizon)
```

The maximum-distance calculation accelerates to maximum speed and then cruises. For wings and rockets, the test also subtracts a turn-time penalty derived from current heading, lateral acceleration, and maximum turn rate. A small reserve accounts for drag and path curvature.

The decision order follows the project idea:

1. test the 5 s oval, then 3 s, 2 s, and 1 s;
2. select the center of the largest oval whose four extremes are all reachable;
3. if no complete oval is reachable, aim at the unchanged-trajectory point in the 1 s oval;
4. at short range or low time-to-contact, stop using the large ovals and enter terminal pursuit.

Terminal pursuit solves the constant-velocity intercept equation. Against a co-directional drone it controls closing speed while matching target lateral velocity. Against an incoming rocket it keeps the interceptor nose-on instead of asking a fixed-wing craft to reverse and velocity-match the threat.

## 10. Physics

Physics advances at a fixed 60 ticks per simulated second. Guidance supplies a requested acceleration, but the flight-model layer decides what force the vehicle can actually produce:

- multirotor and vectored-VTOL craft slew and tilt a thrust vector inside a bounded cone;
- fixed-wing craft add engine thrust only along the nose and turn through bounded lateral aerodynamic acceleration;
- rockets add forward thrust only, have no reverse thrust, and steer at their smaller lateral/turn-rate limit.

```text
velocity(t+Δt) = clamp(velocity + acceleration Δt, max_speed)
position(t+Δt) = position + velocity Δt
```

Actual propulsion acceleration, passive quadratic drag, model-specific airbrakes, maximum speed, and crash gravity are continuous. The UI reports propulsion class and engine output. There are no instantaneous boosters in this milestone. The braking scenario activates a sustained airbrake acceleration rather than deleting a percentage of speed in one frame.

Collision uses the closest separation across the entire relative-motion segment for each tick, preventing fast drones from tunneling through each other between frames. Following impact, both vehicles tumble and fall under gravity.

## 11. Robustness evidence

Six deterministic behaviors are included:

- steady flight;
- lateral/vertical weave;
- piecewise evasive maneuvers;
- repeated airbraking;
- continuous target rotation.
- an incoming rocket attack.

`verify_prototype.py` executes all six and reports identification, hit time, minimum separation, and range error. The rocket-attack row uses the SKYFALL-R1 target. The current checked-in verification report is `artifacts/verification.csv`.

Rotation is handled by known-model pose-compensated physical spans. Maneuvers are handled by the alpha-beta track, model acceleration limits, four-extreme ovals, and terminal velocity matching.

## 12. Known prototype boundaries

- Five original low-poly drone meshes and two original rocket meshes are bundled. They are recognizable real-time silhouettes rather than photorealistic Blender assets.
- The generic detector backend deterministically emits the synthetic camera bounding box. It represents the output contract of YOLO/DINO, not a trained neural network.
- The signal lookup is simulated; it is a state and data-flow demonstration, not radio hardware.
- The current oval is a conservative acceleration-support envelope in the 2D camera plane. A production system would also propagate range-axis uncertainty, latency, pose confidence, wind, actuator dynamics, and safety constraints.
- A monocular camera cannot recover absolute scale for an unknown object. Metric values correctly remain unavailable until known target dimensions are resolved.

These boundaries are deliberately visible rather than hidden, so the project can be presented as a working and testable software proof-of-concept.
