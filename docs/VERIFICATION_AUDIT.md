# ZENITH requirement and correctness audit

This matrix records what the prototype claims, where it is implemented, and how it is checked. It is intended to be usable during the presentation rather than relying on an unexplained visual effect.

| Requirement or question | Implemented behavior | Verification evidence |
|---|---|---|
| Camera-only target input | Guidance receives the synthetic detector box/bearing, signal-resolved model, pinhole estimate, filtered image track, and our integrated state. Target truth is restricted to image rendering, contact, and the labeled error comparison. | Lock-loss tests prove guidance becomes `None` immediately when the image is occluded and returns only after a new detection. |
| Detect first, then query signal | An unknown visual object is acquired before signal progress starts; identity and metric guidance become available only after the configured delay. | `test_signal_lookup_precedes_identity` |
| Pinhole range | `Z = focal_pixels × known_physical_span / apparent_pixels`, with pose-compensated projected dimensions and pixel uncertainty. | Camera formula, known-box range, and minimum-resolution tests; live `F2` analysis. |
| Camera-relative coordinates | `X` is image left/right, `Y` is image up/down, and `Z` is the optical line toward the target. | Orthonormal camera-basis test and the `CALCULATIONS`/`RELATIVE` panels. |
| Four transparent readout panels | Target model, calculations, our drone, and relative/guidance information remain visible at the bottom of the simulation. | Minimum-window screenshot and information page 1. |
| Real 1/2/3/5-second ovals | Border points use the parametric ellipse `center + a cos(t) X + b sin(t) Y`. | `test_rendered_border_satisfies_ellipse_equation` evaluates `x²/a² + y²/b² = 1` for every rendered sample. |
| Ovals and target share one plane | Line-of-sight displacement is removed from every future projection. The target, all centers, ballistic dots, and all extremes satisfy `(point − target) · camera_forward = 0`. | `test_oval_plane_is_perpendicular_to_camera_view`; spectator screenshot. |
| Target cannot escape the projected border under the model assumptions | Positive/negative propulsion support is calculated independently in both plane axes. Directional envelopes circumscribe the support rectangle; vectored envelopes use their acceleration ellipse. | `test_conservative_envelope_contains_allowed_acceleration_samples` samples hundreds of allowed accelerations. |
| Four reachability points | Every oval stores and tests `+X`, `−X`, `+Y`, and `−Y` extremes. Its center is their coordinate average. | Four-extreme and center-average assertions in the guidance tests. |
| Clear reachability color | Green requires `4/4`. If even one required point fails, the full border is red and labeled `BLOCKED n/4`; each point retains its individual color. | `test_entire_oval_color_reports_four_point_reachability` |
| Does the interceptor follow the oval center? | Yes, exactly when an oval passes `4/4`: the solver checks 5, 3, 2, then 1 seconds and sets `aim_point = selected.center`. If none pass, following the center would violate the project rule, so it uses the 1-second unchanged path. Near contact it uses terminal pursuit. | `test_guidance_aims_at_largest_fully_reachable_oval_center` |
| No impossible propulsion | Rotorcraft slew/tilt thrust; fixed wings use axial engine force and bounded aerodynamic turning; rockets reject reverse thrust and airbrakes. | Directional-turn, axial-thrust, rocket-reverse, and manual-authority tests. |
| Gravity for everyone | `9.81 m/s²` downward acceleration is present on every active and crashed flight class. Powered hover is produced by thrust, not by deleting gravity. | `test_every_unpowered_vehicle_class_receives_gravity`; `test_powered_multirotor_hovers_but_engine_cut_falls` |
| Wing glide instead of brick-like fall | Wing lift is perpendicular to the flight path, depends on airspeed/stall speed, and is lower in a vertical fall. Drag eventually removes lift-producing speed. Rockets have no wing lift. | Wing-versus-rocket fall and below-stall lift tests; live information page 3. |
| Demonstrable engine failure | `X` cuts/restarts the player-selected engine. The HUD changes authority and shows requested/actual output plus `ON`/`CUT`. | Engine-toggle test and gravity/aerodynamics information page. |
| Controllable vehicles | `Tab` cycles autonomy, our drone, target, and autonomy. All drone target models and both rockets use the same assisted input boundary. | Manual mode, multirotor heading, rocket restriction, altitude-floor, and advisory override tests. |
| Target loss is honest | Range/guidance/ovals are invalidated. Image bearing rate is extrapolated for at most 1.5 seconds, then a horizon-limited scan widens. Only the detector can reacquire. | Occlusion/reacquisition, horizon-limit, and manual search-override tests. |
| Spectator mode | `F3` selects an independent tactical camera attached to neither pilot; RMB free-look remains presentation-only. | Spectator camera and free-look isolation tests. |
| Full explanation inside the program | `F1` opens four pages covering inputs, axes, panels, ovals, colors, guidance, live forces, controls, target loss, cameras, and limitations. | All four pages render at the minimum 1050×700 window in the automated suite. |
| End-to-end interception | Steady, weave, evasive, braking, rotating, and rocket-attack runs must identify and physically contact the threat. | `verify_prototype.py --duration 30` and the complete scenario/catalogue tests. |

## Aerodynamic boundary

The gravity term is physical and explicit. The lift model is intentionally a transparent proof-of-concept rather than CFD: it uses current airspeed, flight-path direction, a per-model stall speed, and a lift-efficiency factor. Production replacement requires measured lift/drag curves, mass, wing area, angle of attack, atmospheric wind, control-surface response, actuator limits, and flight-test validation.

## Decision-rule clarification

Seeing a red oval while the interceptor follows an amber dot is correct, not a color failure: red says the four-point condition failed, so that oval center is not an allowed choice. The amber dots show the target's projected unchanged-velocity path. A green `SELECTED` oval is the state in which the interceptor is commanded toward that exact oval center.
