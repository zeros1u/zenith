# ZENITH requirement and correctness audit

This matrix states exactly what the prototype claims and how that claim is checked.

| Requirement or question | Implemented behavior | Verification evidence |
|---|---|---|
| Camera-only target input | Guidance receives detector bearing/box, the signal-resolved model, pinhole estimate, filtered track, uncertainty, and our integrated state. | Lock-loss tests prove guidance disappears immediately when the image is occluded. |
| Detect, then query signals | Identity and metric guidance become available only after visual acquisition and the configured lookup delay. | `test_signal_lookup_precedes_identity` |
| Estimated enemy coordinates | The Target panel shows camera-derived world XYZ. Truth is available only inside the expanded verification window. | Minimum-window widget test and saved-frame verification tests. |
| Frozen-now oval definition | At 60 Hz, the current sensor pose is treated as fixed and the target's future pixel region is projected into that frame. | Perspective projection and common-plane guidance tests. |
| Real ellipses on one target plane | The rendered border uses `center + a cos(t) X + b sin(t) Y` and is back-projected onto the plane through the estimated target, perpendicular to the sensor. | Ellipse-equation and target-plane tests. |
| Enemy cannot escape under declared limits | Propulsion support, closest permitted perspective depth, and bounded position/velocity uncertainty form a conservative outer border. Camera-plane crossings are reported as unbounded. | Allowed-acceleration containment, camera-crossing, and five two-second scenario checks. |
| Why four dots are not enough | Four large cardinal points remain visible, but diagonal points lie outside their connecting diamond. All 96 rendered directions decide the border. | `test_entire_oval_color_reports_complete_edge_reachability` deliberately fails a diagonal while keeping `4/4`. |
| Reachability color | Green requires `96/96`; red means at least one rendered direction fails; grey means no finite valid bound. | Complete-edge color and unbounded-prediction tests. |
| Weighted largest-oval guidance | The largest green horizon is selected. Its aim is confidence-weighted toward measured likely motion and limited to 65% of the ellipse. | Weighted-oval selection and normalized-radius assertions. |
| Terminal guidance | The normal rule changes to camera-derived collision lead when our one-second reach enters the smallest region; `0.75 s` TTC is the safety fallback. Solid rockets use proportional navigation throughout. | Default and catalogue interception tests. |
| Gravity and aerodynamics | Gravity is continuously explicit. Rotorcraft spend thrust to hover; wings create speed/stall-dependent lift; rockets have no wing lift. | Gravity, hover, glide, and stall tests. |
| Literal one-shot rockets | Both rocket models can be our interceptor or the target. They ignite automatically, burn for a finite time, cannot throttle/cut/restart/reverse, and use separately limited RCS. | Booster burnout, RCS supply, reverse rejection, both-side catalogue, and rocket interception tests. |
| Honest target loss | Range, guidance, and live ovals become invalid. Search uses only previous image bearing/rate and only a new detection reacquires. | Occlusion, reacquisition, horizon, and manual-search tests. |
| Onboard boresight versus guidance aim | The fixed cross is labeled `SENSOR BORESIGHT` and exists only onboard. A separate diamond marks the world-space guidance aim. | Onboard/spectator reticle rendering test. |
| Expandable UI and minimap | Core panels start open; calculations, minimap, settings, and verification start collapsed. The map uses estimated X/Z, the sensor FOV, and altitude-coded markers. | Collapsible widget rendering at 1050×700. |
| Two-second presentation proof | `G` records the old virtual camera and +2 s oval. The inset keeps that camera fixed while the live vehicle/camera moves, then checks truth only at saved `T+2`. | Recorded-camera immutability and all-drone-scenario containment tests. |
| End-to-end interception | All six scenarios identify and physically contact the assigned threat; every drone target and both incoming rockets are covered. | `verify_prototype.py --duration 30` and the complete scenario/catalogue suite. |

## Claim boundary

“The enemy cannot escape” means it cannot escape the displayed outer region while obeying the simulator's published propulsion model and configured sensor-error bound. It is not a claim about unmodeled wind, damage, deceptive emissions, actuator faults, or real hardware.

The four cardinal dots are a presentation aid. The complete 96-direction border decides red/green. A red oval is never selected even if its cardinal summary says `4/4`; guidance then reports `NO GUARANTEED OVAL` and uses the unchanged-motion fallback until a complete green solution or terminal condition exists.

The verification inset and truth coordinates are simulation-only audit tools. Neither can feed the detector, range estimate, target track, oval construction, or maneuver command.
