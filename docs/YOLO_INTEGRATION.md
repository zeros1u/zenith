# ZENITH custom YOLO integration

## What this addition proves

ZENITH can run its perception-to-guidance loop from the output of an actual
trained image model rather than from a mathematical simulator box. The
`YOLO CUSTOM` pipeline is:

```text
rigid onboard camera pose
        |
clean configured-resolution BGR sensor frame
        |
full-FOV + fixed-boresight-crop batched inference
        |
custom Ultralytics YOLO aerial_target model
        |
box + confidence + image-only contact association
        |
simulated post-detection signal lookup
        |
known-size monocular range + alpha-beta track
        |
1/2/3/5-second ovals + maneuver command
```

The model detects one generic class, `aerial_target`. It does not reveal
FALCON-X1, SMART EVADER, a rocket model, dimensions, speed, or maneuver
authority. This preserves the original exercise sequence: see the object,
query its signal, then use the supplied specification.

## Runtime truth boundary

`SensorFrameRenderer.render_bgr()` returns only a BGR pixel array. It does not
return boxes, actor IDs, coordinates, velocity, orientation, or dataset labels.
`UltralyticsYOLODetector` receives that array and returns only model boxes,
class IDs, and confidence values.

The same renderer has a separately named `render_labeled_bgr()` method used
only by the offline dataset generator. Its exact simulator boxes become YOLO
training text files and are never available to runtime inference.

Multiple YOLO boxes are associated with existing contacts by distance between
current box centers and previous image-track centers. A one-time simulator seed
binds these generic tracks to the actors that answer the later simulated signal
query. That seed is an internal signal/simulation association aid: its
coordinates are never passed to ranging, filtering, oval construction,
priority scoring, overlap selection, or the interceptor command. A regression
test patches the old synthetic box detector after initialization and proves
that YOLO runtime does not call it.

Truth remains necessary outside defense guidance for:

- rasterizing the virtual world into camera pixels;
- moving the target-side scenario autopilot;
- resolving the exercise's simulated signal response;
- detecting physical contact and applying crash physics;
- calculating explicitly marked verification errors.

## Dataset

Run:

```powershell
.\.venv-yolo\Scripts\python.exe tools\generate_yolo_dataset.py
```

The deterministic default dataset contains 1,200 training images and 240 held
out validation images at 960 x 540. It includes:

- every bundled drone and rocket mesh;
- random yaw, pitch, and roll;
- one, two, and three-object frames;
- negative frames without aerial targets;
- multiple sky, cloud, terrain, and lighting variants;
- apparent target spans ranging from sub-3-pixel hard cases to close views.

The generated `datasets/zenith_yolo/metadata.json` records the seed, image
counts, class list, and truth-boundary statement. The dataset and training runs
are reproducible generated material and are excluded from Git; the compact
validated `models/zenith_yolo.pt` result and its metrics JSON are bundled.

## Training and validation

First create the isolated environment:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup_yolo.ps1
```

Then either run `train_yolo.bat` or:

```powershell
.\.venv-yolo\Scripts\python.exe tools\generate_yolo_dataset.py
.\.venv-yolo\Scripts\python.exe tools\train_yolo.py
```

The trainer starts from a nano YOLO model, trains at 960-pixel input size on
the GPU, evaluates the held-out validation split, copies the best checkpoint
to `models/zenith_yolo.pt`, and writes precision, recall, mAP50, and
mAP50-95 to `models/zenith_yolo_metrics.json`.

The bundled 40-epoch checkpoint produced these held-out generated-image
validation results:

| Metric | Result |
|---|---:|
| Precision | 0.987230 |
| Recall | 0.877023 |
| mAP50 | 0.910082 |
| mAP50-95 | 0.708614 |

The model card in `models/README.md` records the dataset, intended use,
limitations, library version, license note, and SHA-256 digest.

The final `artifacts/yolo_verification.csv` runs the complete perception-to-
collision loop for all seven target behaviors. Every row signal-identified and
physically contacted the assigned target; hit times ranged from 2.083 seconds
for the incoming rocket to 11.250 seconds for Tricky AI. Its larger range
MAE/RMSE values are retained as evidence of the expected few-pixel monocular
measurement problem rather than being hidden by synthetic observations.

Ultralytics documents both NumPy frame inference and custom training:

- <https://docs.ultralytics.com/modes/predict/>
- <https://docs.ultralytics.com/modes/train/>

Ultralytics offers AGPL-3.0 and enterprise licensing. This public educational
prototype should retain appropriate attribution and review the chosen license
before any commercial distribution:

- <https://www.ultralytics.com/license>

## Runtime behavior

YOLO is sampled at 30 sensor frames per simulated second while physics and
guidance remain fixed at 60 Hz. The detector always evaluates a native-pixel
crop fixed at the optical center and periodically batches it with the complete
sensor frame. A miss immediately restores full-FOV evaluation until something
is found. The crop is taken from the same pixels and never follows target
truth; it preserves small center-frame objects that would otherwise vanish
when a 1920/3840 image is reduced to the nano model's 960-pixel input. Between
image frames, the most recent valid track and command remain active for at most
one detector interval. Detector latency, device, detection count, confidence,
and apparent pixel span are visible in the HUD or telemetry export.

The runtime confidence threshold is `0.05` because the exercise intentionally
starts with two- or three-pixel silhouettes. Overlapping full/crop predictions
are merged by box IoU and local image-center distance. Multi-contact assignment
then uses only previous/seed image centers; a low-confidence box does not gain
identity or capabilities from YOLO.

YOLO returns no 3D target pose. Reading simulated orientation here would defeat
the purpose of the integration. Instead, after the signal lookup supplies the
known vehicle dimensions, the YOLO range path uses declared known-shape rules:

```text
drone: S = 1.10 * max(width, length), p = max(box_width, box_height)
rocket: S = max(width, height), p = min(box_width, box_height)
Z = focal_pixels * S / p
```

The drone factor `1.10` is a fixed calibration measured from held-out
model boxes: at tiny scales the detector's box is systematically wider than
the exact rendered silhouette label. It is constant for the whole run and
never reads true range, pose, coordinates, or scenario state.

The rocket minor axis represents its cross-section from nose-on through
side-on views. The full bounding diameter is retained as a conservative audit
value and contributes to an explicit shape/orientation ambiguity term. The
estimator also declares confidence and pixel-quantization uncertainty, which
expands the track uncertainty and therefore the prediction ovals. The original
synthetic pose-compensated range path remains unchanged in fallback mode.

## What it does not prove

The custom weights learn ZENITH's generated imagery. They demonstrate real
inference and software portability, not real-world detector accuracy. A
physical deployment would still require:

- labeled real drone and rocket footage from the intended camera;
- domain adaptation for weather, lighting, motion blur, terrain, and sensor
  noise;
- calibrated intrinsics and measured lens distortion;
- target-size/altitude/distance operating limits;
- false-positive, false-negative, and out-of-distribution evaluation;
- bounded hardware latency and dropped-frame behavior;
- redundant collision prevention, human supervision, safety validation, and
  legal/regulatory approval.

At the default 240 m start, a small drone may initially occupy only two or
three pixels. No detector can reliably identify detail that the camera did not
capture. `YOLO CUSTOM` therefore reports no visual lock until the target
becomes sufficiently resolved. This is an honest sensor limitation, not a
software failure.
