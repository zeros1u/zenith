# ZENITH YOLO model card

## Artifact

- file: `zenith_yolo.pt`
- task: object detection
- class: `aerial_target`
- starting checkpoint: `yolo11n.pt`
- training library: Ultralytics `8.4.112`
- input size used for training/validation: `960`
- file size: approximately 5.5 MB
- SHA-256: `d572832f485c6e7c3daecef4595d039a2588d7444d97b2b42a1f342e2f802b54`

## Training data

The deterministic generated dataset has 1,200 training frames and 240
validation frames at 960 x 540, with 1,856 labeled instances across all ZENITH
drone and rocket meshes. It includes negative images, 1-3 target scenes,
varied orientation, scale, sky/terrain style, and targets down to approximately
1.2 pixels. Simulator labels exist only in the offline dataset generator.

## Held-out validation

| Metric | Result |
|---|---:|
| Precision | 0.987230 |
| Recall | 0.877023 |
| mAP50 | 0.910082 |
| mAP50-95 | 0.708614 |

The machine-readable report is `zenith_yolo_metrics.json`. These results
measure generated ZENITH imagery, not real outdoor footage.

## Intended use and limitations

The model is intended to prove an actual replaceable pixel-to-box boundary in
the ZENITH educational simulator. It identifies only the generic aerial-target
shape. It does not identify a vehicle model, measure range, infer capabilities,
or receive simulator coordinates/pose. The later simulated signal lookup
provides the exercise-given model data.

This checkpoint is not validated for deployment, targeting, or safety-critical
use. Physical use would require representative real-camera training data,
calibrated optics, domain adaptation, false-positive/negative testing, hardware
latency validation, redundant safety measures, human supervision, and legal
review.

## License and attribution

The checkpoint was trained from an Ultralytics YOLO checkpoint whose embedded
metadata declares AGPL-3.0 and links to the Ultralytics license page:
<https://www.ultralytics.com/license>. Review licensing before redistribution
or commercial use.
