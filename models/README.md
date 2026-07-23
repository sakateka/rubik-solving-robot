# Public model releases

This directory contains the intentionally published artifacts for the Rubik's
Cube face detector. Training logs, experiments, source images, labels, and
temporary exports remain outside this directory and are not part of a release.

## What the model does

The model detects the nine visible stickers on one front-facing Rubik's Cube
face and assigns a color class to each box.

| Class ID | Color |
| ---: | --- |
| 0 | white |
| 1 | yellow |
| 2 | red |
| 3 | orange |
| 4 | green |
| 5 | blue |

The detector does not encode a sticker order. A consumer must sort the nine
detection centers by Y into three rows, then sort each row by X.

## Released files

| File | Purpose |
| --- | --- |
| `releases/cube-yolov8n-v2.pt` | Ultralytics/PyTorch training and reference model. |
| `releases/cube-yolov8n-v2-320.onnx` | Static ONNX export for host-side inference and conversion. Input: `1x3x320x320`; output: `1x10x2100`. |
| `releases/cube-yolov8n-v2-320-cv181x-bf16.cvimodel` | BF16 model compiled for the SG2002 (`CV181X`) TPU in Milk-V Duo 256M. |

The `.cvimodel` is a baseline TPU artifact. It was built without fused image
preprocessing, so an application must provide the expected preprocessed tensor
or add an explicit preprocessing stage. It is not an image-file command-line
application by itself.

## Required image preparation

This model was trained for a fixed camera rig. For a source frame of
`1920x1080`, crop exactly this rectangle before inference:

```text
left=464, top=32, right=1296, bottom=864
result: 832x832 pixels
```

Resize that square to `320x320`, convert BGR camera data to RGB when needed,
and scale pixel values from `0..255` to `0..1`. The ONNX model expects NCHW
float input: `[batch, channel, height, width]`.

Using a different crop may reintroduce false detections from the camera rig and
will not match the training setup.

## Recommended postprocessing

The following rule was checked on the held-out test set:

1. Keep detections with confidence at least `0.50`.
2. Keep centers inside the expected cube field: normalized `0.05 <= x <= 0.90`
   and `0.05 <= y <= 0.95` in the cropped image.
3. Apply class-agnostic NMS with IoU threshold `0.50`.
4. Require exactly nine remaining boxes. Otherwise reject the frame and capture
   another one.
5. Build the 3x3 result grid by sorting box centers by Y, then X.

Do not silently select the top nine detections: a false positive could replace
a missed sticker.

## Integrity checks

```text
cube-yolov8n-v2.pt
  SHA256 45e97434711a71947b9bfd4dc2a65febf6894f89a76fda1f2ec2c19fdd85c13d

cube-yolov8n-v2-320.onnx
  SHA256 ddc75333ba757fa005e74cf7224a48ed7bcc016914b6710d600f5bc2ec6c99a1

cube-yolov8n-v2-320-cv181x-bf16.cvimodel
  SHA256 1e1fae29c48e8f24a7f8bd155c892875841816e4fbab063be4a63986165aae12
```

## Training context

The released YOLOv8n v2 model was trained at `320x320` on 215 images from the
fixed rig, validated on 50 images, and evaluated on a separate 40-image test
split. Test metrics were mAP50-95 `0.991`, precision `0.999`, and recall
`1.000`. These numbers describe this rig and its data distribution; they are
not a claim of general cube-color recognition under arbitrary lighting.
