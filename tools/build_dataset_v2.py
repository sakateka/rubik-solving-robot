#!/usr/bin/env python3
"""Build the cropped YOLO v2 dataset from a Label Studio YOLO export.

The camera rig is static.  Keeping the cube face in one fixed ROI removes
the orange chassis on the right before both training and deployment.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
import tempfile
import zipfile
from collections import Counter
from pathlib import Path

from PIL import Image


CLASSES = ["white", "yellow", "red", "orange", "green", "blue"]
# (left, top, right, bottom), measured in the original 1920x1080 frames.
# This has only 28--30 px horizontal clearance from the most extreme labelled
# sticker; those frames already include the maximum observed rig play.
ROI = (464, 32, 1296, 864)
SHOT_RE = re.compile(r"shot_(\d+)\.yuv")


def shot_number(name: str) -> int:
    match = SHOT_RE.search(name)
    if not match:
        raise ValueError(f"Cannot find shot number in {name}")
    return int(match.group(1))


def split_for(shot: int) -> str:
    """Keep consecutive captures together so near-identical frames do not leak."""
    group = (shot - 1) // 10
    remainder = group % 7
    if remainder == 1:
        return "val"
    if remainder == 5:
        return "test"
    return "train"


def crop_labels(label_text: str, original_size: tuple[int, int]) -> str:
    original_w, original_h = original_size
    left, top, right, bottom = ROI
    roi_w, roi_h = right - left, bottom - top
    result: list[str] = []

    for line in label_text.splitlines():
        class_id, cx, cy, width, height = line.split()[:5]
        cx, cy, width, height = map(float, (cx, cy, width, height))
        x1 = (cx - width / 2) * original_w
        y1 = (cy - height / 2) * original_h
        x2 = (cx + width / 2) * original_w
        y2 = (cy + height / 2) * original_h
        if not (left <= x1 and x2 <= right and top <= y1 and y2 <= bottom):
            raise ValueError(f"Box {line!r} is outside ROI {ROI}")
        cropped_cx = ((x1 + x2) / 2 - left) / roi_w
        cropped_cy = ((y1 + y2) / 2 - top) / roi_h
        cropped_w = (x2 - x1) / roi_w
        cropped_h = (y2 - y1) / roi_h
        result.append(
            f"{class_id} {cropped_cx:.8f} {cropped_cy:.8f} "
            f"{cropped_w:.8f} {cropped_h:.8f}"
        )
    return "\n".join(result) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--export", type=Path, required=True)
    parser.add_argument("--images", type=Path, default=Path("images"))
    parser.add_argument("--output", type=Path, default=Path("dataset-v2"))
    args = parser.parse_args()

    if args.output.exists():
        sys.exit(f"Refusing to overwrite existing {args.output}")
    source_images = {shot_number(p.name): p for p in args.images.glob("*.png")}
    if len(source_images) != 305:
        sys.exit(f"Expected 305 source images, found {len(source_images)}")

    with tempfile.TemporaryDirectory() as temporary:
        extracted = Path(temporary)
        with zipfile.ZipFile(args.export) as archive:
            archive.extractall(extracted)
        label_files = sorted((extracted / "labels").glob("*.txt"))
        if len(label_files) != 305:
            sys.exit(f"Expected 305 labels in export, found {len(label_files)}")

        args.output.mkdir()
        for split in ("train", "val", "test"):
            (args.output / "images" / split).mkdir(parents=True)
            (args.output / "labels" / split).mkdir(parents=True)

        split_counts: Counter[str] = Counter()
        class_counts: Counter[str] = Counter()
        for label_path in label_files:
            shot = shot_number(label_path.name)
            image_path = source_images.get(shot)
            if image_path is None:
                raise ValueError(f"No image for {label_path.name}")
            split = split_for(shot)
            with Image.open(image_path) as image:
                if image.size != (1920, 1080):
                    raise ValueError(f"Unexpected image size {image.size}: {image_path}")
                cropped = image.crop(ROI)
                out_image = args.output / "images" / split / f"shot_{shot:03d}.png"
                cropped.save(out_image, optimize=True)
            out_label = args.output / "labels" / split / f"shot_{shot:03d}.txt"
            text = crop_labels(label_path.read_text(), (1920, 1080))
            out_label.write_text(text)
            split_counts[split] += 1
            class_counts.update(line.split()[0] for line in text.splitlines())

    (args.output / "data.yaml").write_text(
        "path: .\ntrain: images/train\nval: images/val\ntest: images/test\n"
        "names:\n" + "\n".join(f"  {i}: {name}" for i, name in enumerate(CLASSES)) + "\n"
    )
    (args.output / "README.md").write_text(
        "# Dataset v2\n\n"
        f"Source: `{args.export.name}` (305 saved Label Studio annotations).\n\n"
        f"Fixed ROI on 1920x1080 source frames: `{ROI}`; cropped image size: "
        f"`{ROI[2] - ROI[0]}x{ROI[3] - ROI[1]}`.\n\n"
        "Split by groups of ten consecutive shots to avoid leakage from near-identical captures.\n"
    )
    print("ROI:", ROI, "cropped size:", (ROI[2] - ROI[0], ROI[3] - ROI[1]))
    print("images:", dict(sorted(split_counts.items())))
    print("boxes by class:", dict(sorted(class_counts.items())))


if __name__ == "__main__":
    main()
