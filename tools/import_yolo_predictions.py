#!/usr/bin/env python3
"""Attach YOLO TXT detections to existing Label Studio tasks as predictions.

By default the script only writes an import payload. Pass --apply to send it to
Label Studio after reviewing the payload and setting LABEL_STUDIO_TOKEN.
Saved annotations are always skipped; drafts are never read or modified.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sqlite3
import sys
import urllib.error
import urllib.request
from pathlib import Path


CLASS_NAMES = ["white", "yellow", "red", "orange", "green", "blue"]
UPLOAD_PREFIX = re.compile(r"^[0-9a-f]{8}-(shot_.+\.png)$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database", type=Path, default=Path("samples/label_studio.sqlite3"))
    parser.add_argument(
        "--labels", type=Path, default=Path("runs/detect/runs/detect/prelabels/labels")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("samples/predictions/yolov8n-rocm-predictions.json")
    )
    parser.add_argument("--project-id", type=int, default=1)
    parser.add_argument("--model-version", default="yolov8n-rocm-2026-07-23")
    parser.add_argument("--apply", action="store_true", help="POST the reviewed payload to Label Studio")
    parser.add_argument("--url", default=os.environ.get("LABEL_STUDIO_URL", "http://localhost:8080"))
    return parser.parse_args()


def task_filename(data: str) -> str:
    image = Path(json.loads(data)["image"]).name
    match = UPLOAD_PREFIX.match(image)
    return match.group(1) if match else image


def prediction_result(label_file: Path) -> tuple[list[dict], float]:
    result: list[dict] = []
    confidences: list[float] = []
    for line_number, line in enumerate(label_file.read_text().splitlines(), start=1):
        values = line.split()
        if len(values) != 6:
            raise ValueError(f"{label_file}:{line_number}: expected 6 YOLO values")
        class_id = int(values[0])
        if not 0 <= class_id < len(CLASS_NAMES):
            raise ValueError(f"{label_file}:{line_number}: invalid class ID {class_id}")
        center_x, center_y, width, height, confidence = map(float, values[1:])
        result.append(
            {
                "from_name": "label",
                "to_name": "image",
                "type": "rectanglelabels",
                "original_width": 1920,
                "original_height": 1080,
                "image_rotation": 0,
                "value": {
                    "x": (center_x - width / 2) * 100,
                    "y": (center_y - height / 2) * 100,
                    "width": width * 100,
                    "height": height * 100,
                    "rotation": 0,
                    "rectanglelabels": [CLASS_NAMES[class_id]],
                },
            }
        )
        confidences.append(confidence)
    return result, sum(confidences) / len(confidences) if confidences else 0.0


def build_payload(args: argparse.Namespace) -> tuple[list[dict], int]:
    connection = sqlite3.connect(f"file:{args.database}?mode=ro", uri=True)
    tasks = connection.execute(
        "SELECT id, data FROM task "
        "WHERE project_id = ? AND total_annotations = 0 AND total_predictions = 0 "
        "ORDER BY id",
        (args.project_id,),
    )
    payload: list[dict] = []
    missing = 0
    for task_id, data in tasks:
        image_name = task_filename(data)
        label_file = args.labels / f"{Path(image_name).stem}.txt"
        if not label_file.exists():
            print(f"warning: no prediction for task {task_id}: {image_name}", file=sys.stderr)
            missing += 1
            continue
        result, score = prediction_result(label_file)
        payload.append(
            {
                "task": task_id,
                "model_version": args.model_version,
                "score": score,
                "result": result,
            }
        )
    return payload, missing


def apply_payload(args: argparse.Namespace, payload: list[dict]) -> None:
    token = os.environ.get("LABEL_STUDIO_TOKEN")
    if not token:
        raise SystemExit("LABEL_STUDIO_TOKEN is required with --apply")
    refresh_request = urllib.request.Request(
        f"{args.url.rstrip('/')}/api/token/refresh/",
        data=json.dumps({"refresh": token}).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(refresh_request) as response:
            access_token = json.loads(response.read())["access"]
    except urllib.error.HTTPError as error:
        raise SystemExit(f"Label Studio token refresh failed (HTTP {error.code}): {error.read().decode()}") from error
    request = urllib.request.Request(
        f"{args.url.rstrip('/')}/api/projects/{args.project_id}/import/predictions",
        data=json.dumps(payload).encode(),
        headers={"Authorization": f"Bearer {access_token}", "Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request) as response:
            print(response.read().decode())
    except urllib.error.HTTPError as error:
        raise SystemExit(f"Label Studio returned HTTP {error.code}: {error.read().decode()}") from error


def main() -> None:
    args = parse_args()
    payload, missing = build_payload(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {len(payload)} predictions to {args.output}")
    if missing:
        print(f"skipped {missing} tasks without a matching prediction", file=sys.stderr)
    if args.apply:
        apply_payload(args, payload)


if __name__ == "__main__":
    main()
