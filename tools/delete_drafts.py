#!/usr/bin/env python3
"""Delete annotation drafts from a Label Studio project via the API.

By default the script only lists drafts. Pass --apply to actually delete them.
Annotations and predictions are never touched: this only hits /api/drafts/.
Requires LABEL_STUDIO_TOKEN (legacy token, same as import_yolo_predictions.py).
"""

from __future__ import annotations

import argparse
import json
import os
import sqlite3
import urllib.error
import urllib.request
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=os.environ.get("LABEL_STUDIO_URL", "http://localhost:8080"))
    parser.add_argument("--project-id", type=int, default=1)
    parser.add_argument("--database", type=Path, default=Path("samples/label_studio.sqlite3"))
    parser.add_argument("--apply", action="store_true", help="actually delete the drafts")
    return parser.parse_args()


def request(url: str, access_token: str | None = None, method: str = "GET", payload: dict | None = None) -> bytes:
    headers = {"Content-Type": "application/json"}
    if access_token:
        headers["Authorization"] = f"Bearer {access_token}"
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode() if payload is not None else None,
        headers=headers,
        method=method,
    )
    try:
        with urllib.request.urlopen(req) as response:
            return response.read()
    except urllib.error.HTTPError as error:
        raise SystemExit(f"Label Studio returned HTTP {error.code} for {method} {url}: {error.read().decode()}") from error


def get_access_token(base_url: str) -> str:
    token = os.environ.get("LABEL_STUDIO_TOKEN")
    if not token:
        raise SystemExit("LABEL_STUDIO_TOKEN is required")
    raw = request(f"{base_url}/api/token/refresh/", method="POST", payload={"refresh": token})
    return json.loads(raw)["access"]


def list_drafts(database: Path, project_id: int) -> list[dict]:
    """Read draft IDs only; deletion itself is performed through the HTTP API."""
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    rows = connection.execute(
        "SELECT draft.id, draft.task_id "
        "FROM tasks_annotationdraft AS draft "
        "JOIN task ON task.id = draft.task_id "
        "WHERE task.project_id = ? "
        "ORDER BY draft.id",
        (project_id,),
    ).fetchall()
    connection.close()
    return [{"id": draft_id, "task": task_id} for draft_id, task_id in rows]


def main() -> None:
    args = parse_args()
    base_url = args.url.rstrip("/")
    access_token = get_access_token(base_url)
    drafts = list_drafts(args.database, args.project_id)
    print(f"found {len(drafts)} drafts in project {args.project_id}")
    if not args.apply:
        for draft in drafts[:10]:
            print(f"  draft id={draft['id']} task={draft.get('task')}")
        if len(drafts) > 10:
            print(f"  ... and {len(drafts) - 10} more")
        print("dry run; pass --apply to delete")
        return
    for index, draft in enumerate(drafts, start=1):
        request(f"{base_url}/api/drafts/{draft['id']}/", access_token, method="DELETE")
        if index % 50 == 0 or index == len(drafts):
            print(f"deleted {index}/{len(drafts)}")
    print("done")


if __name__ == "__main__":
    main()
