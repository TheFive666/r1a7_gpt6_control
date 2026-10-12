#!/usr/bin/env python3
"""Minimal JSON CLI for agents that cannot import the Python SDK directly."""
import argparse
import json
from dataclasses import asdict
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT))

from agent.r1a7_sdk import ACTION_SCHEMA, R1A7PickPlaceSDK, SDKError  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("command", choices=("schema", "status", "observe"))
    args = parser.parse_args()
    try:
        if args.command == "schema":
            result = ACTION_SCHEMA
        else:
            sdk = R1A7PickPlaceSDK(args.session)
            if args.command == "status":
                result = sdk.status()
            else:
                result = asdict(sdk.observe())
        print(json.dumps({"ok": True, "result": result}, ensure_ascii=False))
        return 0
    except (SDKError, json.JSONDecodeError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
