"""Read-only integrity and timing audit of an existing raw episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from laptop.recorder.episode import validate_episode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path, help="existing local raw episode directory")
    args = parser.parse_args()
    report = validate_episode(args.episode)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["status"] == "FILE_INTEGRITY_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
