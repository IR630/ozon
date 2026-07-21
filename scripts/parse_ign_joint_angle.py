#!/usr/bin/env python3
"""Read the first ``axis1`` angle from ``ign topic -e`` protobuf text.

Ignition omits scalar fields whose value is the protobuf default. A parked joint
therefore has no ``position: 0`` line at all. Parsing a fixed number of following
lines can then steal an unrelated position from the next block/message. This
parser respects the ``axis1 { ... }`` braces and returns 0.0 only when that exact
block closes without an explicit position.
"""
from __future__ import annotations

import re
import sys
from collections.abc import Iterable


_AXIS_START = re.compile(r"^\s*axis1\s*\{")
_POSITION = re.compile(
    r"^\s*position:\s*"
    r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$"
)


def first_axis1_angle(lines: Iterable[str]) -> float:
    """Return the first axis1 position, including an omitted protobuf zero."""
    depth = 0
    in_axis = False
    for line in lines:
        if not in_axis:
            if not _AXIS_START.match(line):
                continue
            in_axis = True
            depth = line.count("{") - line.count("}")
            if depth <= 0:
                return 0.0
            continue

        match = _POSITION.match(line)
        if match:
            return float(match.group(1))

        depth += line.count("{") - line.count("}")
        if depth <= 0:
            return 0.0

    raise ValueError("no complete axis1 block in Ignition joint-state input")


def main() -> int:
    try:
        angle = first_axis1_angle(sys.stdin)
    except ValueError as exc:
        print(f"joint-angle parse error: {exc}", file=sys.stderr)
        return 1
    print(f"{angle:.17g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
