# -*- coding: utf-8 -*-
"""One joint's angle, read from an `ign topic -e` text stream on stdin.

WHY THIS IS NOT A grep. Ignition prints protobuf TEXT FORMAT, and protobuf omits
any field that holds its default value — so a joint resting at exactly 0 rad has
no `position:` line at all:

    axis1 {                      axis1 {
      xyz { z: 1 }                 xyz { z: 1 }
      limit_upper: 0.95            limit_upper: 0.95
      position: 0.7086           }                    <- parked: the field is GONE
    }

`smoke_estop_stream.sh` read that with `grep -A6 axis1 | grep -m1 "position:"`,
which cannot tell "the blade is at zero" from "keep looking further down" — and
`ign topic -e` STREAMS, so "further down" is another message, arriving later,
about a different instant.

MEASURED, not deduced. A 131-sample trace of one occupied-E-stop run shows the
blade holding 0.7086 rad through the whole stop (the safety property under test),
then parking to 0.0005 rad before the belt was released, then sitting at exactly
0.0 for the next eleven seconds. The smoke, sampling that same parked blade,
reported 3.638 rad and failed the run. Three consecutive runs failed with 1.831,
2.856 and 4.106 rad, and one on `main` failed the opposite assertion — different
numbers every time, from identical code, because the reader and not the cell was
the thing that varied.

So: absent means zero, and only the FIRST message counts — the caller is asking
where the blade is now, not which value happened to scroll past.
"""
import re
import sys

# `position: <number>` inside axis1. The model's own `pose { position { x: ... } }`
# uses a BRACE, not a colon, so it cannot be confused for the joint angle.
_POSITION = re.compile(r"^\s*position:\s*(-?[0-9.]+(?:[eE][-+]?[0-9]+)?)\s*$")


def angle_from_stream(text):
    """Radians from the FIRST message in `text`.

    0.0 when the message is there but the field was omitted (a parked joint),
    and None when NO message arrived at all. Those two must stay distinguishable:
    collapsing "the sensor said nothing" into "the blade is at zero" is the same
    class of defect this file exists to remove, and the caller gates a belt
    restart on it.
    """
    seen_header = False
    for line in text.splitlines():
        if line.startswith("header {"):
            if seen_header:
                break        # into the second message: the first one had no angle
            seen_header = True
        match = _POSITION.match(line)
        if match and seen_header:
            return float(match.group(1))
    return 0.0 if seen_header else None


def main():
    angle = angle_from_stream(sys.stdin.read())
    if angle is None:
        return 1             # no feedback — the caller has an explicit guard
    print(f"{angle:.9f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
