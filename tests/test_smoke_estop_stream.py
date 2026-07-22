# -*- coding: utf-8 -*-
"""Static contracts for the occupied-E-stop stream smoke.

The smoke's whole premise is "the blade is engaged as a wall when the stop
lands". It reaches that blade through its own slow instrumentation (two
`ign topic -e` reads at 2 s each, plus first-publish discovery on the stop
topic), so if the blade's hold expires inside that window the stop hits an
already-parked blade and the smoke tests a state it never set up. Measured:
three runs in a row auto-retracted ~1 s before the stop. These pin the fix so it
cannot silently regress to the too-short production hold.
"""
import re
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "smoke_estop_stream.sh"


def _default(var):
    text = SCRIPT.read_text(encoding="utf-8")
    match = re.search(rf"export {var}=\$\{{{var}:-([0-9.]+)\}}", text)
    assert match, f"no default for {var}"
    return match.group(1)


def test_the_blade_stays_engaged_past_the_smoke_s_own_detection_latency():
    """2.5 s (the production stroke hold) is shorter than the read+publish path
    that gets the smoke to the engaged blade, which is why the stop kept landing
    after a normal retract."""
    hold = float(_default("HOLD_S"))
    assert hold >= 10.0, (
        f"HOLD_S={hold}s risks the blade auto-retracting before the E-stop "
        "arrives — the occupied premise is then never reached")


def test_hold_s_default_is_a_double_literal_not_an_int():
    """`-p hold_s:=15` is parsed as INTEGER and the node rejects it against a
    DOUBLE parameter; the controller then never comes up and the smoke fails as
    'controller never completed its initial soft-start'."""
    assert "." in _default("HOLD_S"), "HOLD_S must be a float literal (e.g. 15.0)"


def test_the_engaged_blade_angle_is_read_by_the_message_parser_not_grep():
    """protobuf omits a zero field, so a parked blade has no `position:` line;
    the joint angle must go through the message parser, not a stream grep."""
    text = SCRIPT.read_text(encoding="utf-8")
    assert "scripts/parse_ign_joint_angle.py" in text
    assert 'grep -A6 "axis1"' not in text, "the stream-grep reader is back"
