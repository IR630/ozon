"""Static contracts for the occupied E-stop smoke setup."""
import re
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "smoke_estop_stream.sh"


def _default(var):
    text = SCRIPT.read_text(encoding="utf-8")
    match = re.search(rf"export {var}=\$\{{{var}:-([0-9.]+)\}}", text)
    assert match, f"no default for {var}"
    return match.group(1)


def test_blade_stays_engaged_past_the_smoke_detection_latency():
    """The safety smoke must stop an engaged blade, not one already parked."""
    hold = float(_default("HOLD_S"))
    assert hold >= 10.0, (
        f"HOLD_S={hold}s risks auto-retracting before the E-stop reaches the blade")


def test_hold_s_is_a_float_literal_for_the_ros_double_parameter():
    assert "." in _default("HOLD_S"), "HOLD_S must be a float literal"


def test_joint_angle_uses_the_protobuf_aware_parser():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "scripts/parse_ign_joint_angle.py" in text
    assert 'grep -A6 "axis1"' not in text
