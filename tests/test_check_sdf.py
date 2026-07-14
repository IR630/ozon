"""Contract tests for the fast SDF validator."""
import os
import subprocess
from pathlib import Path

import pytest
from bash_host import find_bash

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_sdf.sh"


def _bash():
    bash = find_bash()
    if bash is None:
        pytest.skip("bash is unavailable on this host")
    return bash


def test_validator_passes_every_world_to_ign_not_only_the_legacy_baseline(tmp_path):
    """The final diverter world must not be absent from a green SDF check."""
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    calls = tmp_path / "ign-calls.log"
    fake_ign = fake_bin / "ign"
    fake_ign.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >> \"$IGN_CALLS\"\n",
        encoding="utf-8",
    )
    fake_ign.chmod(0o755)
    env = os.environ.copy()
    env["IGN_CALLS"] = str(calls)
    env["PATH"] = f"{fake_bin}{os.pathsep}{env.get('PATH', '')}"

    result = subprocess.run(
        [_bash(), str(SCRIPT)],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    invoked = calls.read_text(encoding="utf-8")
    for world in sorted((ROOT / "sim" / "worlds").glob("*.sdf")):
        relative = world.relative_to(ROOT).as_posix()
        assert f"sdf -k {relative}" in invoked
        assert f"OK: {relative}" in result.stdout

