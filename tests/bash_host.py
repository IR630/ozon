"""Locate a bash able to run this repo's scripts by their host path.

`shutil.which("bash")` on a Windows host with WSL installed returns
`C:\\Windows\\system32\\bash.exe` — the WSL launcher. It resolves its script
argument inside the Linux filesystem, so `bash D:\\vano\\ozon\\scripts\\x.sh`
dies with `D:vanoozonscriptsx.sh: No such file or directory` (bash eats the
backslashes). The old fallback — "if which() found nothing, try Git Bash" —
therefore never fired on precisely the hosts that need it: which() always
finds the stub, so it is never None. Probe candidates for the capability the
tests actually use, executing a script named by a host path, and take the
first that passes. On Linux/macOS/WSL-native the first candidate is the real
bash and passes on the first probe, exactly as before.
"""
import functools
import os
import shutil
import subprocess
import tempfile
from pathlib import Path


def _candidates():
    seen = set()
    found = shutil.which("bash")
    if found:
        seen.add(found.lower())
        yield found
    if os.name == "nt":
        for root in (os.environ.get("ProgramFiles", r"C:\Program Files"),
                     os.environ.get("ProgramW6432", r"C:\Program Files")):
            git_bash = Path(root) / "Git" / "bin" / "bash.exe"
            if git_bash.exists() and str(git_bash).lower() not in seen:
                seen.add(str(git_bash).lower())
                yield str(git_bash)


def _runs_host_paths(bash):
    """True if `bash <host path to a script>` actually executes that script."""
    with tempfile.TemporaryDirectory() as tmp:
        probe = Path(tmp) / "probe.sh"
        probe.write_text("echo ok\n", encoding="utf-8")
        try:
            result = subprocess.run([bash, str(probe)], capture_output=True, text=True, timeout=60)
        except (OSError, subprocess.SubprocessError):
            return False
    return result.returncode == 0 and result.stdout.strip() == "ok"


@functools.lru_cache(maxsize=1)
def find_bash():
    """The first bash on this host that can run a script by its host path, else None."""
    for bash in _candidates():
        if _runs_host_paths(bash):
            return bash
    return None
