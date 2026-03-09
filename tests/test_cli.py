import subprocess
import sys

from robot_arm_sim import __version__


def test_cli_version():
    cmd = [sys.executable, "-m", "robot_arm_sim", "--version"]
    assert subprocess.check_output(cmd).decode().strip() == __version__
