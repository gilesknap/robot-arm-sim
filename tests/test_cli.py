import subprocess
import sys

from robot_arm_sim import __version__


def test_cli_version():
    cmd = [sys.executable, "-m", "robot_arm_sim", "--version"]
    assert __version__ in subprocess.check_output(cmd).decode().strip()


def test_cli_help():
    cmd = [sys.executable, "-m", "robot_arm_sim", "--help"]
    output = subprocess.check_output(cmd).decode()
    assert "analyze" in output
    assert "simulate" in output


def test_analyze_help():
    cmd = [sys.executable, "-m", "robot_arm_sim", "analyze", "--help"]
    output = subprocess.check_output(cmd).decode()
    assert "robot_dir" in output.lower() or "ROBOT_DIR" in output


def test_simulate_help():
    cmd = [sys.executable, "-m", "robot_arm_sim", "simulate", "--help"]
    output = subprocess.check_output(cmd).decode()
    assert "robots_dir" in output.lower() or "ROBOTS_DIR" in output
