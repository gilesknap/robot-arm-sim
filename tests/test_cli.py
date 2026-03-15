from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from typer.testing import CliRunner

from robot_arm_sim import __version__
from robot_arm_sim.__main__ import app

runner = CliRunner()


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


# ---------------------------------------------------------------------------
# Typer CliRunner-based tests
# ---------------------------------------------------------------------------


def test_runner_version():
    """--version flag via CliRunner."""
    result = runner.invoke(app, ["--version"])
    assert result.exit_code == 0
    assert __version__ in result.stdout


def test_runner_help():
    """--help flag via CliRunner."""
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "analyze" in result.stdout


def test_generate_help():
    """generate --help via CliRunner."""
    result = runner.invoke(app, ["generate", "--help"])
    assert result.exit_code == 0
    assert "ROBOT_DIR" in result.stdout or "robot_dir" in result.stdout.lower()


def test_generate_missing_chain(robot_dir_no_urdf: Path):
    """generate errors when chain.yaml is missing."""
    # Remove chain.yaml if it exists
    chain = robot_dir_no_urdf / "chain.yaml"
    if chain.exists():
        chain.unlink()
    result = runner.invoke(app, ["generate", str(robot_dir_no_urdf)])
    assert result.exit_code != 0


def test_generate_with_chain(robot_dir: Path):
    """generate succeeds when chain.yaml exists."""
    chain = robot_dir / "chain.yaml"
    if not chain.exists():
        return  # skip if sample robot doesn't have chain.yaml
    result = runner.invoke(app, ["generate", str(robot_dir)])
    assert result.exit_code == 0
    assert "Done" in result.stdout


def test_analyze_nonexistent_dir():
    """analyze errors on nonexistent directory."""
    result = runner.invoke(app, ["analyze", "/nonexistent/path/to/robot"])
    assert result.exit_code != 0
