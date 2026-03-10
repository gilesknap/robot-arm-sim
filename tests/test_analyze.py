"""Tests for the analysis pipeline."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from robot_arm_sim.analyze import run_analysis


def test_run_analysis_creates_output(robot_dir: Path):
    # Remove existing analysis to test fresh generation
    analysis_dir = robot_dir / "analysis"
    if analysis_dir.exists():
        import shutil

        shutil.rmtree(analysis_dir)

    run_analysis(robot_dir)

    assert analysis_dir.exists()
    assert (analysis_dir / "summary.yaml").exists()

    # Should have at least one part YAML
    part_yamls = list(analysis_dir.glob("*.yaml"))
    assert len(part_yamls) >= 2  # summary + at least one part


def test_run_analysis_missing_stl_dir(tmp_path: Path):
    with pytest.raises(FileNotFoundError, match="stl_files"):
        run_analysis(tmp_path)


def test_run_analysis_summary_format(robot_dir: Path):
    # Remove existing analysis
    analysis_dir = robot_dir / "analysis"
    if analysis_dir.exists():
        import shutil

        shutil.rmtree(analysis_dir)

    run_analysis(robot_dir)

    summary = yaml.safe_load((analysis_dir / "summary.yaml").read_text())
    assert "robot_name" in summary
    assert "part_count" in summary
    assert "parts" in summary
    assert isinstance(summary["parts"], list)
    assert summary["part_count"] == len(summary["parts"])

    for part in summary["parts"]:
        assert "name" in part
        assert "file" in part
        assert "role_hint" in part
