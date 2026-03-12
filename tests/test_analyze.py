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


def test_run_analysis_preserves_manual_connection_points(robot_dir: Path):
    """Manual bore placements survive re-analysis by default."""
    analysis_dir = robot_dir / "analysis"

    # Run initial analysis to generate YAMLs
    if analysis_dir.exists():
        import shutil

        shutil.rmtree(analysis_dir)
    run_analysis(robot_dir)

    # Pick the first part YAML and inject a manual connection point
    part_yamls = [p for p in analysis_dir.glob("*.yaml") if p.name != "summary.yaml"]
    assert part_yamls
    target = part_yamls[0]

    data = yaml.safe_load(target.read_text())
    manual_cp = {
        "end": "proximal",
        "position": [1.0, 2.0, 3.0],
        "axis": [0.0, 0.0, 1.0],
        "radius_mm": 5.0,
        "method": "manual",
        "center": True,
    }
    data["connection_points"] = [manual_cp]
    with open(target, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    # Re-run analysis — manual CP should be preserved
    run_analysis(robot_dir)

    reloaded = yaml.safe_load(target.read_text())
    cps = reloaded["connection_points"]
    manual_found = [cp for cp in cps if cp.get("method") == "manual"]
    assert len(manual_found) == 1
    cp = manual_found[0]
    assert cp["end"] == "proximal"
    assert cp["position"] == [1.0, 2.0, 3.0]
    assert cp["center"] is True


def test_run_analysis_override_manual_replaces_all(robot_dir: Path):
    """With override_manual=True, manual CPs are replaced by auto-detection."""
    analysis_dir = robot_dir / "analysis"

    if analysis_dir.exists():
        import shutil

        shutil.rmtree(analysis_dir)
    run_analysis(robot_dir)

    part_yamls = [p for p in analysis_dir.glob("*.yaml") if p.name != "summary.yaml"]
    assert part_yamls
    target = part_yamls[0]

    data = yaml.safe_load(target.read_text())
    manual_cp = {
        "end": "proximal",
        "position": [1.0, 2.0, 3.0],
        "axis": [0.0, 0.0, 1.0],
        "radius_mm": 5.0,
        "method": "manual",
    }
    data["connection_points"] = [manual_cp]
    with open(target, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    # Re-run with override — manual CP should be gone
    run_analysis(robot_dir, override_manual=True)

    reloaded = yaml.safe_load(target.read_text())
    cps = reloaded["connection_points"]
    manual_found = [cp for cp in cps if cp.get("method") == "manual"]
    assert len(manual_found) == 0
