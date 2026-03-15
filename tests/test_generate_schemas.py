"""Tests for JSON schema generation."""

from __future__ import annotations

import json
from pathlib import Path

from robot_arm_sim.generate_schemas import SCHEMAS, generate


def test_generate_creates_schema_files(tmp_path: Path, monkeypatch: object):
    """generate() creates JSON schema files in a temporary schemas/ directory."""
    import robot_arm_sim.generate_schemas as gs_mod

    # Make the module think __file__ resolves so that root = tmp_path
    # The code does: root = Path(__file__).resolve().parent.parent.parent
    # We create the expected structure so pyproject.toml is found at tmp_path
    fake_pkg = tmp_path / "src" / "robot_arm_sim"
    fake_pkg.mkdir(parents=True)
    fake_file = fake_pkg / "generate_schemas.py"
    fake_file.write_text("")
    (tmp_path / "pyproject.toml").write_text("[project]\nname = 'test'\n")

    # Patch __file__ on the module
    original_file = gs_mod.__file__
    monkeypatch.setattr(gs_mod, "__file__", str(fake_file))  # type: ignore[attr-defined]

    try:
        generate()
    finally:
        monkeypatch.setattr(gs_mod, "__file__", original_file)  # type: ignore[attr-defined]

    schemas_dir = tmp_path / "schemas"
    assert schemas_dir.is_dir()

    for name in SCHEMAS:
        schema_file = schemas_dir / f"{name}.json"
        assert schema_file.exists(), f"Schema file {name}.json not created"
        content = json.loads(schema_file.read_text())
        assert isinstance(content, dict)


def test_schemas_dict_has_expected_entries():
    """SCHEMAS dict contains the expected model names."""
    expected = {"part_analysis", "chain", "summary", "specs", "view_mapping"}
    assert set(SCHEMAS.keys()) == expected


def test_each_model_produces_valid_schema():
    """Each model in SCHEMAS produces a valid JSON schema dict."""
    for name, model_cls in SCHEMAS.items():
        schema = model_cls.model_json_schema()
        assert isinstance(schema, dict), f"{name} schema is not a dict"
        # Should be serializable to JSON
        json_str = json.dumps(schema, indent=2)
        roundtrip = json.loads(json_str)
        assert roundtrip == schema, f"{name} schema JSON roundtrip failed"


def test_generate_runs_without_error():
    """generate() runs end-to-end without raising."""
    generate()
    repo_root = Path(__file__).resolve().parent.parent
    schemas_dir = repo_root / "schemas"
    assert schemas_dir.is_dir()
    for name in SCHEMAS:
        assert (schemas_dir / f"{name}.json").exists()
