"""Tests for Pydantic schema validation of all robot YAML files."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from robot_arm_sim.models import (
    Chain,
    PartAnalysis,
    Specs,
    Summary,
    ViewMapping,
    load_chain_yaml,
    load_part_yaml,
    load_specs_yaml,
    load_summary_yaml,
    load_view_mapping_yaml,
    save_chain_yaml,
    save_part_yaml,
    save_specs_yaml,
    save_summary_yaml,
    save_view_mapping_yaml,
)

ROBOTS_DIR = Path(__file__).resolve().parent.parent / "robots"
SCHEMAS_DIR = Path(__file__).resolve().parent.parent / "schemas"


# ---------------------------------------------------------------------------
# Discover all YAML files for parametrized validation tests
# ---------------------------------------------------------------------------


def _find_part_yamls() -> list[Path]:
    paths = []
    for robot_dir in ROBOTS_DIR.iterdir():
        if not robot_dir.is_dir():
            continue
        analysis_dir = robot_dir / "analysis"
        if not analysis_dir.exists():
            continue
        for p in sorted(analysis_dir.glob("*.yaml")):
            if p.name != "summary.yaml":
                paths.append(p)
    return paths


def _find_chain_yamls() -> list[Path]:
    return sorted(ROBOTS_DIR.glob("*/chain.yaml"))


def _find_summary_yamls() -> list[Path]:
    return sorted(ROBOTS_DIR.glob("*/analysis/summary.yaml"))


def _find_specs_yamls() -> list[Path]:
    return sorted(ROBOTS_DIR.glob("*/specs.yaml"))


def _find_view_mapping_yamls() -> list[Path]:
    return sorted(ROBOTS_DIR.glob("*/view_mapping.yaml"))


# ---------------------------------------------------------------------------
# Validation tests — all existing YAML files must parse with Pydantic
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("yaml_path", _find_part_yamls(), ids=lambda p: p.name)
def test_validate_part_yaml(yaml_path: Path) -> None:
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    PartAnalysis.model_validate(data)


@pytest.mark.parametrize("yaml_path", _find_chain_yamls(), ids=lambda p: p.parent.name)
def test_validate_chain_yaml(yaml_path: Path) -> None:
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    Chain.model_validate(data)


@pytest.mark.parametrize(
    "yaml_path", _find_summary_yamls(), ids=lambda p: p.parent.parent.name
)
def test_validate_summary_yaml(yaml_path: Path) -> None:
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    Summary.model_validate(data)


@pytest.mark.parametrize("yaml_path", _find_specs_yamls(), ids=lambda p: p.parent.name)
def test_validate_specs_yaml(yaml_path: Path) -> None:
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    Specs.model_validate(data)


@pytest.mark.parametrize(
    "yaml_path",
    _find_view_mapping_yamls(),
    ids=lambda p: p.parent.name,
)
def test_validate_view_mapping_yaml(yaml_path: Path) -> None:
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    ViewMapping.model_validate(data)


# ---------------------------------------------------------------------------
# Round-trip tests: load -> save -> reload -> compare
# ---------------------------------------------------------------------------


def test_round_trip_part_yaml(tmp_path: Path) -> None:
    part_yamls = _find_part_yamls()
    if not part_yamls:
        pytest.skip("No part YAML files found")
    src = part_yamls[0]
    model = load_part_yaml(src)
    out = tmp_path / "part.yaml"
    save_part_yaml(model, out)
    reloaded = load_part_yaml(out)
    assert model.model_dump() == reloaded.model_dump()


def test_round_trip_chain_yaml(tmp_path: Path) -> None:
    chain_yamls = _find_chain_yamls()
    if not chain_yamls:
        pytest.skip("No chain YAML files found")
    src = chain_yamls[0]
    model = load_chain_yaml(src)
    out = tmp_path / "chain.yaml"
    save_chain_yaml(model, out)
    reloaded = load_chain_yaml(out)
    assert model.model_dump() == reloaded.model_dump()


def test_round_trip_summary_yaml(tmp_path: Path) -> None:
    summaries = _find_summary_yamls()
    if not summaries:
        pytest.skip("No summary YAML files found")
    src = summaries[0]
    model = load_summary_yaml(src)
    out = tmp_path / "summary.yaml"
    save_summary_yaml(model, out)
    reloaded = load_summary_yaml(out)
    assert model.model_dump() == reloaded.model_dump()


def test_round_trip_specs_yaml(tmp_path: Path) -> None:
    specs = _find_specs_yamls()
    if not specs:
        pytest.skip("No specs YAML files found")
    src = specs[0]
    model = load_specs_yaml(src)
    out = tmp_path / "specs.yaml"
    save_specs_yaml(model, out)
    reloaded = load_specs_yaml(out)
    assert model.model_dump() == reloaded.model_dump()


def test_round_trip_view_mapping_yaml(tmp_path: Path) -> None:
    vms = _find_view_mapping_yamls()
    if not vms:
        pytest.skip("No view_mapping YAML files found")
    src = vms[0]
    model = load_view_mapping_yaml(src)
    out = tmp_path / "view_mapping.yaml"
    save_view_mapping_yaml(model, out)
    reloaded = load_view_mapping_yaml(out)
    assert model.model_dump() == reloaded.model_dump()


# ---------------------------------------------------------------------------
# Schema freshness — committed JSON schemas must match current models
# ---------------------------------------------------------------------------

_SCHEMA_MODELS = {
    "part_analysis": PartAnalysis,
    "chain": Chain,
    "summary": Summary,
    "specs": Specs,
    "view_mapping": ViewMapping,
}


@pytest.mark.parametrize("name,model_cls", _SCHEMA_MODELS.items(), ids=_SCHEMA_MODELS)
def test_schemas_are_up_to_date(name: str, model_cls: type) -> None:
    schema_path = SCHEMAS_DIR / f"{name}.json"
    if not schema_path.exists():
        pytest.fail(f"Missing schema file: {schema_path}")
    committed = json.loads(schema_path.read_text())
    current = model_cls.model_json_schema()
    assert committed == current, (
        f"Schema {name}.json is stale. Run 'uv run generate-schemas' to update."
    )
