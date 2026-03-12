"""Generate JSON Schema files from Pydantic models.

Usage:
    uv run generate-schemas
"""

from __future__ import annotations

import json
from pathlib import Path

from robot_arm_sim.models import (
    Chain,
    PartAnalysis,
    Specs,
    Summary,
    ViewMapping,
)

SCHEMAS = {
    "part_analysis": PartAnalysis,
    "chain": Chain,
    "summary": Summary,
    "specs": Specs,
    "view_mapping": ViewMapping,
}


def generate() -> None:
    """Generate JSON Schema files into schemas/ at repo root."""
    # Find repo root (directory containing pyproject.toml)
    root = Path(__file__).resolve().parent.parent.parent
    if not (root / "pyproject.toml").exists():
        # Fallback: cwd
        root = Path.cwd()

    schemas_dir = root / "schemas"
    schemas_dir.mkdir(exist_ok=True)

    for name, model_cls in SCHEMAS.items():
        schema = model_cls.model_json_schema()
        out_path = schemas_dir / f"{name}.json"
        out_path.write_text(json.dumps(schema, indent=2) + "\n")
        print(f"Wrote {out_path}")

    print(f"Generated {len(SCHEMAS)} schemas in {schemas_dir}")
