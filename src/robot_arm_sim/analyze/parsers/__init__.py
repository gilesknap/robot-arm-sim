"""Parser registry for mesh file formats."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from .stl_parser import STLParser

if TYPE_CHECKING:
    from .base import AbstractMeshParser

_PARSERS: dict[str, type[AbstractMeshParser]] = {
    ".stl": STLParser,
}


def get_parser(file_path: Path) -> AbstractMeshParser:
    """Get the appropriate parser for a mesh file."""
    ext = file_path.suffix.lower()
    parser_cls = _PARSERS.get(ext)
    if parser_cls is None:
        supported = ", ".join(_PARSERS.keys())
        raise ValueError(f"Unsupported format '{ext}'. Supported: {supported}")
    return parser_cls()
