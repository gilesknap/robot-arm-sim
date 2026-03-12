"""Abstract base class for mesh parsers."""

from __future__ import annotations

import abc
from pathlib import Path

from robot_arm_sim.models import PartAnalysis


class AbstractMeshParser(abc.ABC):
    """Base class for mesh file parsers."""

    @abc.abstractmethod
    def supported_extensions(self) -> list[str]:
        """Return list of supported file extensions (e.g. ['.stl'])."""

    @abc.abstractmethod
    def parse(self, file_path: Path) -> PartAnalysis:
        """Parse a mesh file and return analysis results."""
