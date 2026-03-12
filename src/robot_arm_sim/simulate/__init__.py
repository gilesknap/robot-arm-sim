"""Simulation module: URDF loading, kinematics, and NiceGUI app."""

from __future__ import annotations

from pathlib import Path


def run_simulator(robots_dir: Path, port: int = 8080) -> None:
    """Launch the NiceGUI simulator for robots in *robots_dir*."""
    from .app import create_app

    create_app(robots_dir, port=port)


def main() -> None:
    """Standalone entry point for the simulate command."""
    import typer

    app = typer.Typer()

    @app.command()
    def simulate(
        robots_dir: Path = typer.Argument(
            ..., help="Path to directory containing robot folders"
        ),
        port: int = typer.Option(8080, help="Port for the web UI"),
    ) -> None:
        run_simulator(robots_dir, port=port)

    app()
