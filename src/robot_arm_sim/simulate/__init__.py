"""Simulation module: URDF loading, kinematics, and NiceGUI app."""

from __future__ import annotations

from pathlib import Path


def run_simulator(robot_dir: Path, port: int = 8080) -> None:
    """Launch the NiceGUI simulator for a robot."""
    from .app import create_app

    create_app(robot_dir, port=port)


def main() -> None:
    """Standalone entry point for the simulate command."""
    import typer

    app = typer.Typer()

    @app.command()
    def simulate(
        robot_dir: Path = typer.Argument(..., help="Path to robot directory"),
        port: int = typer.Option(8080, help="Port for the web UI"),
    ) -> None:
        run_simulator(robot_dir, port=port)

    app()
