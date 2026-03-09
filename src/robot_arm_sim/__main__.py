"""Interface for ``python -m robot_arm_sim``."""

from __future__ import annotations

from pathlib import Path

import typer

from . import __version__

__all__ = ["main"]

app = typer.Typer(
    name="robot-arm-sim",
    help="Analyze robot arm CAD models and simulate them interactively.",
)


def _version_callback(value: bool) -> None:
    if value:
        typer.echo(f"robot-arm-sim {__version__}")
        raise typer.Exit()


@app.callback()
def common(
    version: bool | None = typer.Option(
        None,
        "--version",
        "-v",
        help="Show version and exit.",
        callback=_version_callback,
        is_eager=True,
    ),
) -> None:
    """Robot arm simulation pipeline."""


@app.command()
def analyze(
    robot_dir: Path = typer.Argument(
        ...,
        help="Path to robot directory (must contain stl_files/).",
        exists=True,
        file_okay=False,
        dir_okay=True,
    ),
) -> None:
    """Analyze STL files and generate part YAML + renders."""
    from .analyze import run_analysis

    typer.echo(f"Analyzing robot in {robot_dir}...")
    run_analysis(robot_dir)
    typer.echo("Done.")


@app.command()
def simulate(
    robot_dir: Path = typer.Argument(
        ...,
        help="Path to robot directory (must contain robot.urdf).",
        exists=True,
        file_okay=False,
        dir_okay=True,
    ),
    port: int = typer.Option(8080, help="Port for the web UI."),
) -> None:
    """Launch interactive 3D simulator for a robot arm."""
    from .simulate import run_simulator

    typer.echo(f"Starting simulator for {robot_dir} on port {port}...")
    run_simulator(robot_dir, port=port)


def main() -> None:
    app()


if __name__ == "__main__":
    main()
