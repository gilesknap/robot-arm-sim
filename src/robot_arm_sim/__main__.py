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
def generate(
    robot_dir: Path = typer.Argument(
        ...,
        help="Path to robot directory (must contain analysis/).",
        exists=True,
        file_okay=False,
        dir_okay=True,
    ),
    chain_file: Path = typer.Argument(
        ...,
        help="Path to chain.yaml kinematic chain specification.",
        exists=True,
        file_okay=True,
        dir_okay=False,
    ),
    output: Path = typer.Option(
        None,
        help="Output URDF path (default: <robot_dir>/robot.urdf).",
    ),
) -> None:
    """Generate URDF from kinematic chain spec + analysis data."""
    from .analyze.urdf_generator import generate_urdf

    analysis_dir = robot_dir / "analysis"
    stl_dir = robot_dir / "stl_files"
    output_path = output or (robot_dir / "robot.urdf")

    typer.echo(f"Generating URDF from {chain_file}...")
    messages = generate_urdf(chain_file, analysis_dir, stl_dir, output_path)
    for msg in messages:
        typer.echo(msg)
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
