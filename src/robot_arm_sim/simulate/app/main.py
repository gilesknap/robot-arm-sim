"""Main app entry point — creates the NiceGUI simulator."""

from __future__ import annotations

from pathlib import Path

from nicegui import app, ui

from ..urdf_loader import load_urdf
from .controls import build_controls_panel
from .edit_bores import build_edit_bores
from .js_snippets import BEFOREUNLOAD_JS
from .scene_objects import build_scene
from .state import SimulatorState
from .toolbar import build_toolbar, build_visibility_section


def _discover_robots(robots_dir: Path) -> dict[str, Path]:
    """Return {name: path} for every sub-dir containing robot.urdf."""
    found: dict[str, Path] = {}
    for child in sorted(robots_dir.iterdir()):
        if child.is_dir() and (child / "robot.urdf").exists():
            found[child.name] = child
    return found


def create_app(robots_dir: Path, port: int = 8080) -> None:
    """Create and run the NiceGUI simulator app.

    *robots_dir* is the parent directory whose sub-folders each contain
    a ``robot.urdf`` and associated assets.
    """
    robots = _discover_robots(robots_dir)
    if not robots:
        raise FileNotFoundError(
            f"No robot directories with robot.urdf found in {robots_dir}."
        )

    # Serve STL files for all robots under /stl/<robot_name>/
    for name, rdir in robots.items():
        stl_dir = rdir / "stl_files"
        if stl_dir.exists():
            app.add_static_files(f"/stl/{name}", str(stl_dir))

    default_robot = next(iter(robots))

    @app.get("/healthz")
    async def healthz():
        return {"status": "ok"}

    @ui.page("/")
    def index():
        _build_ui(robots, default_robot)

    @ui.page("/{robot_name}")
    def robot_page(robot_name: str):
        if robot_name not in robots:
            ui.label(f"Unknown robot: {robot_name}").classes("text-h5")
            return
        _build_ui(robots, robot_name)

    ui.run(
        port=port,
        title="Robot Simulator",
        reload=False,
        show=False,
        reconnect_timeout=30,
    )


_SCENE_WRAPPER_CSS = """
.sim-scene-wrapper {
    flex: 1 1 0;
    min-height: 0;
    position: relative;
    overflow: hidden;
    width: 100%;
}
"""


def _build_ui(robots: dict[str, Path], current_name: str) -> None:
    """Build the simulator UI for the selected robot."""
    robot_dir = robots[current_name]
    robot = load_urdf(robot_dir / "robot.urdf")
    state = SimulatorState(robot, robot_dir)

    # Fill viewport: h-screen on the content container, no default padding
    ui.context.client.content.classes("h-screen p-2").style("min-height: unset; gap: 0")
    ui.add_css(_SCENE_WRAPPER_CSS)

    # Title row with robot selector dropdown
    with ui.row().classes("items-center").style("gap: 12px"):
        ui.label("Robot:").classes("text-h4")
        ui.select(
            options=list(robots.keys()),
            value=current_name,
            on_change=lambda e: ui.navigate.to(f"/{e.value}"),
        ).props("dense outlined").style("min-width: 180px")

    with (
        ui.row()
        .classes("w-full h-full")
        .style("flex: 1; min-height: 0; flex-wrap: nowrap; gap: 0")
    ):
        # Left panel: 3D scene + toolbar
        with ui.column().classes("h-full").style("flex: 1; gap: 0; min-width: 0"):
            build_scene(state)
            build_toolbar(state)
            build_edit_bores(state)

        # Right panel: controls + visibility
        with (
            ui.column()
            .classes("p-4 h-full")
            .style("width: 280px; flex-shrink: 0; overflow-y: auto")
        ):
            build_controls_panel(state)
            build_visibility_section(state)

    # Register beforeunload handler for state persistence
    ui.run_javascript(BEFOREUNLOAD_JS)
