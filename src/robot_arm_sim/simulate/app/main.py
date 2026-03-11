"""Main app entry point — creates the NiceGUI simulator."""

from __future__ import annotations

from pathlib import Path

from nicegui import app, ui

from ..urdf_loader import load_urdf
from .controls import build_controls_panel
from .edit_bores import build_edit_bores
from .scene_objects import build_scene
from .state import SimulatorState
from .toolbar import build_toolbar, build_visibility_panel


def create_app(robot_dir: Path, port: int = 8080) -> None:
    """Create and run the NiceGUI simulator app."""
    urdf_path = robot_dir / "robot.urdf"
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"No robot.urdf found in {robot_dir}. "
            "Run the assembly-reasoning skill first."
        )

    # Serve STL files
    stl_dir = robot_dir / "stl_files"
    if stl_dir.exists():
        app.add_static_files("/stl", str(stl_dir))

    @app.get("/healthz")
    async def healthz():
        return {"status": "ok"}

    @ui.page("/")
    def index():
        robot = load_urdf(urdf_path)
        _build_ui(robot, robot_dir)

    ui.run(
        port=port,
        title=f"Robot Sim: {robot_dir.name}",
        reload=False,
        show=False,
        reconnect_timeout=30,
    )


def _build_ui(robot, robot_dir):
    """Build the simulator UI."""
    state = SimulatorState(robot, robot_dir)

    ui.label(f"Robot: {robot.name}").classes("text-h4")

    with ui.row().style("flex-wrap: nowrap; gap: 0"):
        # Left panel: 3D scene + toolbar
        with ui.column().style("gap: 0"):
            build_scene(state)
            build_toolbar(state)
            build_visibility_panel(state)
            build_edit_bores(state)

        # Right panel: controls
        with ui.column().classes("p-4").style("width: 280px; overflow-y: auto"):
            build_controls_panel(state)
