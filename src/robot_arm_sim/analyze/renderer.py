"""Off-screen rendering of mesh parts to PNG images using pyrender + EGL."""

from __future__ import annotations

import logging
import os
from pathlib import Path

import numpy as np
import trimesh

logger = logging.getLogger(__name__)

# Use EGL for headless rendering (must be set before pyrender import)
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

# Camera angles: name -> (elevation_deg, azimuth_deg)
VIEWS = {
    "front": (0, 0),
    "side": (0, 90),
    "top": (90, 0),
    "iso": (35, 45),
}


def _camera_pose(elev_deg: float, azim_deg: float, distance: float) -> np.ndarray:
    """Compute a 4x4 camera pose looking at the origin."""
    elev = np.radians(elev_deg)
    azim = np.radians(azim_deg)

    # Camera position in spherical coordinates
    x = distance * np.cos(elev) * np.sin(azim)
    y = distance * np.cos(elev) * np.cos(azim)
    z = distance * np.sin(elev)
    cam_pos = np.array([x, y, z])

    # Look-at matrix (camera looks at origin, up = Z)
    forward = -cam_pos / np.linalg.norm(cam_pos)
    world_up = np.array([0.0, 0.0, 1.0])

    # Handle degenerate case when looking straight down/up
    if abs(np.dot(forward, world_up)) > 0.99:
        world_up = np.array([0.0, 1.0, 0.0])

    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)

    pose = np.eye(4)
    pose[:3, 0] = right
    pose[:3, 1] = up
    pose[:3, 2] = -forward
    pose[:3, 3] = cam_pos
    return pose


def render_views(
    mesh: trimesh.Trimesh,
    output_dir: Path,
    part_name: str,
    resolution: tuple[int, int] = (800, 600),
) -> list[str]:
    """Render multiple views of a mesh to PNG files.

    Returns list of relative paths to rendered images.
    Falls back gracefully if rendering is unavailable.
    """
    try:
        import pyrender
    except Exception as e:
        logger.warning(f"pyrender unavailable, skipping renders: {e}")
        return []

    output_dir.mkdir(parents=True, exist_ok=True)
    rendered_paths: list[str] = []

    # Center the mesh at the origin for consistent camera framing
    centroid = mesh.centroid
    centered = mesh.copy()
    centered.vertices -= centroid

    # Build pyrender mesh
    pr_mesh = pyrender.Mesh.from_trimesh(centered)
    distance = float(np.max(centered.extents)) * 2.5

    for view_name, (elev, azim) in VIEWS.items():
        filename = f"{part_name}_{view_name}.png"
        filepath = output_dir / filename

        try:
            scene = pyrender.Scene(
                ambient_light=np.array([0.3, 0.3, 0.3, 1.0]),
            )
            scene.add(pr_mesh)

            # Camera
            camera = pyrender.PerspectiveCamera(yfov=np.radians(45))
            cam_pose = _camera_pose(elev, azim, distance)
            scene.add(camera, pose=cam_pose)

            # Light
            light = pyrender.DirectionalLight(color=np.ones(3), intensity=3.0)
            scene.add(light, pose=cam_pose)

            # Render
            renderer = pyrender.OffscreenRenderer(*resolution)
            try:
                color, _ = renderer.render(scene)
            finally:
                renderer.delete()

            # Save as PNG
            import imageio

            imageio.imwrite(str(filepath), color)
            rendered_paths.append(f"renders/{filename}")
        except Exception as e:
            logger.warning(f"Could not render {view_name} view for {part_name}: {e}")

    return rendered_paths
