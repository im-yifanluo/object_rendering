"""PyBullet scene renderer for MNet-style object assets."""

from pathlib import Path

import numpy as np
import pybullet as p

from object_rendering.render.scene_io import (
    WORLD_OFFSET,
    build_model_library,
    load_scene_npz,
)

DEFAULT_RENDERER = "tiny"

PYBULLET_RENDERERS = {
    "tiny": p.ER_TINY_RENDERER,
    "cpu": p.ER_TINY_RENDERER,
    "opengl": p.ER_BULLET_HARDWARE_OPENGL,
    "gpu": p.ER_BULLET_HARDWARE_OPENGL,
}


def resolve_renderer(renderer):
    """Resolve a renderer name or integer constant to a PyBullet constant."""
    if isinstance(renderer, int):
        return renderer

    renderer = renderer.lower()

    try:
        return PYBULLET_RENDERERS[renderer]
    except KeyError as exc:
        valid = ", ".join(sorted(PYBULLET_RENDERERS))
        raise ValueError(
            f"Unknown renderer '{renderer}'. Valid options: {valid}"
        ) from exc


def reshape_rgba(arr, height, width):
    """Normalize a PyBullet RGBA buffer to an image array."""
    arr = np.asarray(arr)
    if arr.ndim == 3 and arr.shape[:2] == (height, width):
        return arr.astype(np.uint8, copy=False)
    return arr.reshape(height, width, 4).astype(np.uint8)


def reshape_segmentation(arr, height, width):
    """Normalize a PyBullet segmentation buffer to an image array."""
    arr = np.asarray(arr)
    if arr.ndim == 2 and arr.shape == (height, width):
        return arr.astype(np.int32, copy=False)
    return arr.reshape(height, width).astype(np.int32)


def make_transparent(rgb, segmentation, alpha=0.6):
    """Convert PyBullet color and segmentation outputs into RGBA."""
    rgb = np.asarray(rgb)
    segmentation = np.asarray(segmentation)

    if rgb.shape[-1] == 4:
        rgb = rgb[:, :, :3]

    alpha_channel = np.where(
        segmentation == -1,
        0,
        255 * alpha,
    ).astype(np.uint8)
    return np.dstack((rgb, alpha_channel)).astype(np.uint8)


class PyBulletSceneRenderer:
    """Stateful PyBullet renderer for a loaded object scene."""

    def __init__(self, models_dir, renderer=DEFAULT_RENDERER):
        self.client_id = p.connect(p.DIRECT)
        if self.client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet in DIRECT mode")

        self.models_dir = Path(models_dir)
        self.model_library = build_model_library(self.models_dir)
        self.renderer = resolve_renderer(renderer)

    def load_scene(self, scene_path):
        """Load object URDFs for a scene npz file into PyBullet."""
        p.resetSimulation(physicsClientId=self.client_id)

        scene = load_scene_npz(scene_path)
        model_names = scene["model_names"]
        poses = scene["poses"]

        for model_name, pose in zip(model_names, poses):
            model_name = str(model_name)
            try:
                urdf_path = self.model_library[model_name]
            except KeyError as exc:
                raise ValueError(
                    f"Scene references unknown model '{model_name}'. "
                    f"Known models: {sorted(self.model_library)}"
                ) from exc

            p.loadURDF(
                str(urdf_path),
                basePosition=(
                    np.asarray(pose[:3], dtype=float) + WORLD_OFFSET
                ).tolist(),
                baseOrientation=np.asarray(pose[3:], dtype=float).tolist(),
                useFixedBase=True,
                physicsClientId=self.client_id,
            )

    def render(self, width, height, view_matrix, projection_matrix, alpha=0.6):
        """Render the loaded scene as a transparent RGBA image."""
        image = p.getCameraImage(
            width,
            height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            shadow=0,
            renderer=self.renderer,
            physicsClientId=self.client_id,
        )

        rgb = reshape_rgba(image[2], height, width)
        segmentation = reshape_segmentation(image[4], height, width)

        return make_transparent(rgb, segmentation, alpha=alpha)

    def disconnect(self):
        """Disconnect this renderer from its PyBullet client."""
        p.disconnect(physicsClientId=self.client_id)
