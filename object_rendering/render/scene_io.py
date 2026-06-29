"""Asset and scene loading helpers for MNet-style object scenes."""

from pathlib import Path

import numpy as np


WORLD_OFFSET = np.array([0.35, 0.35, 0.0])


def build_model_library(models_dir):
    """Map object names to their model.urdf paths."""
    models_dir = Path(models_dir)
    if not models_dir.is_dir():
        raise FileNotFoundError(
            f"Models directory does not exist: {models_dir}"
        )

    model_library = {}

    for urdf_path in sorted(models_dir.glob("*/model.urdf")):
        object_name = urdf_path.parent.name
        model_library[object_name] = urdf_path

    if not model_library:
        raise FileNotFoundError(
            f"No model.urdf files found under: {models_dir}"
        )

    return model_library


def load_scene_npz(scene_path):
    """Load model names and poses from an MNet-style scene npz file."""
    scene_path = Path(scene_path)
    if not scene_path.is_file():
        raise FileNotFoundError(f"Scene file does not exist: {scene_path}")

    data = np.load(scene_path, allow_pickle=True)
    required_keys = {"model_names", "poses"}
    missing_keys = required_keys.difference(data.files)
    if missing_keys:
        raise ValueError(
            f"Scene file {scene_path} is missing keys: "
            f"{sorted(missing_keys)}"
        )

    return {
        "model_names": data["model_names"],
        "poses": data["poses"],
    }
