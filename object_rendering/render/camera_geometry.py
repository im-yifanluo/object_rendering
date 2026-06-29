"""Camera geometry helpers for matching OpenCV and PyBullet cameras."""

import numpy as np
import pybullet as p


CAMERA_NEAR = 0.05
CAMERA_FAR = 10.0


def rt_to_transform(rotation, translation):
    """Build a homogeneous transform from rotation and translation."""
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float).reshape(3)

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    return transform


def invert_transform(transform):
    """Invert a rigid homogeneous transform."""
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation

    return inverse


def compute_projection_matrix(
    camera_k,
    width,
    height,
    near=CAMERA_NEAR,
    far=CAMERA_FAR,
):
    """Compute a PyBullet projection matrix from OpenCV intrinsics."""
    camera_k = np.asarray(camera_k, dtype=float)

    fx = camera_k[0, 0]
    fy = camera_k[1, 1]
    cx = camera_k[0, 2]
    cy = camera_k[1, 2]

    left = -cx * near / fx
    right = (width - cx) * near / fx
    bottom = -(height - cy) * near / fy
    top = cy * near / fy

    return p.computeProjectionMatrix(left, right, bottom, top, near, far)


def compute_view_matrix(rotation_cw_cv, translation_cw_cv):
    """Compute a PyBullet view matrix from an AprilTag camera pose."""
    # OpenCV camera coordinates -> OpenGL/PyBullet camera coordinates.
    cv_to_gl = np.diag([1.0, -1.0, -1.0])

    rotation_cw_gl = cv_to_gl @ np.asarray(rotation_cw_cv, dtype=float)
    translation_cw_gl = cv_to_gl @ np.asarray(
        translation_cw_cv,
        dtype=float,
    ).reshape(3)

    transform_cw_gl = rt_to_transform(rotation_cw_gl, translation_cw_gl)
    transform_wc_gl = invert_transform(transform_cw_gl)

    rotation_wc = transform_wc_gl[:3, :3]
    translation_wc = transform_wc_gl[:3, 3]

    eye = translation_wc
    forward = -rotation_wc[:, 2]
    up = rotation_wc[:, 1]
    target = eye + forward

    return p.computeViewMatrix(eye.tolist(), target.tolist(), up.tolist())
