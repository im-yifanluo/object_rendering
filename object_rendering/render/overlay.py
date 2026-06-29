"""Image compositing helpers for rendered RGBA overlays."""

import cv2
import numpy as np


def overlay_rgba_on_bgr(bg_bgr, fg_rgba):
    """Blend an RGBA foreground over an OpenCV BGR background."""
    bg_bgr = np.asarray(bg_bgr)
    fg_rgba = np.asarray(fg_rgba)

    if fg_rgba.shape[-1] != 4:
        raise ValueError("fg_rgba must have 4 channels")

    if fg_rgba.shape[:2] != bg_bgr.shape[:2]:
        fg_rgba = cv2.resize(
            fg_rgba,
            (bg_bgr.shape[1], bg_bgr.shape[0]),
            interpolation=cv2.INTER_AREA,
        )

    fg_rgb = fg_rgba[:, :, :3].astype(float)
    alpha = fg_rgba[:, :, 3].astype(float) / 255.0

    bg_rgb = cv2.cvtColor(bg_bgr, cv2.COLOR_BGR2RGB).astype(float)

    blended_rgb = alpha[..., None] * fg_rgb + (1.0 - alpha[..., None]) * bg_rgb
    blended_rgb = np.clip(blended_rgb, 0, 255).astype(np.uint8)

    return cv2.cvtColor(blended_rgb, cv2.COLOR_RGB2BGR)
