"""Direct RealSense camera input for the object rendering pipeline."""

from dataclasses import dataclass

import numpy as np
import pyrealsense2 as rs

@dataclass
class CameraFrame:
    """One color frame plus the camera intrinsics needed for rendering."""

    bgr: np.ndarray
    camera_k: np.ndarray
    width: int
    height: int


class RealSenseCameraSource:
    """Small wrapper around a RealSense color stream."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.width = width
        self.height = height
        self.fps = fps

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.profile = None
        self.camera_k: np.ndarray | None = None

    def start(self) -> None:
        self.config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps,
        )

        self.profile = self.pipeline.start(self.config)

        color_profile = (
            self.profile
            .get_stream(rs.stream.color)
            .as_video_stream_profile()
        )
        intrinsics = color_profile.get_intrinsics()

        self.width = intrinsics.width
        self.height = intrinsics.height
        self.camera_k = np.array(
            [
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def read(self) -> CameraFrame | None:
        if self.camera_k is None:
            raise RuntimeError("Camera must be started before reading frames")
        
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            return None

        bgr = np.asanyarray(color_frame.get_data()).copy()

        return CameraFrame(
            bgr=bgr,
            camera_k=self.camera_k,
            width=self.width,
            height=self.height,
        )

    def stop(self) -> None:
        self.pipeline.stop()