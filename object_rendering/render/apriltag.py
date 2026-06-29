"""AprilTag detection helpers for camera frames."""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from pupil_apriltags import Detector


APRILTAG_SIZE = 0.12
APRILTAG_FAMILY = "tag36h11"


@dataclass
class AprilTagDetection:
    """Pose and metadata for one detected AprilTag."""

    tag_id: int
    corners: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    decision_margin: float


class AprilTagDetector:
    """Reusable wrapper around pupil_apriltags.Detector."""

    def __init__(self, tag_family=APRILTAG_FAMILY, tag_size=APRILTAG_SIZE):
        self.tag_family = tag_family
        self.tag_size = tag_size
        self.detector = Detector(families=tag_family)

    def detect(self, bgr_image, camera_k) -> Optional[AprilTagDetection]:
        """Detect the best AprilTag in a BGR image."""
        camera_k = np.asarray(camera_k, dtype=float)

        if camera_k.shape != (3, 3):
            raise ValueError("camera_k must be a 3x3 camera intrinsic matrix")

        fx = camera_k[0, 0]
        fy = camera_k[1, 1]
        cx = camera_k[0, 2]
        cy = camera_k[1, 2]

        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(fx, fy, cx, cy),
            tag_size=self.tag_size,
        )

        if not detections:
            return None

        detection = max(
            detections,
            key=lambda det: getattr(det, "decision_margin", 0.0),
        )

        rotation = detection.pose_R
        translation = np.asarray(detection.pose_t, dtype=float).reshape(3)

        world_flip = np.diag([1.0, -1.0, -1.0])
        rotation = rotation @ world_flip.T

        return AprilTagDetection(
            tag_id=getattr(detection, "tag_id", 0),
            corners=np.asarray(detection.corners, dtype=np.int32),
            rotation=rotation,
            translation=translation,
            decision_margin=getattr(detection, "decision_margin", 0.0),
        )
