"""OpenCV display output for the object rendering pipeline."""

import cv2
import numpy as np


class OpenCvDisplaySink:
    """Small wrapper around an OpenCV display window."""

    def __init__(self, window_name: str = "object_rendering", enabled: bool = True):
        self.window_name = window_name
        self.enabled = enabled
        self.started = False

    def start(self) -> None:
        if not self.enabled:
            return

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.started = True

    def show(self, bgr_image: np.ndarray) -> bool:
        if not self.enabled:
            return True

        if not self.started:
            self.start()

        cv2.imshow(self.window_name, bgr_image)

        key = cv2.waitKey(1) & 0xFF
        return key not in (ord("q"), 27)

    def close(self) -> None:
        if self.enabled and self.started:
            cv2.destroyWindow(self.window_name)
            self.started = False