"""ROS node that publishes simulated object overlays on camera frames."""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from object_rendering.render.apriltag import AprilTagDetector
from object_rendering.render.camera_geometry import (
    compute_projection_matrix,
    compute_view_matrix,
)
from object_rendering.render.overlay import overlay_rgba_on_bgr
from object_rendering.render.simulation_renderer import PyBulletSceneRenderer


class SceneRendererNode(Node):
    """Coordinate camera input, tag detection, render, and overlay output."""

    def __init__(self):
        super().__init__("scene_renderer")

        self.bridge = CvBridge()

        # Declare communication parameters.
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info",
        )
        self.declare_parameter(
            "output_topic",
            "/object_rendering/overlay_image",
        )

        # Declare parameters for assets root, task name, and scene file
        self.declare_parameter("assets_root", "")
        self.declare_parameter("task_name", "grasping_in_clutter")
        self.declare_parameter("scene_file", "0.npz")

        # Declare parameters for renderer and overlay alpha
        self.declare_parameter("renderer", "tiny")
        self.declare_parameter("overlay_alpha", 0.6)
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("tag_size", 0.12)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        assets_root = self.get_parameter("assets_root").value
        if assets_root:
            self.assets_root = Path(assets_root)
        else:
            package_share = Path(
                get_package_share_directory("object_rendering")
            )
            self.assets_root = package_share / "assets" / "testing"

        self.task_name = self.get_parameter("task_name").value
        self.scene_file = self.get_parameter("scene_file").value
        self.renderer_name = self.get_parameter("renderer").value
        self.overlay_alpha = self.get_parameter("overlay_alpha").value
        self.tag_family = self.get_parameter("tag_family").value
        self.tag_size = self.get_parameter("tag_size").value

        self.apriltag_detector = AprilTagDetector(
            tag_family=self.tag_family,
            tag_size=self.tag_size,
        )
        self.logged_first_tag = False
        self.logged_missing_tag = False

        task_assets_dir = self.assets_root / self.task_name
        self.models_dir = task_assets_dir / "models"
        self.scene_path = task_assets_dir / "scenes" / self.scene_file

        self.get_logger().info(f"Models dir: {self.models_dir}")
        self.get_logger().info(f"Scene path: {self.scene_path}")
        self.get_logger().info(f"Assets root: {self.assets_root}")
        self.get_logger().info(f"Task name: {self.task_name}")
        self.get_logger().info(f"Scene file: {self.scene_file}")
        self.get_logger().info(
            f"AprilTag family: {self.tag_family}, size: {self.tag_size} m"
        )

        self.camera_k = None
        self.camera_width = None
        self.camera_height = None
        self.logged_first_image = False

        # Initialize the PyBullet scene renderer and load the scene
        self.scene_renderer = PyBulletSceneRenderer(
            self.models_dir,
            renderer=self.renderer_name,
        )
        self.scene_renderer.load_scene(self.scene_path)

        # Set up subscriptions and publisher
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.overlay_pub = self.create_publisher(
            Image,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Subscribing to image: {self.image_topic}")
        self.get_logger().info(
            f"Subscribing to camera info: {self.camera_info_topic}"
        )
        self.get_logger().info(f"Overlay output will be: {self.output_topic}")

    def camera_info_callback(self, msg):
        """Store the latest camera intrinsic matrix from CameraInfo."""
        self.camera_k = np.array(msg.k, dtype=float).reshape(3, 3)
        self.camera_width = msg.width
        self.camera_height = msg.height

    def image_callback(self, msg):
        """Render and publish an overlay for each incoming camera frame."""
        # Check if camera info has been received
        if self.camera_k is None:
            self.get_logger().warning(
                "Waiting for CameraInfo before processing images."
            )
            return

        # Log the first received image and camera info
        if not self.logged_first_image:
            self.get_logger().info(
                f"Image received: width={msg.width}, height={msg.height}, "
                f"encoding={msg.encoding}"
            )
            self.get_logger().info(
                f"CameraInfo received: width={self.camera_width}, "
                f"height={self.camera_height}, K={self.camera_k.tolist()}"
            )
            self.logged_first_image = True

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Detect AprilTag in the image
        detection = self.apriltag_detector.detect(cv_image, self.camera_k)

        if detection is None:
            # Publish the original image until a tag is visible.
            if not self.logged_missing_tag:
                self.get_logger().warning("No AprilTag detected.")
                self.logged_missing_tag = True
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            output_msg.header = msg.header
            self.overlay_pub.publish(output_msg)
            return

        self.logged_missing_tag = False

        # Compute camera matrices from CameraInfo and AprilTag pose.
        projection_matrix = compute_projection_matrix(
            self.camera_k,
            self.camera_width,
            self.camera_height,
        )

        view_matrix = compute_view_matrix(
            detection.rotation,
            detection.translation,
        )

        # Render the simulated scene and overlay it on the camera frame.
        rendered_rgba = self.scene_renderer.render(
            self.camera_width,
            self.camera_height,
            view_matrix,
            projection_matrix,
            alpha=self.overlay_alpha,
        )

        # Overlay the rendered RGBA image on the original BGR image
        overlay_bgr = overlay_rgba_on_bgr(cv_image, rendered_rgba)

        if not self.logged_first_tag:
            self.get_logger().info(
                f"AprilTag detected: id={detection.tag_id}, "
                f"translation={detection.translation.tolist()}, "
                f"decision_margin={detection.decision_margin}"
            )
            self.get_logger().info(f"Projection matrix: {projection_matrix}")
            self.get_logger().info(f"View matrix: {view_matrix}")
            self.get_logger().info(
                f"Rendered RGBA shape: {rendered_rgba.shape}"
            )
            self.logged_first_tag = True

        output_msg = self.bridge.cv2_to_imgmsg(overlay_bgr, encoding="bgr8")
        output_msg.header = msg.header

        self.overlay_pub.publish(output_msg)


def main(args=None):
    """Start the scene renderer node."""
    rclpy.init(args=args)

    node = SceneRendererNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
