"""ROS node that publishes simulated object overlays on camera frames."""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

# # Legacy Script
# from cv_bridge import CvBridge
# from rclpy.qos import qos_profile_sensor_data
# from sensor_msgs.msg import CameraInfo, Image

from object_rendering.render.apriltag import AprilTagDetector
from object_rendering.render.camera_geometry import (
    compute_projection_matrix,
    compute_view_matrix,
)
from object_rendering.render.overlay import overlay_rgba_on_bgr
from object_rendering.render.simulation_renderer import PyBulletSceneRenderer

from object_rendering.io.display import OpenCvDisplaySink
from object_rendering.io.realsense_camera import RealSenseCameraSource


class SceneRendererNode(Node):
    """Coordinate camera input, tag detection, render, and overlay output."""

    def __init__(self):
        super().__init__("scene_renderer")

        # # Legacy Script
        # self.bridge = CvBridge()

        # # Declare communication parameters.
        # self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        # self.declare_parameter(
        #     "camera_info_topic",
        #     "/camera/camera/color/camera_info",
        # )
        # self.declare_parameter(
        #     "output_topic",
        #     "/object_rendering/overlay_image",
        # )
        
        # Declare parameters for camera settings and display options
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("display_enabled", True)
        self.declare_parameter("display_window_name", "object_rendering")
        
        self.camera_width = self.get_parameter("camera_width").value
        self.camera_height = self.get_parameter("camera_height").value
        self.camera_fps = self.get_parameter("camera_fps").value
        self.display_enabled = self.get_parameter("display_enabled").value
        self.display_window_name = self.get_parameter("display_window_name").value

        # Declare parameters for assets root, task name, and scene file
        self.declare_parameter("assets_root", "")
        self.declare_parameter("task_name", "grasping_in_clutter")
        self.declare_parameter("scene_file", "0.npz")

        # Declare parameters for renderer and overlay alpha
        self.declare_parameter("renderer", "tiny")
        self.declare_parameter("overlay_alpha", 0.6)
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("tag_size", 0.12)

        # Set up the assets root directory, task name, and scene file
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

        # Set up the AprilTag detector
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

        self.logged_first_image = False

        # Initialize the PyBullet scene renderer and load the scene
        self.scene_renderer = PyBulletSceneRenderer(
            self.models_dir,
            renderer=self.renderer_name,
        )
        self.scene_renderer.load_scene(self.scene_path)
        
        # Set up the RealSense camera source and OpenCV display sink
        self.camera = RealSenseCameraSource(
            width=self.camera_width,
            height=self.camera_height,
            fps=self.camera_fps,
        )
        self.camera.start()

        self.display = OpenCvDisplaySink(
            window_name=self.display_window_name,
            enabled=self.display_enabled,
        )
        self.display.start()
        
        timer_period = 1.0 / float(self.camera_fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # # Legacy ROS 2 code for subscriptions and publisher
        
        # self.image_topic = self.get_parameter("image_topic").value
        # self.camera_info_topic = self.get_parameter("camera_info_topic").value
        # self.output_topic = self.get_parameter("output_topic").value
        
        # # Set up subscriptions and publisher
        # self.camera_info_sub = self.create_subscription(
        #     CameraInfo,
        #     self.camera_info_topic,
        #     self.camera_info_callback,
        #     qos_profile_sensor_data,
        # )

        # self.image_sub = self.create_subscription(
        #     Image,
        #     self.image_topic,
        #     self.image_callback,
        #     qos_profile_sensor_data,
        # )

        # self.overlay_pub = self.create_publisher(
        #     Image,
        #     self.output_topic,
        #     qos_profile_sensor_data,
        # )

        # self.get_logger().info(f"Subscribing to image: {self.image_topic}")
        # self.get_logger().info(
        #     f"Subscribing to camera info: {self.camera_info_topic}"
        # )
        # self.get_logger().info(f"Overlay output will be: {self.output_topic}")

    def process_frame(self, frame):
        """Render an overlay for one RealSense camera frame."""
        cv_image = frame.bgr
        camera_k = frame.camera_k
        camera_width = frame.width
        camera_height = frame.height

        if not self.logged_first_image:
            self.get_logger().info(
                f"RealSense frame received: width={camera_width}, "
                f"height={camera_height}, K={camera_k.tolist()}"
            )
            self.logged_first_image = True

        detection = self.apriltag_detector.detect(cv_image, camera_k)

        if detection is None:
            if not self.logged_missing_tag:
                self.get_logger().warning("No AprilTag detected.")
                self.logged_missing_tag = True
            return cv_image

        self.logged_missing_tag = False

        projection_matrix = compute_projection_matrix(
            camera_k,
            camera_width,
            camera_height,
        )

        view_matrix = compute_view_matrix(
            detection.rotation,
            detection.translation,
        )

        rendered_rgba = self.scene_renderer.render(
            camera_width,
            camera_height,
            view_matrix,
            projection_matrix,
            alpha=self.overlay_alpha,
        )

        overlay_bgr = overlay_rgba_on_bgr(cv_image, rendered_rgba)

        if not self.logged_first_tag:
            self.get_logger().info(
                f"AprilTag detected: id={detection.tag_id}, "
                f"translation={detection.translation.tolist()}, "
                f"decision_margin={detection.decision_margin}"
            )
            self.get_logger().info(f"Projection matrix: {projection_matrix}")
            self.get_logger().info(f"View matrix: {view_matrix}")
            self.get_logger().info(f"Rendered RGBA shape: {rendered_rgba.shape}")
            self.logged_first_tag = True

        return overlay_bgr
    
    def timer_callback(self):
        """Read one camera frame, render it, and display the result."""
        frame = self.camera.read()

        if frame is None:
            self.get_logger().warning("No RealSense color frame received.")
            return

        output_bgr = self.process_frame(frame)
        keep_running = self.display.show(output_bgr)

        if not keep_running:
            rclpy.shutdown()
            
    def destroy_node(self):
        """Release camera and display resources before shutting down."""
        self.camera.stop()
        self.display.close()
        self.scene_renderer.disconnect()
        super().destroy_node()
        

    # # ========================== Legacy ROS 2 Callbacks ==========================
    # def camera_info_callback(self, msg):
    #     """Store the latest camera intrinsic matrix from CameraInfo."""
    #     self.camera_k = np.array(msg.k, dtype=float).reshape(3, 3)
    #     self.camera_width = msg.width
    #     self.camera_height = msg.height

    # def camera_callback(self, msg):
    #     """Render and publish an overlay for each incoming camera frame."""
    #     # Check if camera info has been received
    #     if self.camera_k is None:
    #         self.get_logger().warning(
    #             "Waiting for CameraInfo before processing images."
    #         )
    #         return

    #     # Log the first received image and camera info
    #     if not self.logged_first_image:
    #         self.get_logger().info(
    #             f"Image received: width={msg.width}, height={msg.height}, "
    #             f"encoding={msg.encoding}"
    #         )
    #         self.get_logger().info(
    #             f"CameraInfo received: width={self.camera_width}, "
    #             f"height={self.camera_height}, K={self.camera_k.tolist()}"
    #         )
    #         self.logged_first_image = True

    #     # Convert ROS Image message to OpenCV image
    #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    #     # Detect AprilTag in the image
    #     detection = self.apriltag_detector.detect(cv_image, self.camera_k)

    #     if detection is None:
    #         # Publish the original image until a tag is visible.
    #         if not self.logged_missing_tag:
    #             self.get_logger().warning("No AprilTag detected.")
    #             self.logged_missing_tag = True
    #         output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    #         output_msg.header = msg.header
    #         self.overlay_pub.publish(output_msg)
    #         return

    #     self.logged_missing_tag = False

    #     # Compute camera matrices from CameraInfo and AprilTag pose.
    #     projection_matrix = compute_projection_matrix(
    #         self.camera_k,
    #         self.camera_width,
    #         self.camera_height,
    #     )

    #     view_matrix = compute_view_matrix(
    #         detection.rotation,
    #         detection.translation,
    #     )

    #     # Render the simulated scene and overlay it on the camera frame.
    #     rendered_rgba = self.scene_renderer.render(
    #         self.camera_width,
    #         self.camera_height,
    #         view_matrix,
    #         projection_matrix,
    #         alpha=self.overlay_alpha,
    #     )

    #     # Overlay the rendered RGBA image on the original BGR image
    #     overlay_bgr = overlay_rgba_on_bgr(cv_image, rendered_rgba)

    #     if not self.logged_first_tag:
    #         self.get_logger().info(
    #             f"AprilTag detected: id={detection.tag_id}, "
    #             f"translation={detection.translation.tolist()}, "
    #             f"decision_margin={detection.decision_margin}"
    #         )
    #         self.get_logger().info(f"Projection matrix: {projection_matrix}")
    #         self.get_logger().info(f"View matrix: {view_matrix}")
    #         self.get_logger().info(
    #             f"Rendered RGBA shape: {rendered_rgba.shape}"
    #         )
    #         self.logged_first_tag = True

    #     output_msg = self.bridge.cv2_to_imgmsg(overlay_bgr, encoding="bgr8")
    #     output_msg.header = msg.header

    #     self.overlay_pub.publish(output_msg)

        

def main(args=None):
    """Start the scene renderer node."""
    rclpy.init(args=args)

    node = SceneRendererNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
