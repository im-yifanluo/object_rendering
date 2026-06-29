# object_rendering

`object_rendering` is a ROS 2 Python package for rendering simulated object
placement guides over a live RGB camera stream.

The package is intentionally hardware-agnostic:

- camera drivers live outside this package, for example `realsense2_camera`
- display tools live outside this package, for example `rqt_image_view`, RViz, or
  `image_view`
- this package consumes camera topics, computes the overlay, and publishes an
  output image topic

The current pipeline is:

```text
sensor_msgs/Image + sensor_msgs/CameraInfo
  -> OpenCV image conversion
  -> AprilTag detection
  -> camera projection/view matrix computation
  -> PyBullet scene rendering
  -> transparent render creation from PyBullet segmentation
  -> RGBA-over-camera alpha blend
  -> sensor_msgs/Image overlay output
```

## Relationship To ManipulationNet

This package adapts only the rendering-related methodology from the local
ManipulationNet reference under `ref/mnet_client`.

Reused concepts:

- AprilTag pose estimation with `pupil-apriltags`
- camera intrinsics to PyBullet projection matrix
- AprilTag pose to PyBullet view matrix
- MNet model and scene asset layout
- PyBullet `getCameraImage` rendering
- segmentation-mask-to-transparent-RGBA conversion
- RGBA-over-camera alpha blending

Intentionally not included:

- server/client submission code
- team code or one-time code handling
- scoring services
- video hashing or upload logic
- `BaseClient`, `SubmissionClient`, or `LocalTestClient`

Reference files:

- `ref/mnet_client/mnet_client/tasks/grasping_in_clutter.py`
- `ref/mnet_client/mnet_client/base/base_client.py`

## Package Layout

```text
object_rendering/
  nodes/
    scene_renderer_node.py
  render/
    apriltag.py
    camera_geometry.py
    overlay.py
    scene_io.py
    simulation_renderer.py
assets/
  testing/
    grasping_in_clutter/
      models/
      scenes/
    tabletop_manipulation/
      models/
      scenes/
```

`nodes/` contains ROS-facing code: parameters, subscriptions, publishers, and
logging.

`render/` contains reusable Python computation code. These modules should not
depend on ROS messages.

## Runtime Topics

Default input topics assume a default RealSense ROS launch:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

Default output topic:

```text
/object_rendering/overlay_image
```

Topic names are parameters, so they can be changed for Rocky, a rosbag, or a
different camera.

## Parameters

`scene_renderer` declares these parameters:

```text
image_topic          default: /camera/camera/color/image_raw
camera_info_topic    default: /camera/camera/color/camera_info
output_topic         default: /object_rendering/overlay_image
assets_root          default: package share directory/assets/testing
task_name            default: grasping_in_clutter
scene_file           default: 0.npz
renderer             default: tiny
overlay_alpha        default: 0.6
tag_family           default: tag36h11
tag_size             default: 0.12
```

`renderer` accepts:

```text
tiny, cpu     -> PyBullet TinyRenderer
opengl, gpu   -> PyBullet hardware OpenGL renderer
```

Start with `tiny`. It matches the MNet reference and is the most portable for
headless robot machines.

## Assets

Models describe individual objects:

```text
models/<object_name>/
  model.urdf
  textured_simple.obj
  textured_simple.obj.mtl
  texture_map.png
```

Scenes describe object arrangements:

```text
scenes/<scene>.npz
```

Each scene contains:

```text
model_names
poses  # [x, y, z, qx, qy, qz, qw]
```

The package installs files under `assets/` into the ROS package share directory.
If `assets_root` is left empty, the node loads:

```text
<package_share>/assets/testing
```

When running directly from source without an installed/sourced ROS workspace,
pass `assets_root` explicitly:

```bash
ros2 run object_rendering scene_renderer --ros-args \
  -p assets_root:=/absolute/path/to/object_rendering/assets/testing
```

## AprilTag

The default tag family is:

```text
tag36h11
```

The default tag size is:

```text
0.12 meters
```

The printed physical tag size must match `tag_size`, or the estimated pose and
overlay alignment will be wrong.

Printable tag images are available from AprilRobotics:

- https://github.com/AprilRobotics/apriltag-imgs
- https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag36h11

## Installation On The ROS 2 Machine

Install ROS dependencies with the target ROS distribution sourced or available:

```bash
sudo apt install \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-ament-index-python \
  python3-numpy \
  python3-opencv
```

Install Python packages that are not declared as standard ROS package
dependencies here:

```bash
python3 -m pip install pybullet pupil-apriltags
```

Build from a ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select object_rendering
source install/setup.bash
```

## Running

Launch your camera driver separately. For a RealSense camera, that is typically
done by the external `realsense2_camera` package:

```bash
ros2 launch realsense2_camera rs_launch.py
```

Check the published topics:

```bash
ros2 topic list
ros2 topic info /camera/camera/color/image_raw
ros2 topic info /camera/camera/color/camera_info
```

Run the renderer:

```bash
ros2 run object_rendering scene_renderer
```

Run with explicit Rocky or test topics:

```bash
ros2 run object_rendering scene_renderer --ros-args \
  -p image_topic:=/camera/camera/color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info \
  -p output_topic:=/object_rendering/overlay_image \
  -p task_name:=grasping_in_clutter \
  -p scene_file:=0.npz \
  -p tag_size:=0.12 \
  -p renderer:=tiny
```

View the output using an external visualization tool:

```bash
rqt_image_view
```

Select:

```text
/object_rendering/overlay_image
```

## Development Checks

This development machine does not have the full ROS 2 runtime installed. The
checks that can run here are syntax and package metadata checks:

```bash
python3 -m py_compile \
  object_rendering/nodes/scene_renderer_node.py \
  object_rendering/render/apriltag.py \
  object_rendering/render/camera_geometry.py \
  object_rendering/render/overlay.py \
  object_rendering/render/scene_io.py \
  object_rendering/render/simulation_renderer.py \
  setup.py

python3 setup.py --name
```

On the ROS 2 machine, also run:

```bash
colcon test --packages-select object_rendering
colcon test-result --verbose
```

## Official Documentation

ROS 2:

- Packages: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- Python publishers/subscribers: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- Python parameters: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
- Launch files: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
- Nodes: https://docs.ros.org/en/rolling/Concepts/Basic/About-Nodes.html
- Topics: https://docs.ros.org/en/rolling/Concepts/Basic/About-Topics.html

Messages:

- `sensor_msgs/Image`: https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/Image.msg
- `sensor_msgs/CameraInfo`: https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/CameraInfo.msg

Camera and image tools:

- Intel RealSense ROS wrapper: https://github.com/IntelRealSense/realsense-ros
- `cv_bridge`: https://index.ros.org/p/cv_bridge/
- OpenCV color conversions: https://docs.opencv.org/4.x/d8/d01/group__imgproc__color__conversions.html

AprilTag and rendering:

- `pupil-apriltags`: https://pypi.org/project/pupil-apriltags/
- AprilRobotics tag images: https://github.com/AprilRobotics/apriltag-imgs
- PyBullet: https://pybullet.org/wordpress/
