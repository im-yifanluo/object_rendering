# object_rendering

`object_rendering` is a ROS 2 Python package for rendering simulated object
placement guides over a live RealSense RGB camera stream.

The current pipeline keeps camera input and display inside the same ROS node to
avoid passing high-rate images through ROS topics during this phase:

```text
RealSense color frame
  -> OpenCV BGR image
  -> AprilTag detection
  -> camera projection/view matrix computation
  -> PyBullet scene rendering
  -> transparent render creation from PyBullet segmentation
  -> RGBA-over-camera alpha blend
  -> OpenCV display window
```

The camera and display are still isolated behind small interfaces in
`object_rendering/io/`, so they can be replaced later by ROS topics, a rosbag,
or benchmarking hooks without rewriting the rendering code.

## Relationship To ManipulationNet

This package adapts the rendering-related methodology from the local
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
  io/
    display.py
    realsense_camera.py
  nodes/
    scene_renderer_node.py
  render/
    apriltag.py
    camera_geometry.py
    overlay.py
    scene_io.py
    simulation_renderer.py
launch/
  render_pipeline.py
assets/
  testing/
    grasping_in_clutter/
      models/
      scenes/
    tabletop_manipulation/
      models/
      scenes/
```

`nodes/` contains ROS-facing orchestration: parameters, logging, timer loop, and
lifecycle cleanup.

`io/` contains hardware/display adapters. These modules own direct RealSense
capture and OpenCV display.

`render/` contains reusable computation code. These modules should not depend on
ROS messages.

## Parameters

`scene_renderer` declares these parameters:

```text
camera_width         default: 640
camera_height        default: 480
camera_fps           default: 30
display_enabled      default: true
display_window_name  default: object_rendering
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
headless or inconsistent GPU environments.

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

Install ROS and system dependencies with the target ROS distribution sourced or
available:

```bash
sudo apt install \
  ros-$ROS_DISTRO-rclpy \
  ros-$ROS_DISTRO-ament-index-python \
  python3-numpy \
  python3-opencv
```

Install the direct camera and rendering Python packages on the target machine:

```bash
python3 -m pip install pyrealsense2 pybullet pupil-apriltags
```

`pyrealsense2` may also require Intel RealSense SDK/librealsense setup on the
target device.

Build from a ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select object_rendering
source install/setup.bash
```

## Running

Do not launch `realsense2_camera` at the same time as this node for the current
direct-camera pipeline. This node opens the RealSense camera through
`pyrealsense2`.

Run with the package launch file:

```bash
ros2 launch object_rendering render_pipeline.py
```

The launch file starts the `scene_renderer` executable with the default
parameters documented above. It is installed by `setup.py` along with the
runtime assets.

You can also run the node directly:

```bash
ros2 run object_rendering scene_renderer
```

Run with explicit camera and scene settings:

```bash
ros2 run object_rendering scene_renderer --ros-args \
  -p camera_width:=640 \
  -p camera_height:=480 \
  -p camera_fps:=30 \
  -p display_enabled:=true \
  -p task_name:=grasping_in_clutter \
  -p scene_file:=0.npz \
  -p tag_size:=0.12 \
  -p renderer:=tiny
```

Press `q` or `Esc` in the OpenCV display window to stop.

## Development Checks

This development machine does not have the full ROS 2, RealSense, and rendering
runtime installed. The checks that can run here are syntax and package metadata
checks:

```bash
python3 -m py_compile \
  object_rendering/nodes/scene_renderer_node.py \
  object_rendering/io/realsense_camera.py \
  object_rendering/io/display.py \
  object_rendering/render/apriltag.py \
  object_rendering/render/camera_geometry.py \
  object_rendering/render/overlay.py \
  object_rendering/render/scene_io.py \
  object_rendering/render/simulation_renderer.py \
  setup.py \
  launch/render_pipeline.py

python3 setup.py --name
```

On the ROS 2 target machine, build and run the launch file with the RealSense
camera connected.

## Official Documentation

ROS 2:

- Packages: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- Python parameters: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
- Launch files: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
- Nodes: https://docs.ros.org/en/rolling/Concepts/Basic/About-Nodes.html

RealSense and display:

- RealSense Python examples: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples
- RealSense OpenCV viewer example: https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_viewer_example.py
- OpenCV HighGUI display: https://docs.opencv.org/4.x/d7/dfc/group__highgui.html
- OpenCV color conversions: https://docs.opencv.org/4.x/d8/d01/group__imgproc__color__conversions.html

AprilTag and rendering:

- `pupil-apriltags`: https://pypi.org/project/pupil-apriltags/
- AprilRobotics tag images: https://github.com/AprilRobotics/apriltag-imgs
- PyBullet: https://pybullet.org/wordpress/
