# List of Dependencies

## ivS-Graphs

This document lists all dependencies used by ivS-Graphs, including inherited code from ORB-SLAM3 and additional libraries introduced for scene graph construction.

## Code in **core/src** and **core/include** folders

### Inherited from ORB-SLAM3

- _ORBextractor.cc_ — Modified version of `orb.cpp` from OpenCV. BSD licensed.
- _PnPsolver.h, PnPsolver.cc_ — Modified version of the EPnP solver by Vincent Lepetit. FreeBSD licensed.
- _MLPnPsolver.h, MLPnPsolver.cc_ — Modified version of MLPnP by Steffen Urban ([source](https://github.com/urbste/opengv)). BSD licensed.
- _ORBmatcher::DescriptorDistance_ in _ORBmatcher.cc_ — From [Bit Twiddling Hacks](http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel). Public domain.

## Code in **core/Thirdparty** folder

- **DBoW2** — Modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib). BSD licensed.
- **g2o** — Modified version of [g2o](https://github.com/RainerKuemmerle/g2o). BSD licensed.
- **Sophus** — [Sophus](https://github.com/strasdat/Sophus) Lie group library. [MIT license](https://opensource.org/licenses/MIT).
- **nlohmann/json** — [JSON for Modern C++](https://github.com/nlohmann/json). [MIT license](https://opensource.org/licenses/MIT).
- **pcl_custom** — Custom PCL utility headers.

## System Library Dependencies

| Library | Version | License | Purpose |
|---------|---------|---------|----------|
| [OpenCV](https://opencv.org/) | ≥ 4.2 | BSD | Computer vision, feature extraction |
| [Eigen3](https://eigen.tuxfamily.org/) | ≥ 3.1.0 | MPL2 (≥3.1.1) | Linear algebra |
| [Pangolin](https://github.com/stevenlovegrove/Pangolin) | ≥ 0.8 | MIT | 3D visualization |
| [PCL](https://pointclouds.org/) | — | BSD | Point cloud processing |
| [OpenGL](https://www.opengl.org/) | — | — | Rendering |
| [GLEW](https://glew.sourceforge.net/) | — | BSD/MIT | OpenGL extension loading |
| [lsqcpp](https://github.com/Rookfighter/least-squares-cpp) | — | MIT | Least-squares optimization |

## ROS2 Dependencies

| Package | License | Purpose |
|---------|---------|----------|
| `rclcpp` / `rclpy` | Apache-2.0 | ROS2 client libraries |
| `cv_bridge` | BSD | ROS ↔ OpenCV image conversion |
| `image_transport` | BSD | Image topic transport |
| `tf2`, `tf2_ros`, `tf2_eigen`, `tf2_sensor_msgs` | BSD | Coordinate frame transforms |
| `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs` | Apache-2.0 | Standard message types |
| `message_filters` | BSD | Synchronized message subscription |
| `pcl_ros` | BSD | PCL ↔ ROS integration |
| `rviz_visual_tools` | BSD | RViz visualization utilities |
| `backward_ros` | MIT | Stack trace debugging |
| `depth_image_proc` | BSD | Depth image processing |

## External ROS Packages (separate clones)

| Package | Repository | License | Purpose |
|---------|-----------|---------|----------|
| `segmenter_ros` | [snt-arg/scene_segment_ros](https://github.com/snt-arg/scene_segment_ros) | GPL-3.0 | Panoptic scene segmentation |
| `aruco_ros` _(optional)_ | [pal-robotics/aruco_ros](https://github.com/pal-robotics/aruco_ros) | BSD | Fiducial marker detection |
| `mav_voxblox_planning` _(optional)_ | [snt-arg/mav_voxblox_planning](https://github.com/snt-arg/mav_voxblox_planning) | BSD/MIT | Voxel-based room detection |

## Python Dependencies

See [docker/requirements.txt](docker/requirements.txt) for the full list of Python packages used by the segmentation and evaluation modules.
