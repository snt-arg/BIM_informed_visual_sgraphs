# 🚀 Install ivS-Graphs

## 📝 I. Package Installation

### Install OpenCV

ivS-Graphs requires **OpenCV v4.2+** for computer vision tasks, which can be installed via the [installation instructions](https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html) page. For Ubuntu 24.04, install it using below commands.

```bash
sudo apt update
sudo apt install libopencv-dev -y

# Check the version
dpkg -l libopencv-dev
```

### Install Eigen3

ivS-Graphs relies on **Eigen3**, a C++ template library for linear algebra (matrices, vectors, solvers, etc.). You can install it via APT:

```bash
sudo apt install libeigen3-dev
```

> 🛎️ Tip: `libeigen3-dev` is a header-only library and no linking is required, as `CMake` will automatically find it.

### Install Pangolin

[Pangolin](https://github.com/stevenlovegrove/Pangolin) is a lightweight and portable library used for visualizing 3D data, camera views, and prototyping video-based programs. ivS-Graphs has been tested with **Pangolin v0.8**.

```bash
# Clone
git clone --branch v0.9.1 --depth 1 https://github.com/stevenlovegrove/Pangolin.git

# Install using CMake
cd Pangolin
mkdir build && cd build
cmake ..
make -j
sudo make install
```

### Install Required ROS Libraries

Ensure that all necessary `ROS` dependencies are installed:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-rviz-visual-tools ros-jazzy-depth-image-proc ros-jazzy-backward-ros
sudo apt install ros-jazzy-pcl-ros
```

> 🛎️ Note: The current version of ivS-Graphs supports **ROS2 Jazzy** and is primarily tested on Ubuntu 24.04.

### Install RealSense Library (Optional)

To use an Intel RealSense camera for **live mode** or **data collection**, you will need to install the necessary drivers and libraries. Please follow the detailed setup guide available in [RealSense Setup Instructions](/doc/RealSense/README.md) page. This includes steps for:

- Installing `librealsense`
- Verifying camera connection
- Enabling live streaming or data recording

### Install `aruco_ros` (Optional) <a id="aruco"></a>

ivS-Graphs can operate independently of fiducial markers, but it also supports enhanced semantic labeling using detected markers (e.g., assigning room names) when a valid marker database is provided.

If you would like to use marker-driven data augmentation to the generated scene graph, install the [`aruco_ros`](https://github.com/pal-robotics/aruco_ros) library:

```bash
git clone -b humble-devel https://github.com/pal-robotics/aruco_ros.git
```

> 🛎️ Note: Use the sample modified version of the marker detector launch file available [here](doc/template_aruco_ros.launch). This setup ensures proper marker detection and integration with ivS-Graphs.

### Install the In-house Scene Segmenter <a id="segmenter"></a>

ivS-Graphs relies on a ROS-based **Panoptic Scene Segmentation** module, available at [scene_segment_ros](https://github.com/snt-arg/scene_segment_ros), to semantically identify **building components** (i.e., walls and ground surfaces). This module supports multiple segmentation backends, including:

- [**YOSO**](https://github.com/hujiecpp/YOSO) _(recommended for best performance)_
- [**PanopticFCN**](https://github.com/dvlab-research/PanopticFCN)

Clone the repository and set it ready for integration:

```bash
git clone -b ros2-jazzy https://github.com/snt-arg/scene_segment_ros.git

# Install Python packages
pip install -r src/requirements.txt

# Follow the rest installation guide in the repository
```

### 🦊 Install Voxblox Skeleton <a id="voxblox"></a>

To detect **structural elements** (such as rooms and corridors), ivS-Graphs requires `loco planning` integrated with `voxblox`, available [here](https://github.com/snt-arg/mav_voxblox_planning/tree/master). This module provides free-space information crucial for **cluster-based structural element detection**. Clone and install the module using below commands:

```bash
git clone https://github.com/snt-arg/mav_voxblox_planning.git

# Follow the build instructions in the repository's README
```

## ⚙️ II. Build the Project

After installing all the required dependencies and modules listed above, clone the repository and build it using `colcon`:

```bash
# Create a ROS2 workspace
mkdir -p ~/ivsgraphs_ws/src
cd ~/ivsgraphs_ws/src

# Clone the ivS-Graphs repository into the src folder
git clone https://github.com/snt-arg/visual_sgraphs.git

# Build the workspace
cd ~/ivsgraphs_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
rosdep update
```
