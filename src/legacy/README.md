# Legacy ROS1 Wrappers — UNPORTED, DO NOT USE

These files contain **ROS1 API** (`ros::init`, `ros::NodeHandle`, `ROS_WARN`, etc.) and have **not been ported to ROS2**. They are kept here for reference only and are **not built** by the current `CMakeLists.txt`.

| File | Description |
|------|-------------|
| `ros_mono.cc` | Monocular mode wrapper (ROS1) |
| `ros_mono_inertial.cc` | Monocular-Inertial mode wrapper (ROS1) |
| `ros_stereo.cc` | Stereo mode wrapper (ROS1) |
| `ros_stereo_inertial.cc` | Stereo-Inertial mode wrapper (ROS1) |
| `ros_rgbd_inertial.cc` | RGB-D Inertial mode wrapper (partially ported, contains duplicate classes) |

The only active ROS2 wrapper is [`src/ros_rgbd.cc`](../ros_rgbd.cc).

If you need one of these modes in ROS2, these files can serve as a starting point for porting.
