# Informed visual S-Graphs (ivS-Graphs)

![ivS-Graphs](doc/ivsgraphs.gif "ivS-Graphs")

<!-- Badges -->

[![arXiv](https://img.shields.io/badge/arXiv-2509.13972-b31b1b.svg)](http://arxiv.org/abs/2509.13972)
[![Static Badge](https://img.shields.io/badge/Docker-available-%23B31B1B?style=flat&logo=docker&logoColor=%232496ED&color=%232496ED)](/docker/README.md)
[![Static Badge](https://img.shields.io/badge/YouTube-watch-%23FF0000?style=flat&logo=youtube&logoColor=%23FF0000&color=%23FF0000)](https://www.youtube.com/watch?v=jO3rq93ZRtM)
![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-%2322314E?style=flat&logo=ros&logoColor=%2322314E&color=%2322314E)
[![Static Badge](https://img.shields.io/badge/License-GPLv3-%2387C540?style=flat&logo=gplv3&logoColor=%2387C540&color=%2387C540)](/LICENSE)

**ivS-Graphs** is inspired by [LiDAR iS-Graphs](https://www.arxiv.org/abs/2408.01737)  and extends [vS-graphs](https://github.com/snt-arg/visual_sgraphs) by integrating architecural Building Information Modelling (**BIM**) with an **optimizable 3D scene graphs**, enhancing mapping and localization accuracy through scene understanding which enables construction monitoring. 

## 🧠 ivS-Graphs Architecture

Below diagram shows the detailed architecture of the **ivS-Graphs** framework, The pipeline takes BIM and RGB-D camera data as inputs. The SLAM backbone front-end processes visual data into keyframes, map points, and wall segments. Our contributions are highlighted in green:(1) initial alignment followed by a (2) continuous wall association and (3) the integration of BIM in the back-end of the system. These establish BIM-to-SLAM correspondences (WA ↔ WS ) that are introduced as constraints into the back-end graph. The resulting optimized map (right) aligns the evolving as-built structure with the as-planned BIM

![ivS-Graphs Flowchart](doc/sys.png "ivS-Graphs Flowchart")

## ⚙️ Prerequisites and Installation

For system requirements, dependencies, and setup instructions, refer to the [Installation Guide](/doc/INSTALLATION.md).

## 🔨 Configurations

You can read about the SLAM-related configuration parameters (independent of the `ROS2` wrapper) in [the config folder](/config/README.md). These configurations can be modified in the [system_params.yaml](/config/system_params.yaml) file. For more information on ROS-related configurations and usage, see the [ROS parameter documentation](/doc/ROS.md) page.

## 🏗️ BIM Wall Data

**ivS-Graphs** aligns the SLAM map with a building's walls, provided as a CSV file with one row per wall:

| Column      | Description                                    |
| ----------- | ----------------------------------------------- |
| `TAG`       | Unique wall ID                                  |
| `X-min`     | X coordinate of the wall's start point (m)      |
| `Y-min`     | Y coordinate of the wall's start point (m)      |
| `Z-min`     | Z coordinate of the wall's start point (m)      |
| `X-nor`     | X component of the wall's normal vector         |
| `Y-nor`     | Y component of the wall's normal vector         |
| `Length`    | Wall length (m)                                 |
| `thickness` | Wall thickness (m)                              |

You provide this CSV yourself (e.g., exported from your BIM/floor plan), and pass its path via the `walls_csv` launch argument described below.

## 🚀 Getting Started

Once you have installed the required dependencies, configured the parameters, and prepared your BIM wall CSV, you are ready to run **ivS-Graphs**! Follow the steps below to get started:

1. Source ivS-Graphs and run it by `ros2 launch vs_graphs vs_graphs.launch.py walls_csv:=/path/to/your_bim_file.csv bimID_1:=<wall_id> bimID_2:=<wall_id>`, where `bimID_1`/`bimID_2` are the `TAG`s of two **non-parallel** walls from your CSV, used for initial alignment. This automatically runs the ivS-Graphs core and the semantic segmentation module for **building component** (walls and ground surfaces) recognition.
2. (Optional) If you intend to detect **structural elements** (rooms and corridors) too, run the cluster-based solution using `ros2 launch voxblox_skeleton skeletonize_map_vsgraphs.launch 2>/dev/null`.

   - In this case, you need to source `voxblox` with a `--extend` command, and then launch the framework:

   ```bash
   source /opt/ros/jazzy/setup.bash &&
   source ~/[IVSGRAPHS_WS_PATH]/install/setup.bash &&
   source ~/[VOXBLOX_PATH]/install/setup.bash --extend &&
   ros2 launch vs_graphs vs_graphs.launch.py walls_csv:=/path/to/your_bim_file.csv bimID_1:=<wall_id> bimID_2:=<wall_id>
   ```

3. (Optional) If you have a database of ArUco markers representing room/corridor labels, do not forget to run `aruco_ros` using `ros2 launch aruco_ros marker_publisher.launch`.
4. Now, play a recorded `bag` file by running `ros2 bag play [sample].bag --clock`.

✨ For a complete list of configurable launch arguments, check the [Launch Parameters](/launch/README.md).

✨ For detailed description on how to use a RealSense D400 series camera for live feed and data collection, check [this page](/doc/RealSense/README.md).

> 🛎️ Note: The current version of ivS-Graphs supports **ROS2 Jazzy** and is primarily tested on Ubuntu 24.04.2 LTS.

## 🐋 Docker

For a fully reproducible and environment-independent setup, check the [Docker](/docker) section.

## 📏 Benchmarking

To evaluate ivS-Graphs against other visual SLAM frameworks, read the [evaluation and benchmarking documentation](/evaluation/README.md).

## 📚 Citation

This work has been accepted for publication in **IEEE Robotics and Automation Letters (RA-L)**. If you use ivS-Graphs in your research, please cite:

```bibtex
@misc{bikandinoya2025biminformedvisualslam,
      title={BIM Informed Visual SLAM for Construction Monitoring}, 
      author={Asier Bikandi-Noya and Miguel Fernandez-Cortizas and Muhammad Shaheer and Ali Tourani and Holger Voos and Jose Luis Sanchez-Lopez},
      year={2025},
      eprint={2509.13972},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={http://arxiv.org/abs/2509.13972}, 
}
```

> 🛎️ Note: This citation currently points to the arXiv preprint; it will be updated with the official IEEE RA-L reference once available.

## 📎 Related Repositories

- 🔧 [Visual S-Graphs](https://github.com/snt-arg/visual_sgraphs)
- 🎞️ Scene Segmentor ([ROS2 Jazzy](https://github.com/snt-arg/scene_segment_ros))


## 🔑 License

This project is licensed under the GPL-3.0 license - see the [LICENSE](/LICENSE) for more details.

