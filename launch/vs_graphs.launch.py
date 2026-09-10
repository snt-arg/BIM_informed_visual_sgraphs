# This file is part of ivS-Graphs.
#
# Modifications Copyright (C) 2025-2026 SnT, University of Luxembourg
# Asier Bikandi-Noya, Miguel Fernandez-Cortizas, Muhammad Shaheer, Ali
# Tourani, Holger Voos, and Jose Luis Sanchez-Lopez.
#
# Copyright (C) 2023-2025 SnT, University of Luxembourg
# Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
#
# ivS-Graphs is free software: you can redistribute it and/or modify it under the terms
# of the GNU General Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later version.
#
# ivS-Graphs is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with this program.
# If not, see <https://www.gnu.org/licenses/>.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory

from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            # Set environment variable to suppress TF2 warnings
            SetEnvironmentVariable("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "ERROR"),
            # Global arguments
            DeclareLaunchArgument(
                "bimID_1", default_value="1"
            ),  # ID of the first BIM wall used for initial alignment (see your BIM CSV)
            DeclareLaunchArgument(
                "bimID_2", default_value="3"
            ),  # ID of the second BIM wall used for initial alignment (see your BIM CSV)
            DeclareLaunchArgument("runAgraph", default_value="true"),
            DeclareLaunchArgument(
                "XYZcoord", default_value="false"
            ),  # Default vocabulary file
            DeclareLaunchArgument("justInitialAlignment", default_value="false"),
            DeclareLaunchArgument(
                "walls_csv", default_value="your_bim_file.csv"
            ),  # CSV filename (relative to config/) or absolute path
            DeclareLaunchArgument("offline", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("colored_pointcloud", default_value="true"),
            DeclareLaunchArgument("visualize_segmented_scene", default_value="true"),
            # Topics
            DeclareLaunchArgument("camera_frame", default_value="camera"),
            DeclareLaunchArgument("sensor_config", default_value="RealSense_D435i"),
            DeclareLaunchArgument(
                "rgb_image_topic",
                default_value="/camera/realsense/color/image_raw",  # Modify  realsense/camera
            ),
            DeclareLaunchArgument(
                "rgb_camera_info_topic",
                default_value="/camera/realsense/color/camera_info",  # Modify  realsense/camera
            ),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/realsense/aligned_depth_to_color/image_raw",  # Modify  realsense/camera
            ),
            # VS-Graphs Node
            Node(
                name="vs_graphs",
                package="vs_graphs",
                executable="ros_rgbd",
                output="screen",
                arguments=["--ros-args", "--log-level", "tf2_buffer:=error"],
                parameters=[
                    {"use_sim_time": LaunchConfiguration("offline")},
                    {
                        "voc_file": LaunchConfiguration(
                            "voc_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/Vocabulary/ORBvoc.txt.bin",
                            ],
                        )
                    },
                    {
                        "settings_file": LaunchConfiguration(
                            "settings_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/config/RGB-D/",
                                LaunchConfiguration("sensor_config"),
                                ".yaml",
                            ],
                        )
                    },
                    {
                        "sys_params_file": LaunchConfiguration(
                            "sys_params_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/config/system_params.yaml",
                            ],
                        )
                    },
                    {"bimID_1": LaunchConfiguration("bimID_1")},
                    {"bimID_2": LaunchConfiguration("bimID_2")},
                    {"runAgraph": LaunchConfiguration("runAgraph")},
                    {"XYZcoord": LaunchConfiguration("XYZcoord")},
                    {
                        "justInitialAlignment": LaunchConfiguration(
                            "justInitialAlignment"
                        )
                    },
                    {"roll": 0.0},
                    {"yaw": 1.5697},
                    {"pitch": -1.5697},
                    {"frame_map": "map"},
                    {"frame_world": "world"},
                    {"frame_camera": "camera"},
                    {"frame_bim": "bim_vis"},
                    {"enable_pangolin": False},
                    {"static_transform": True},
                    {"colored_pointcloud": False},
                    {"publish_pointclouds": True},
                ],
                remappings=[
                    ("/camera/rgb/image_raw", LaunchConfiguration("rgb_image_topic")),
                    (
                        "/camera/depth_registered/image_raw",
                        LaunchConfiguration("depth_image_topic"),
                    ),
                ],
            ),
            # Static Transforms
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="bc_to_se",
                arguments=["0", "-3", "0", "0", "0", "0", "build_comp", "struc_elem"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_bc",
                arguments=["0", "-5", "0", "0", "0", "0", "world", "build_comp"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_BIM",
                # arguments=["2", "2.2", "5", "3.5", "0", "0", "map", "bim_vis"],
                arguments=["0", "0", "0", "0", "0", "0", "world", "bim_vis"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_to_camera_optical",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "camera",
                    "camera_color_optical_frame",
                ],
            ),
            # RViz
            Node(
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "-d",
                    [
                        get_package_share_directory("vs_graphs"),
                        "/config/Visualization/vsgraphs_rgbd.rviz",
                    ],
                ],
                output="screen",
            ),
            # Nodelete
            ComposableNodeContainer(
                name="depth_image_proc_container",
                package="rclcpp_components",
                namespace="",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzrgbNode",
                        name="point_cloud_xyzrgb_node",
                        remappings=[
                            (
                                "rgb/camera_info",
                                LaunchConfiguration("rgb_camera_info_topic"),
                            ),
                            (
                                "rgb/image_rect_color",
                                LaunchConfiguration("rgb_image_topic"),
                            ),
                            (
                                "depth_registered/image_rect",
                                LaunchConfiguration("depth_image_topic"),
                            ),
                            ("points", "/camera/depth/points"),
                        ],
                    ),
                ],
            ),
            # Semantic Scene Segmenter Node
            Node(
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_yoso.py",
                output="screen",
                parameters=[
                    {"visualize": LaunchConfiguration("visualize_segmented_scene")}
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_yoso.yaml",
                    ],
                ],
            ),
            OpaqueFunction(function=launch_isgraphs),
        ]
    )


def launch_isgraphs(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("vs_graphs")
    walls_csv_arg = LaunchConfiguration("walls_csv").perform(context)
    if os.path.isabs(walls_csv_arg):
        walls_data_file = walls_csv_arg
    else:
        walls_data_file = os.path.join(pkg_dir, "config", walls_csv_arg)

    csv_publishing_cmd = Node(
        package="vs_graphs",
        executable="csv_publishing_node",
        parameters=[
            {"wall_file_path": walls_data_file}
        ],
        output="screen",
    )

    return [
        csv_publishing_cmd,
    ]
