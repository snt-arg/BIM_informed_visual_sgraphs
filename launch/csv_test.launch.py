# This file is part of ivS-Graphs.
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
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_isgraphs(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("vs_graphs")
    walls_data_file = os.path.join(pkg_dir, "config", "Uni_building 1.csv")
    csv_publishing_cmd = Node(
        package="vs_graphs",
        executable="csv_publishing_node",
        parameters=[
            {"wall_file_path": walls_data_file, "visualize_bim_markers": True},
            # {"room_file_path": rooms_data_file},
            # {"door_file_path": doors_data_file},
            # {"mesh_file_path": mesh_file}
        ],
        output="screen",
    )

    return [
        csv_publishing_cmd,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
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
            OpaqueFunction(function=launch_isgraphs),
        ]
    )
