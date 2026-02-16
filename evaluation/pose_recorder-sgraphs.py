#!/usr/bin/env python3

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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs


class TextFileGeneratorNode(Node):
    def __init__(self):
        super().__init__("text_file_generator")
        self.slam_pose_topic_name = "/s_graphs/odom_pose_corrected"
        self.slam_pose_file = open("stamped_traj_estimate.txt", "w+")
        self.slam_pose_file.write("#timestamp tx ty tz qx qy qz qw\n")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.slam_sub = self.create_subscription(
            PoseStamped, self.slam_pose_topic_name, self.slam_pose_callback, 10
        )

        self.get_logger().info(f"Recording poses from {self.slam_pose_topic_name}")
        self.message_count = 0

    def slam_pose_callback(self, slam_pose_msg):
        # FIXED: Use message timestamp instead of current time
        # Original (incorrect): time = self.get_clock().now().to_msg()
        # Fixed: Use the timestamp from the message header
        time_stamp = slam_pose_msg.header.stamp
        time_in_seconds = time_stamp.sec + time_stamp.nanosec * 1e-9

        pose = slam_pose_msg.pose
        odom_x = pose.position.x
        odom_y = pose.position.y
        odom_z = pose.position.z

        # FIXED: Use all quaternion components, not just z and w
        odom_qx = pose.orientation.x
        odom_qy = pose.orientation.y
        odom_qz = pose.orientation.z
        odom_qw = pose.orientation.w

        # Format the values and write to the file in TUM format
        formatted_line = f"{time_in_seconds:.9f} {odom_x:.9f} {odom_y:.9f} {odom_z:.9f} {odom_qx:.9f} {odom_qy:.9f} {odom_qz:.9f} {odom_qw:.9f}\n"
        self.slam_pose_file.write(formatted_line)
        self.slam_pose_file.flush()  # Ensure data is written immediately

        # Log progress every 100 messages
        self.message_count += 1
        if self.message_count % 100 == 0:
            self.get_logger().info(
                f"Recorded {self.message_count} poses. Latest timestamp: {time_in_seconds:.3f}"
            )

    def __del__(self):
        # Ensure file is closed when node is destroyed
        if hasattr(self, "slam_pose_file"):
            self.slam_pose_file.close()
            self.get_logger().info("Trajectory file closed")


def main(args=None):
    rclpy.init(args=args)
    node = TextFileGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Recording stopped by user")
    finally:
        # Properly close the file before destroying the node
        if hasattr(node, "slam_pose_file"):
            node.slam_pose_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
