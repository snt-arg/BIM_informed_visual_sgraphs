/**
 * This file is part of ivS-Graphs.
 *
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * ivS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ivS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <fstream>
#include <iostream>
#include <rclcpp/time_source.hpp>
#include <vector>

#include "std_msgs/msg/int32.hpp"
#include "vs_graphs/msg/bim_wall_data.hpp"
#include "vs_graphs/msg/bim_wall_data_array.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

namespace is_graphs {

static Eigen::Vector4d transform_plane(const Eigen::Vector4d &plane_coeffs,
                                       const Eigen::Isometry3d &transform) {

  // Transform a plane defined by a normal vector and distance
  // from the origin using an isometry transformation.
  // From H. Bavle paper "Situational Graphs for Robot Navigation in Structured
  // Indoor Environments"

  Eigen::Vector4d transformed_plane;
  Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
  transform_matrix.block<3, 3>(0, 0) = transform.rotation().matrix();
  Eigen::Vector3d translation = -transform.translation();
  transform_matrix.matrix().row(3) << translation.x(), translation.y(),
      translation.z(), 1.0;

  RCLCPP_DEBUG(
      rclcpp::get_logger("csv_publishing_node"),
      "Transforming plane with coefficients: %s",
      (std::ostringstream() << plane_coeffs.transpose()).str().c_str());
  transformed_plane = transform_matrix * plane_coeffs;
  RCLCPP_DEBUG(
      rclcpp::get_logger("csv_publishing_node"),
      "Transformed plane coefficients: %s",
      (std::ostringstream() << transformed_plane.transpose()).str().c_str());
  return transformed_plane;
}

struct BIMWall {
  int id;
  Eigen::Vector3d normal;
  double distance;
  double length;
  double width;
  double height = 2.0; // Height is not used in the current implementation
  Eigen::Vector3d start_coords = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_coords = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_coords = Eigen::Vector3d::Zero();

  BIMWall() = delete;

  void compute_end_coords() {
    // Compute end_coords based on start_coords, normal, and length
    Eigen::Vector3d perpendicular_direction =
        normal.cross(Eigen::Vector3d::UnitZ()).normalized();
    if (normal.x() > 0) {
      perpendicular_direction =
          -perpendicular_direction; // Ensure the direction is correct based on
                                    // the normal vector
    } else if (normal.x() == 0) {
      // If the normal vector is aligned with the Z-axis, use a different
      // perpendicular direction
      perpendicular_direction =
          Eigen::Vector3d::UnitX(); // Use X-axis as the perpendicular direction
    }
    end_coords = start_coords + perpendicular_direction * length;
    center_coords = (start_coords + end_coords) / 2.0;
  }

  BIMWall(int id, const Eigen::Vector3d &start_coords,
          const Eigen::Vector3d &normal, double distance, double length,
          double width)
      : id(id), normal(normal), distance(distance), length(length),
        width(width), start_coords(start_coords) {
    // compute end_coords and center_coords
    // End coordinates are computed as start_coords  moving along the
    // perpendicular direction of the wall by the length of the wall (towards
    // the growth direction of the components)
    RCLCPP_INFO(rclcpp::get_logger("BIMWall"), "Creating BIMWall with id: %d",
                id);
    compute_end_coords();
  }

  vs_graphs::msg::BIMWallData to_msg() const {
    // Convert BIMWall to a vs_graphs::msg::BIMWallData message
    vs_graphs::msg::BIMWallData msg;
    msg.header.frame_id = "bim_origin";
    msg.header.stamp = rclcpp::Clock().now();
    msg.id = id;
    msg.normal.x = normal.x();
    msg.normal.y = normal.y();
    msg.normal.z = normal.z();
    msg.distance = distance;

    msg.length = length;
    msg.width = width;
    msg.height = height;

    msg.centroid.x = center_coords.x();
    msg.centroid.y = center_coords.y();
    msg.centroid.z = center_coords.z();

    msg.start.x = start_coords.x();
    msg.start.y = start_coords.y();
    msg.start.z = start_coords.z();

    return msg;
  }

  void transform(const Eigen::Isometry3d &transform) {
    // From first S-Graphs Paper
    Eigen::Vector4d plane_coeffs(normal.x(), normal.y(), normal.z(), distance);
    Eigen::Vector4d transformed_coeffs =
        transform_plane(plane_coeffs, transform);
    normal = Eigen::Vector3d(transformed_coeffs.x(), transformed_coeffs.y(),
                             transformed_coeffs.z());
    distance = transformed_coeffs.w();
    start_coords = transform * start_coords;
    end_coords = transform * end_coords;
    center_coords = transform * center_coords;
  }

  friend std::ostream &operator<<(std::ostream &os, const BIMWall &wall) {
    os << "BIMWall(id: " << wall.id << ", start_coords: ["
       << wall.start_coords.transpose() << "]"
       << ", end_coords: [" << wall.end_coords.transpose() << "]"
       << ", normal: [" << wall.normal.transpose() << "]"
       << ", distance: " << wall.distance << ", length: " << wall.length
       << ", width: " << wall.width << ")";
    return os;
  }

  visualization_msgs::msg::Marker center_marker() const {
    // Convert BIMWall to a visualization marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "center_wall";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = center_coords.x();
    marker.pose.position.y = center_coords.y();
    marker.pose.position.z = center_coords.z();

    marker.scale.x = 0.1; // Width of the wall
    marker.scale.y = 0.1; // Length of the wall
    marker.scale.z = 0.1; // Height of the wall

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f; // Fully opaque
    return marker;
  }

  visualization_msgs::msg::Marker origin_marker() const {
    // Convert BIMWall to a visualization marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "origin";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = start_coords.x();
    marker.pose.position.y = start_coords.y();
    marker.pose.position.z = start_coords.z();

    marker.scale.x = 0.3; // Width of the wall
    marker.scale.y = 0.3; // Length of the wall
    marker.scale.z = 0.3; // Height of the wall

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6f; // Fully opaque
    return marker;
  }

  visualization_msgs::msg::Marker end_marker() const {
    // Convert BIMWall to a visualization marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "end";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = end_coords.x();
    marker.pose.position.y = end_coords.y();
    marker.pose.position.z = end_coords.z();

    marker.scale.x = 0.3; // Width of the wall
    marker.scale.y = 0.3; // Length of the wall
    marker.scale.z = 0.3; // Height of the wall

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f; // Fully opaque
    return marker;
  }

  visualization_msgs::msg::Marker normal_marker() const {
    // Convert BIMWall to a visualization marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "normal";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose.position.x = center_coords.x();
    marker.pose.position.y = center_coords.y();
    marker.pose.position.z = center_coords.z();
    // Set the orientation of the arrow to point in the direction of the normal
    // vector

    Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d::UnitX(), normal.normalized());
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    marker.scale.x = 0.5; // Width of the ARROW
    marker.scale.y = 0.1; // Length of the ARROW
    marker.scale.z = 0.1; // Height of the ARROW

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.6f; // Fully opaque
    return marker;
  }

  visualization_msgs::msg::Marker marker() {
    // Convert BIMWall to a visualization marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "wall";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = center_coords.x();
    marker.pose.position.y = center_coords.y();
    marker.pose.position.z =
        center_coords.z() +
        height / 2.0; // Adjust height to be at the center of the wall
    // Set the orientation of the arrow to point in the direction of the normal
    // vector

    Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(
        Eigen::Vector3d::UnitX(), normal.normalized());
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    marker.scale.x = width;  // Width of the wall
    marker.scale.y = length; // Length of the wall
    marker.scale.z = height; // Height of the wall

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6f; // Fully opaque
    return marker;
  }
};

class CSVPublishingNode : public rclcpp::Node {
public:
  CSVPublishingNode() : Node("csv_publishing_node") {

    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    initialize_params();
    initialize_publishers();

    // TODO: Remove this hardcoded path from here

    auto csv_content = read_csv(wall_file_path);
    // Read wall data from CSV file
    walls_data = obtain_walls_data_from_csv(csv_content);

    number_of_walls.data = walls_data.size();
    // number_of_rooms.data = rooms_tag_id.size();
    // number_of_doors.data = doors_tag_id.size();

    // 1. TRANSLATION: Move BIM to desired position
    Eigen::Isometry3d translation_transform = Eigen::Isometry3d::Identity();
    translation_transform.translation() =
        Eigen::Vector3d(0.0, 0.0, 0.0); // Move BIM by ()

    // 2. ROTATION: Rotate BIM to desired orientation
    Eigen::Isometry3d rotation_transform = Eigen::Isometry3d::Identity();
    rotation_transform.rotate(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

    Eigen::Isometry3d transform_change_axis = Eigen::Isometry3d::Identity();
    transform_change_axis.matrix() << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0,
        0, 0, 1; // Identity transformation

    for (auto &wall : walls_data) {
      wall.transform(rotation_transform);
      wall.transform(transform_change_axis);
    }

    while (rclcpp::ok()) {

      number_of_walls_pub->publish(number_of_walls);

      if (visualize_bim_markers) {
        visualization_msgs::msg::MarkerArray markers;
        for (auto &wall : walls_data) {
          markers.markers.push_back(wall.center_marker());
          markers.markers.push_back(wall.origin_marker());
          markers.markers.push_back(wall.end_marker());
          markers.markers.push_back(wall.normal_marker());
          markers.markers.push_back(wall.marker());
        }
        markers_pub->publish(markers);
      }

      // Publish the wall data as a BIMWallDataArray message
      vs_graphs::msg::BIMWallDataArray wall_data_array;
      wall_data_array.walls.reserve(walls_data.size());
      for (const auto &wall : walls_data) {
        wall_data_array.walls.push_back(wall.to_msg());
      }
      bim_wall_data_pub->publish(wall_data_array);

      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
  }

private:
  void initialize_params() {
    this->declare_parameter<bool>("visualize_bim_markers", false);
    visualize_bim_markers = this->get_parameter("visualize_bim_markers")
                                .get_parameter_value()
                                .get<bool>();
    RCLCPP_INFO(this->get_logger(), "Visualize BIM markers: %s",
                visualize_bim_markers ? "true" : "false");

    // this->declare_parameter("wall_file_path");
    this->declare_parameter<std::string>("wall_file_path", "");
    // this->declare_parameter("room_file_path");
    // this->declare_parameter("door_file_path");
    // this->declare_parameter("mesh_file_path");
    wall_file_path = this->get_parameter("wall_file_path")
                         .get_parameter_value()
                         .get<std::string>();

    //////////MODIFY/////////////
    // std::string pkg_dir =
    // ament_index_cpp::get_package_share_directory("vs_graphs"); wall_file_path
    // = pkg_dir + "/config/topfloor_walls_org.csv"; room_file_path =
    //     this->get_parameter("room_file_path").get_parameter_value().get<std::string>();
    // door_file_path =
    //     this->get_parameter("door_file_path").get_parameter_value().get<std::string>();
    // mesh_file_path =
    //     this->get_parameter("mesh_file_path").get_parameter_value().get<std::string>();

    // log the path of the wall file
    RCLCPP_INFO(this->get_logger(), "Wall file path: %s",
                wall_file_path.c_str());
  }

  void initialize_publishers() {

    number_of_walls_pub = this->create_publisher<std_msgs::msg::Int32>(
        "agraph/number_of_walls", 1);
    // number_of_rooms_pub =
    //     this->create_publisher<std_msgs::msg::Int32>("agraph/number_of_rooms",
    //     1);
    // number_of_doors_pub =
    //     this->create_publisher<std_msgs::msg::Int32>("agraph/number_of_doors",
    //     1);
    // markers_pub =
    //     this->create_publisher<visualization_msgs::msg::MarkerArray>("csv_data",
    //     16);
    bim_wall_data_pub =
        this->create_publisher<vs_graphs::msg::BIMWallDataArray>(
            "bim/wall_data", 10);

    if (visualize_bim_markers) {
      // Create a publisher for visualization markers
      markers_pub =
          this->create_publisher<visualization_msgs::msg::MarkerArray>(
              "bim/markers", 16);
    }
  }

private:
  using CSVContent = std::unordered_map<std::string, std::vector<double>>;

  /** @brief
   * Reads a CSV file and returns its content as a map where the keys are column
   * names and the values are vectors of doubles representing the column data.
   * @param file_path The path to the CSV file.
   * @return A map containing the CSV content.
   */
  CSVContent read_csv(std::string file_path) {
    // Reads a CSV file into a map where the keys are column names
    // and the values are vectors of doubles representing the column data.
    CSVContent content;

    // Create an input filestream
    std::ifstream myFile(file_path);

    // Make sure the file is open
    if (!myFile.is_open())
      throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    double val;
    std::vector<std::string> colnames;

    // Read the column names
    if (myFile.good()) {
      // Extract the first line in the file
      std::getline(myFile, line);
      std::stringstream ss(line);

      // Extract each column name
      while (std::getline(ss, colname, ',')) {
        // Initialize and add <colname, int vector> pairs to result
        content.emplace(colname, std::vector<double>());
        // Store the column name for later use
        colnames.push_back(colname);
      }
    }

    while (std::getline(myFile, line)) {
      std::stringstream ss(line);
      int colIdx = 0;
      // Extract each value
      while (ss >> val) {
        // Add the current integer to the 'colIdx' column's values vector

        if (colIdx >= colnames.size()) {
          throw std::runtime_error("Column index out of bounds at " +
                                   std::to_string(colIdx) +
                                   " for line: " + line);
        }
        if (content.find(colnames[colIdx]) == content.end()) {
          throw std::runtime_error("Column name not found: " +
                                   colnames[colIdx]);
        }
        content.at(colnames[colIdx]).push_back(val);
        // If the next token is a comma, ignore it and move on
        if (ss.peek() == ',')
          ss.ignore();
        // Increment the column index
        colIdx++;
      }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("csv_publishing_node"),
                 "CSV file read successfully. Number of columns: %zu",
                 content.size());
    // Close file
    myFile.close();

    return content;
  }

  /** @brief
   * Reads wall data from a CSV file and returns a vector of BIMWall objects.
   * @param csv_content The content of the CSV file as a map of column names to
   * values.
   * @return A vector of BIMWall objects containing the wall data.
   */
  std::vector<BIMWall>
  obtain_walls_data_from_csv(const CSVContent &csv_content) {
    std::vector<BIMWall> walls_data;
    RCLCPP_DEBUG(rclcpp::get_logger("csv_publishing_node"),
                 "Obtaining walls data from CSV. Columns: %zu",
                 csv_content.size());

    size_t num_walls = csv_content.at("TAG").size();
    for (size_t i = 0; i < num_walls; ++i) {
      int id = csv_content.at("TAG")[i];
      Eigen::Vector3d start_coords(csv_content.at("X-min")[i],
                                   csv_content.at("Y-min")[i],
                                   csv_content.at("Z-min")[i]);
      Eigen::Vector3d normal(csv_content.at("X-nor")[i],
                             csv_content.at("Y-nor")[i], 0.0);

      double length = csv_content.at("Length")[i];
      // double width = csv_content.at("thickness")[i];
      double width = 0.1;

      // Create a BIMWall object and populate it with data from the CSV
      double distance = -normal.dot(start_coords);
      BIMWall wall(id, start_coords, normal, distance, length, width);
      RCLCPP_DEBUG(rclcpp::get_logger("csv_publishing_node"),
                   "Wall created: id=%d", id);
      walls_data.push_back(wall);
    }
    return walls_data;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr number_of_walls_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub;
  rclcpp::Publisher<vs_graphs::msg::BIMWallDataArray>::SharedPtr
      bim_wall_data_pub;

  std_msgs::msg::Int32 number_of_walls;
  std::vector<BIMWall> walls_data;
  std::string wall_file_path;
  bool visualize_bim_markers = false;
};

} // namespace is_graphs
// PLUGINLIB_EXPORT_CLASS(is_graphs::CSVPublishingNodelet, nodelet::Nodelet)
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<is_graphs::CSVPublishingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
