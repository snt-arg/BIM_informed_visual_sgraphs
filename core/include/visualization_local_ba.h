/**
 * This file is part of ivS-Graphs.
 *
 * Modifications Copyright (C) 2025-2026 SnT, University of Luxembourg
 * Asier Bikandi-Noya, Miguel Fernandez-Cortizas, Muhammad Shaheer, Ali
 * Tourani, Holger Voos, and Jose Luis Sanchez-Lopez.
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

#ifndef VISUALIZATION_LOCAL_BA_H
#define VISUALIZATION_LOCAL_BA_H

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ORB_SLAM3 {
class Plane;
}
// Forward declarations to avoid heavy includes
namespace g2o {
class SparseOptimizer;
}

namespace vs_graphs_visualization {

struct PlaneVisualizationData {
  Eigen::Vector3d centre;
  Eigen::Quaterniond q;
  Eigen::Vector3d scale;
};

// Publisher initialization function
void initializeLocalBAPublisher(std::shared_ptr<rclcpp::Node> node);

// Publishing function
void publishLocalBAVisualization(
    const visualization_msgs::msg::MarkerArray &markers);

// Basic marker creation functions
visualization_msgs::msg::Marker
createBasicMarker(const std::string &frame_id, const rclcpp::Time &stamp,
                  int id, const std::vector<Eigen::Vector3d> &points, int type,
                  double scale = 0.5,
                  const Eigen::Vector4d &color = {0, 0, 0, 1});

visualization_msgs::msg::Marker
createSphereListMarker(const std::string &frame_id, const rclcpp::Time &stamp,
                       int id, const std::vector<Eigen::Vector3d> &points,
                       double scale = 0.1,
                       const Eigen::Vector4d &color = {0, 0, 1, 1});

visualization_msgs::msg::Marker
createLineListMarker(const std::string &frame_id, const rclcpp::Time &stamp,
                     int id, const std::vector<Eigen::Vector3d> &lines,
                     double scale = 0.02,
                     const Eigen::Vector4d &color = {1, 0, 0, 0.8});

visualization_msgs::msg::MarkerArray
createAxisMarker(const std::string &frame_id, const rclcpp::Time &stamp, int id,
                 const Eigen::Isometry3d &transform, double scale = 0.2);

// Plane visualization
PlaneVisualizationData convertPlaneToVisualization(
    const Eigen::Vector4d &plane_coeffs,
    const Eigen::Vector3d &reference_point = Eigen::Vector3d::Zero());

visualization_msgs::msg::Marker
createPlaneMarker(const std::string &frame_id, const rclcpp::Time &stamp,
                  int id, const PlaneVisualizationData &data,
                  const Eigen::Vector4d &color = {0, 0.5, 1, 0.6});

// Individual component visualization functions
visualization_msgs::msg::MarkerArray
visualizeKeyFrames(g2o::SparseOptimizer *optimizer, const std::string &frame_id,
                   int &marker_id);

visualization_msgs::msg::MarkerArray
visualizeMapPoints(g2o::SparseOptimizer *optimizer, const std::string &frame_id,
                   int &marker_id);

visualization_msgs::msg::MarkerArray
visualizePlanes(g2o::SparseOptimizer *optimizer, const std::string &frame_id,
                int &marker_id);

visualization_msgs::msg::MarkerArray
visualizeMarkers(g2o::SparseOptimizer *optimizer, const std::string &frame_id,
                 int &marker_id);

visualization_msgs::msg::MarkerArray
visualizeBIMWalls(g2o::SparseOptimizer *optimizer,
                  const std::vector<ORB_SLAM3::Plane *> &bimWalls,
                  const std::string &frame_id, int &marker_id);

visualization_msgs::msg::MarkerArray
visualizeFixedStatus(g2o::SparseOptimizer *optimizer,
                     const std::string &frame_id, int &marker_id);

visualization_msgs::msg::MarkerArray visualizeLocalBAGraph(
    g2o::SparseOptimizer* optimizer,
    const std::vector<ORB_SLAM3::Plane*>& bimWalls,
    const std::vector<ORB_SLAM3::Plane*>& detectedPlanes,
    const std::vector<std::pair<ORB_SLAM3::Plane*, ORB_SLAM3::Plane*>>& g_associations,
    const std::string& frame_id = "world",
    int initial_id = 1000);

visualization_msgs::msg::MarkerArray
visualizeEdges(g2o::SparseOptimizer *optimizer,
               const std::vector<ORB_SLAM3::Plane *> &bimWalls,
               const std::vector<ORB_SLAM3::Plane *> &detectedPlanes,
               const std::string &frame_id, int &marker_id,
               bool useCentroidsForEdges);
visualization_msgs::msg::MarkerArray
visualizeDetectedPlanes(g2o::SparseOptimizer *optimizer,
                        const std::vector<ORB_SLAM3::Plane *> &detectedPlanes,
                        const std::string &frame_id, int &marker_id);
visualization_msgs::msg::MarkerArray visualizeDetectedPlanesCentroid(
    g2o::SparseOptimizer *optimizer,
    const std::vector<ORB_SLAM3::Plane *> &detectedPlanes,
    const std::string &frame_id, int &marker_id);

} // namespace vs_graphs_visualization

#endif // VISUALIZATION_LOCAL_BA_H