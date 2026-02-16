#include "visualization_local_ba.h"
#include "OptimizableTypes.h"
#include "Geometric/Plane.h" 

#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

namespace vs_graphs_visualization {

// Global publisher for local BA visualization
static rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr g_localBAPublisher = nullptr;
static std::shared_ptr<rclcpp::Node> g_nodePtr = nullptr;
// Keep track of previous marker IDs for cleanup**
static std::set<int> g_previous_bim_edge_markers;

// Function to initialize the publisher
void initializeLocalBAPublisher(std::shared_ptr<rclcpp::Node> node) {
    g_nodePtr = node;
    g_localBAPublisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "local_ba_graph", 1);
    // std::cout << "🎯 Local BA visualization publisher initialized!" << std::endl;
}

// Function to publish visualization
void publishLocalBAVisualization(const visualization_msgs::msg::MarkerArray& markers) {
    if (g_localBAPublisher) {
        g_localBAPublisher->publish(markers);
        // std::cout << "📊 Published " << markers.markers.size() 
        //           << " local BA visualization markers" << std::endl;
    } 
    // else {
    //     std::cout << "❌ Local BA publisher not initialized!" << std::endl;
    // }
}

visualization_msgs::msg::MarkerArray visualizeFixedStatus(
    g2o::SparseOptimizer* optimizer,
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray status_array;
    std::vector<Eigen::Vector3d> fixed_kf_positions;
    std::vector<Eigen::Vector3d> optimizable_kf_positions;
    std::vector<Eigen::Vector3d> fixed_plane_centers;
    std::vector<Eigen::Vector3d> optimizable_plane_centers;
    
    // Check KeyFrame vertices
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto se3_vertex = dynamic_cast<g2o::VertexSE3Expmap*>(vertex_pair.second);
        if (se3_vertex) {
            g2o::SE3Quat Tcw = se3_vertex->estimate();
            g2o::SE3Quat Twc = Tcw.inverse();
            Eigen::Vector3d position = Twc.translation();
            
            if (se3_vertex->fixed()) {
                fixed_kf_positions.push_back(position);
            } else {
                optimizable_kf_positions.push_back(position);
            }
        }
        
        // Check Plane vertices
        auto plane_vertex = dynamic_cast<g2o::VertexPlane*>(vertex_pair.second);
        if (plane_vertex) {
            g2o::Plane3D plane = plane_vertex->estimate();
            Eigen::Vector3d center = -plane.coeffs()(3) * plane.normal();
            
            if (plane_vertex->fixed()) {
                fixed_plane_centers.push_back(center);
            } else {
                optimizable_plane_centers.push_back(center);
            }
        }
    }
    
    // Fixed KeyFrames (RED)
    if (!fixed_kf_positions.empty()) {
        auto fixed_kf_marker = createSphereListMarker(
            frame_id, rclcpp::Time(0), marker_id++, fixed_kf_positions, 
            0.08, {1.0, 0.0, 0.0, 1.0}); // RED
        fixed_kf_marker.ns = "local_ba/fixed_keyframes";
        status_array.markers.push_back(fixed_kf_marker);
    }
    
    // Optimizable KeyFrames (GREEN)
    if (!optimizable_kf_positions.empty()) {
        auto opt_kf_marker = createSphereListMarker(
            frame_id, rclcpp::Time(0), marker_id++, optimizable_kf_positions, 
            0.06, {0.0, 1.0, 0.0, 1.0}); // GREEN
        opt_kf_marker.ns = "local_ba/optimizable_keyframes";
        status_array.markers.push_back(opt_kf_marker);
    }
    
    // Fixed Planes (ORANGE - BIM walls)
    if (!fixed_plane_centers.empty()) {
        auto fixed_plane_marker = createSphereListMarker(
            frame_id, rclcpp::Time(0), marker_id++, fixed_plane_centers, 
            0.12, {1.0, 0.5, 0.0, 0.9}); // ORANGE
        fixed_plane_marker.ns = "local_ba/fixed_bim_planes";
        status_array.markers.push_back(fixed_plane_marker);
    }
    
    // Optimizable Planes (CYAN - detected planes)
    if (!optimizable_plane_centers.empty()) {
        auto opt_plane_marker = createSphereListMarker(
            frame_id, rclcpp::Time(0), marker_id++, optimizable_plane_centers, 
            0.10, {0.0, 1.0, 1.0, 0.9}); // CYAN
        opt_plane_marker.ns = "local_ba/optimizable_detected_planes";
        status_array.markers.push_back(opt_plane_marker);
    }
    
    // std::cout << "🔴 Fixed KeyFrames: " << fixed_kf_positions.size() << std::endl;
    // std::cout << "🟢 Optimizable KeyFrames: " << optimizable_kf_positions.size() << std::endl;
    // std::cout << "🟠 Fixed BIM Planes: " << fixed_plane_centers.size() << std::endl;
    // std::cout << "🔵 Optimizable Detected Planes: " << optimizable_plane_centers.size() << std::endl;
    
    return status_array;
}

visualization_msgs::msg::Marker createBasicMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    const std::vector<Eigen::Vector3d>& points,
    int type,
    double scale,
    const Eigen::Vector4d& color) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    
    marker.scale.x = marker.scale.y = marker.scale.z = scale;
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    
    marker.pose.orientation.w = 1.0;
    
    marker.points.resize(points.size());
    marker.colors.resize(points.size());
    
    for (size_t i = 0; i < points.size(); i++) {
        marker.points[i].x = points[i].x();
        marker.points[i].y = points[i].y();
        marker.points[i].z = points[i].z();
        
        marker.colors[i].r = color[0];
        marker.colors[i].g = color[1];
        marker.colors[i].b = color[2];
        marker.colors[i].a = color[3];
    }
    
    return marker;
}

visualization_msgs::msg::Marker createSphereListMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    const std::vector<Eigen::Vector3d>& points,
    double scale,
    const Eigen::Vector4d& color) {
    
    return createBasicMarker(frame_id, stamp, id, points, 
                           visualization_msgs::msg::Marker::SPHERE_LIST, scale, color);
}

visualization_msgs::msg::Marker createLineListMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    const std::vector<Eigen::Vector3d>& lines,
    double scale,
    const Eigen::Vector4d& color) {
    
    return createBasicMarker(frame_id, stamp, id, lines, 
                           visualization_msgs::msg::Marker::LINE_LIST, scale, color);
}

visualization_msgs::msg::MarkerArray createAxisMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    const Eigen::Isometry3d& transform,
    double scale) {
    
    visualization_msgs::msg::MarkerArray axis_array;
    
    // X-axis (red)
    visualization_msgs::msg::Marker x_axis;
    x_axis.header.frame_id = frame_id;
    x_axis.header.stamp = stamp;
    x_axis.id = id;
    x_axis.type = visualization_msgs::msg::Marker::ARROW;
    x_axis.action = visualization_msgs::msg::Marker::ADD;
    
    geometry_msgs::msg::Point start, end;
    start.x = transform.translation().x();
    start.y = transform.translation().y();
    start.z = transform.translation().z();
    
    Eigen::Vector3d x_dir = transform.linear() * Eigen::Vector3d(scale, 0, 0);
    end.x = start.x + x_dir.x();
    end.y = start.y + x_dir.y();
    end.z = start.z + x_dir.z();
    
    x_axis.points = {start, end};
    x_axis.scale.x = scale * 0.02; // shaft diameter
    x_axis.scale.y = scale * 0.05; // head diameter
    x_axis.color.r = 1.0; x_axis.color.g = 0.0; x_axis.color.b = 0.0; x_axis.color.a = 1.0;
    
    // Y-axis (green)
    visualization_msgs::msg::Marker y_axis = x_axis;
    y_axis.id = id + 1;
    Eigen::Vector3d y_dir = transform.linear() * Eigen::Vector3d(0, scale, 0);
    end.x = start.x + y_dir.x();
    end.y = start.y + y_dir.y();
    end.z = start.z + y_dir.z();
    y_axis.points = {start, end};
    y_axis.color.r = 0.0; y_axis.color.g = 1.0; y_axis.color.b = 0.0;
    
    // Z-axis (blue)
    visualization_msgs::msg::Marker z_axis = x_axis;
    z_axis.id = id + 2;
    Eigen::Vector3d z_dir = transform.linear() * Eigen::Vector3d(0, 0, scale);
    end.x = start.x + z_dir.x();
    end.y = start.y + z_dir.y();
    end.z = start.z + z_dir.z();
    z_axis.points = {start, end};
    z_axis.color.r = 0.0; z_axis.color.g = 0.0; z_axis.color.b = 1.0;
    
    axis_array.markers = {x_axis, y_axis, z_axis};
    return axis_array;
}

PlaneVisualizationData convertPlaneToVisualization(
    const Eigen::Vector4d& plane_coeffs,
    const Eigen::Vector3d& reference_point) {
    
    PlaneVisualizationData data;
    
    // Extract normal and distance
    Eigen::Vector3d normal = plane_coeffs.head<3>().normalized();
    double distance = plane_coeffs(3) / plane_coeffs.head<3>().norm();
    
    // Calculate center point on plane
    data.centre = -distance * normal;
    
    // Create orientation from normal
    Eigen::Vector3d default_normal(0, 0, 1);
    data.q = Eigen::Quaterniond::FromTwoVectors(default_normal, normal);
    
    // Set default scale
    data.scale = Eigen::Vector3d(2.0, 2.0, 0.05); // width, height, thickness
    
    return data;
}

visualization_msgs::msg::Marker createPlaneMarker(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    int id,
    const PlaneVisualizationData& data,
    const Eigen::Vector4d& color) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(5);
    
    marker.pose.position.x = data.centre.x();
    marker.pose.position.y = data.centre.y();
    marker.pose.position.z = data.centre.z();
    
    marker.pose.orientation.x = data.q.x();
    marker.pose.orientation.y = data.q.y();
    marker.pose.orientation.z = data.q.z();
    marker.pose.orientation.w = data.q.w();
    
    marker.scale.x = data.scale.x();
    marker.scale.y = data.scale.y();
    marker.scale.z = data.scale.z();
    
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    
    return marker;
}

visualization_msgs::msg::MarkerArray visualizeKeyFrames(
    g2o::SparseOptimizer* optimizer,
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray keyframe_array;
    std::vector<Eigen::Vector3d> kf_positions;
    std::vector<Eigen::Isometry3d> kf_poses;
    
    // Extract KeyFrame vertices
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto se3_vertex = dynamic_cast<g2o::VertexSE3Expmap*>(vertex_pair.second);
        if (se3_vertex) {
            // g2o::SE3Quat pose = se3_vertex->estimate();
            // Eigen::Vector3d position = pose.translation();
            // kf_positions.push_back(position);

            // **FIX: Get world-to-camera transform for proper position**
            g2o::SE3Quat Tcw = se3_vertex->estimate();
            g2o::SE3Quat Twc = Tcw.inverse();  // World-to-camera (camera pose in world)
            Eigen::Vector3d position = Twc.translation();  // Camera position in world
            kf_positions.push_back(position);
            
            // Convert to Isometry3d for axis visualization
            Eigen::Isometry3d iso_pose = Eigen::Isometry3d::Identity();
            iso_pose.translation() = position;
            iso_pose.linear() = Twc.rotation().toRotationMatrix();
            kf_poses.push_back(iso_pose);
        }
    }
    
    if (!kf_positions.empty()) {
        // KeyFrame positions as spheres
        auto kf_spheres = createSphereListMarker(
            frame_id, rclcpp::Time(0), marker_id++, kf_positions, 0.05, {1.0, 0.5, 0.0, 0.8});
        kf_spheres.ns = "local_ba/keyframes";
        keyframe_array.markers.push_back(kf_spheres);
        
        // KeyFrame axes
        for (const auto& pose : kf_poses) {
            auto axis_markers = createAxisMarker(frame_id, rclcpp::Time(0), marker_id, pose, 0.2);
            for (auto& axis_marker : axis_markers.markers) {
                axis_marker.ns = "local_ba/keyframe_axes";
                keyframe_array.markers.push_back(axis_marker);
            }
            marker_id += 3; // X, Y, Z axes
        }
    }
    
    return keyframe_array;
}

visualization_msgs::msg::MarkerArray visualizeMapPoints(
    g2o::SparseOptimizer* optimizer,
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray mappoint_array;
    std::vector<Eigen::Vector3d> mp_positions;
    
    // Extract MapPoint vertices
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto point_vertex = dynamic_cast<g2o::VertexSBAPointXYZ*>(vertex_pair.second);
        if (point_vertex) {
            mp_positions.push_back(point_vertex->estimate());
        }
    }
    
    if (!mp_positions.empty()) {
        auto mp_spheres = createSphereListMarker(
            frame_id, rclcpp::Time(0), marker_id++, mp_positions, 0.02, {1, 1, 0, 0.6});
        mp_spheres.ns = "local_ba/mappoints";
        mappoint_array.markers.push_back(mp_spheres);
    }
    
    return mappoint_array;
}

visualization_msgs::msg::MarkerArray visualizePlanes(
    g2o::SparseOptimizer* optimizer,
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray plane_array;
    
    // Extract Plane vertices
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto plane_vertex = dynamic_cast<g2o::VertexPlane*>(vertex_pair.second);
        if (plane_vertex) { //&& !plane_vertex->fixed()) { // Only show non-fixed planes (detected planes)
            g2o::Plane3D plane = plane_vertex->estimate();
            Eigen::Vector4d coeffs = plane.coeffs();
            
            auto plane_data = convertPlaneToVisualization(coeffs);
            auto plane_marker = createPlaneMarker(
                frame_id, rclcpp::Time(rclcpp::Clock().now()), marker_id++, plane_data, {0, 0.5, 1, 0.6});
            plane_marker.ns = "local_ba/detected_planes";
            plane_array.markers.push_back(plane_marker);
        }
    }
    
    return plane_array;
}

visualization_msgs::msg::MarkerArray visualizeMarkers(
    g2o::SparseOptimizer* optimizer,
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray marker_array;
    std::vector<Eigen::Vector3d> marker_positions;
    std::vector<Eigen::Isometry3d> marker_poses;
    
    // Extract Marker vertices (assuming they use SE3 representation)
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto se3_vertex = dynamic_cast<g2o::VertexSE3Expmap*>(vertex_pair.second);
        if (se3_vertex) {
            // Need to distinguish between KeyFrames and Markers based on ID range
            // Assuming markers have higher IDs (you may need to adjust this logic)
            int vertex_id = vertex_pair.first;
            if (vertex_id > 100000) { // Adjust this threshold based on your ID scheme
                g2o::SE3Quat pose = se3_vertex->estimate();
                Eigen::Vector3d position = pose.translation();
                marker_positions.push_back(position);
                
                Eigen::Isometry3d iso_pose = Eigen::Isometry3d::Identity();
                iso_pose.translation() = position;
                iso_pose.linear() = pose.rotation().toRotationMatrix();
                marker_poses.push_back(iso_pose);
            }
        }
    }
    
    if (!marker_positions.empty()) {
        // Marker positions as cubes
        auto marker_cubes = createBasicMarker(
            frame_id, rclcpp::Time(0), marker_id++, marker_positions, 
            visualization_msgs::msg::Marker::CUBE_LIST, 0.15, {1, 0, 1, 0.8});
        marker_cubes.ns = "local_ba/markers";
        marker_array.markers.push_back(marker_cubes);
        
        // Marker axes
        for (const auto& pose : marker_poses) {
            auto axis_markers = createAxisMarker(frame_id, rclcpp::Time(0), marker_id, pose, 0.15);
            for (auto& axis_marker : axis_markers.markers) {
                axis_marker.ns = "local_ba/marker_axes";
                marker_array.markers.push_back(axis_marker);
            }
            marker_id += 3;
        }
    }
    
    return marker_array;
}

visualization_msgs::msg::MarkerArray visualizeDetectedPlanesCentroid(
    g2o::SparseOptimizer* optimizer,
    const std::vector<ORB_SLAM3::Plane*>& detectedPlanes, // **ADDED PARAMETER**
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray plane_array;
    
    // **Create a map from vertex ID to detected plane for quick lookup**
    std::unordered_map<int, ORB_SLAM3::Plane*> vertexToDetectedPlane;
    for (const auto& plane : detectedPlanes) {
        if (plane) {
            vertexToDetectedPlane[plane->getOpIdG()] = plane;
        }
    }
    
    // Extract Plane vertices (non-fixed = detected planes)
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto plane_vertex = dynamic_cast<g2o::VertexPlane*>(vertex_pair.second);
        if (plane_vertex)// && !plane_vertex->fixed()) { // Only non-fixed planes (detected planes) 
        {
            // **GET THE ORIGINAL DETECTED PLANE OBJECT**
            auto it = vertexToDetectedPlane.find(vertex_pair.first);
            if (it != vertexToDetectedPlane.end()) {
                ORB_SLAM3::Plane* originalPlane = it->second;
                
                // **USE CENTROID FROM ORIGINAL DETECTED PLANE**
                Eigen::Vector3f centroid = originalPlane->getCentroid();
                Eigen::Vector3d normal = originalPlane->getGlobalEquation().normal();
                float length = originalPlane->getLength();
                std::cout << "🔍 Detected Plane #" << originalPlane->getId() 
                            << " - Length: " << length 
                            << " - Using scale: " << std::max(length, 0.2f) << std::endl;


                // Create enhanced plane visualization data
                PlaneVisualizationData plane_data;
                plane_data.centre = centroid.cast<double>();
                plane_data.q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), normal);
                plane_data.scale = Eigen::Vector3d(std::max(length, 0.5f), 2.5, 0.1);
                
                // Color based on association with BIM
                Eigen::Vector4d color;
                if (originalPlane->getBIMId() != 0) {
                    color = {0.0, 1.0, 0.0, 0.7}; // Green for associated planes
                } else {
                    color = {0.0, 0.5, 1.0, 0.6}; // Blue for unassociated planes
                }
                
                auto plane_marker = createPlaneMarker(
                    frame_id, rclcpp::Time(0), marker_id++, plane_data, color);
                plane_marker.ns = "local_ba/detected_planesCentroids";
                plane_array.markers.push_back(plane_marker);
                
                // **ADD DETECTED PLANE ID TEXT LABEL**
                visualization_msgs::msg::Marker planeLabel;
                planeLabel.header.frame_id = frame_id;
                planeLabel.header.stamp = rclcpp::Time(0);
                planeLabel.id = marker_id++;
                planeLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                planeLabel.action = visualization_msgs::msg::Marker::ADD;
                planeLabel.lifetime = rclcpp::Duration::from_seconds(0);
                planeLabel.ns = "local_ba/detected_planeCentroids_labels";
                
                // Position label above the plane
                planeLabel.pose.position.x = centroid.x();
                planeLabel.pose.position.y = centroid.y();
                planeLabel.pose.position.z = centroid.z() + 1.5;
                
                std::string labelText = "Det#" + std::to_string(originalPlane->getId());
                if (originalPlane->getBIMId() != 0) {
                    labelText += "↔BIM#" + std::to_string(originalPlane->getBIMId());
                }
                planeLabel.text = labelText;
                planeLabel.scale.z = 0.25;
                planeLabel.color.r = 1.0;
                planeLabel.color.g = 1.0;
                planeLabel.color.b = 0.0;
                planeLabel.color.a = 1.0;
                
                plane_array.markers.push_back(planeLabel);
                
                // std::cout << "🔵 Detected Plane #" << originalPlane->getId() 
                //           << " visualized at centroid: [" << centroid.transpose() << "]" << std::endl;
            }
        }
    }
    
    return plane_array;
}



visualization_msgs::msg::MarkerArray visualizeDetectedPlanes(
    g2o::SparseOptimizer* optimizer,
    const std::vector<ORB_SLAM3::Plane*>& detectedPlanes, // **ADDED PARAMETER**
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray plane_array;
    
    // **Create a map from vertex ID to detected plane for quick lookup**
    std::unordered_map<int, ORB_SLAM3::Plane*> vertexToDetectedPlane;
    for (const auto& plane : detectedPlanes) {
        if (plane) {
            vertexToDetectedPlane[plane->getOpIdG()] = plane;
        }
    }

    // Extract Plane vertices - **ONLY PROCESS PLANES THAT ARE IN THE DETECTED PLANES LIST**
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto plane_vertex = dynamic_cast<g2o::VertexPlane*>(vertex_pair.second);
        if (plane_vertex) {
            
            // **ONLY PROCESS IF THIS VERTEX IS IN THE DETECTED PLANES MAP**
            auto it = vertexToDetectedPlane.find(vertex_pair.first);
            if (it != vertexToDetectedPlane.end()) {
                // This vertex corresponds to a detected plane (not a BIM wall)
                ORB_SLAM3::Plane* originalPlane = it->second;
                
                g2o::Plane3D plane = plane_vertex->estimate();
                Eigen::Vector4d coeffs = plane.coeffs();
                
                auto plane_data = convertPlaneToVisualization(coeffs);
                
                // Color based on association with BIM
                Eigen::Vector4d color;
                if (originalPlane->getBIMId() != 0) {
                    color = {0.0, 1.0, 0.0, 0.7}; // Green for associated planes
                } else {
                    color = {0.0, 0.5, 1.0, 0.6}; // Blue for unassociated planes
                }
                
                auto plane_marker = createPlaneMarker(
                    frame_id, rclcpp::Time(rclcpp::Clock().now()), marker_id++, plane_data, color);
                plane_marker.ns = "local_ba/detected_planes";
                plane_array.markers.push_back(plane_marker);
                
                // **ADD DETECTED PLANE ID TEXT LABEL**
                visualization_msgs::msg::Marker planeLabel;
                planeLabel.header.frame_id = frame_id;
                planeLabel.header.stamp = rclcpp::Time(0);
                planeLabel.id = marker_id++;
                planeLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                planeLabel.action = visualization_msgs::msg::Marker::ADD;
                planeLabel.lifetime = rclcpp::Duration::from_seconds(0);
                planeLabel.ns = "local_ba/detected_plane_labels";
                
                // Position label at plane center (from convertPlaneToVisualization)
                planeLabel.pose.position.x = plane_data.centre.x();
                planeLabel.pose.position.y = plane_data.centre.y();
                planeLabel.pose.position.z = plane_data.centre.z() + 1.5;
                
                std::string labelText = "Det#" + std::to_string(originalPlane->getId());
                if (originalPlane->getBIMId() != 0) {
                    labelText += "↔BIM#" + std::to_string(originalPlane->getBIMId());
                }
                planeLabel.text = labelText;
                planeLabel.scale.z = 0.25;
                planeLabel.color.r = 1.0;
                planeLabel.color.g = 1.0;
                planeLabel.color.b = 0.0;
                planeLabel.color.a = 1.0;
                
                plane_array.markers.push_back(planeLabel);
            }
            // If vertex not found in detectedPlanes map, it's a BIM wall - skip it
        }
    }
    
    return plane_array;
}

visualization_msgs::msg::MarkerArray visualizeBIMWalls(
    g2o::SparseOptimizer* optimizer,
    const std::vector<ORB_SLAM3::Plane*>& bimWalls, // **ADDED PARAMETER**
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray bim_array;
    
    // **Create a map from vertex ID to BIM wall for quick lookup**
    std::unordered_map<int, ORB_SLAM3::Plane*> vertexToBimWall;
    for (const auto& bimWall : bimWalls) {
        if (bimWall) {
            vertexToBimWall[bimWall->getOpIdG()] = bimWall;
        }
    }
    
    // Extract BIM Wall vertices (fixed planes)
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto plane_vertex = dynamic_cast<g2o::VertexPlane*>(vertex_pair.second);
        if (plane_vertex && plane_vertex->fixed()) { // BIM walls are fixed
            
            // **GET THE ORIGINAL BIM WALL OBJECT**
            auto it = vertexToBimWall.find(vertex_pair.first);
            if (it != vertexToBimWall.end()) {
                ORB_SLAM3::Plane* originalBimWall = it->second;
                
                // **USE CENTROID FROM ORIGINAL BIM WALL**
                Eigen::Vector3f centroid = originalBimWall->getCentroid();
                // change the centroid y
                centroid.y() += 5.0; // Move down by 1 meter
                Eigen::Vector3d normal = originalBimWall->getGlobalEquation().normal();
                float length = originalBimWall->getLength();
                
                // Create enhanced plane visualization data
                PlaneVisualizationData plane_data;
                plane_data.centre = centroid.cast<double>();
                plane_data.q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), normal);
                plane_data.scale = Eigen::Vector3d(std::max(length, 1.0f), 3.0, 0.15); // Dynamic scaling
                
                // Color based on BIM ID
                Eigen::Vector4d color;
                if (originalBimWall->getBIMId() == 1) {
                    color = {1.0, 0.4, 0.0, 0.8}; // Bright orange for BIM #1
                } else if (originalBimWall->getBIMId() == 3) {
                    color = {1.0, 0.6, 0.2, 0.8}; // **CHANGED: Lighter orange for BIM #3**
                } else {
                    color = {1.0, 0.5, 0.0, 0.7}; // Default orange
                }
                
                auto plane_marker = createPlaneMarker(
                    frame_id, rclcpp::Time(0), marker_id++, plane_data, color);
                plane_marker.ns = "local_ba/bim_walls";
                bim_array.markers.push_back(plane_marker);
                
                // **ADD BIM WALL ID TEXT LABEL**
                visualization_msgs::msg::Marker wallLabel;
                wallLabel.header.frame_id = frame_id;
                wallLabel.header.stamp = rclcpp::Time(0);
                wallLabel.id = marker_id++;
                wallLabel.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                wallLabel.action = visualization_msgs::msg::Marker::ADD;
                wallLabel.lifetime = rclcpp::Duration::from_seconds(0);
                wallLabel.ns = "local_ba/bim_wall_labels";
                
                // Position label above the wall
                wallLabel.pose.position.x = centroid.x();
                wallLabel.pose.position.y = centroid.y();
                wallLabel.pose.position.z = centroid.z() + 1.8; // Above the wall
                
                wallLabel.text = "BIM#" + std::to_string(originalBimWall->getBIMId());
                wallLabel.scale.z = 0.3; // Text height
                wallLabel.color.r = 1.0;
                wallLabel.color.g = 1.0;
                wallLabel.color.b = 1.0;
                wallLabel.color.a = 1.0;
                
                bim_array.markers.push_back(wallLabel);
                
                // std::cout << "📍 BIM Wall #" << originalBimWall->getBIMId() 
                //           << " visualized at centroid: [" << centroid.transpose() << "]" << std::endl;
            }
        }
    }
    
    return bim_array;
}

/*visualization_msgs::msg::MarkerArray visualizeBIMWalls(
    g2o::SparseOptimizer* optimizer,
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray bim_array;
    
    // Extract BIM Wall vertices (fixed planes)
    for (const auto& vertex_pair : optimizer->vertices()) {
        auto plane_vertex = dynamic_cast<g2o::VertexPlane*>(vertex_pair.second);
        if (plane_vertex && plane_vertex->fixed()) { // BIM walls are fixed
            g2o::Plane3D plane = plane_vertex->estimate();
            Eigen::Vector4d coeffs = plane.coeffs();
            
            auto plane_data = convertPlaneToVisualization(coeffs);
            plane_data.scale = Eigen::Vector3d(4.0, 3.0, 0.1); // Larger scale for BIM walls
            
            auto plane_marker = createPlaneMarker(
                frame_id, rclcpp::Time(0), marker_id++, plane_data, {1, 0.5, 0, 0.7});
            plane_marker.ns = "local_ba/bim_walls";
            bim_array.markers.push_back(plane_marker);
        }
    }
    
    return bim_array;
}
*/

/*visualization_msgs::msg::MarkerArray visualizeEdges(
    g2o::SparseOptimizer* optimizer,
    const std::vector<ORB_SLAM3::Plane*>& detectedPlanes,
    const std::vector<ORB_SLAM3::Plane*>& bimWalls,
    const std::string& frame_id,
    int& marker_id) {
    
    visualization_msgs::msg::MarkerArray edge_array;
    
    std::vector<Eigen::Vector3d> kf_mp_lines;
    std::vector<Eigen::Vector3d> kf_plane_lines;
    std::vector<Eigen::Vector3d> bim_edges;
    
    // **STEP 1: CREATE DELETE MARKERS FOR PREVIOUS BIM EDGES**
    for (int old_id : g_previous_bim_edge_markers) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = frame_id;
        delete_marker.header.stamp = rclcpp::Time(0);
        delete_marker.id = old_id;
        delete_marker.ns = "local_ba/bim_alignment_edges";
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        edge_array.markers.push_back(delete_marker);
    }
    g_previous_bim_edge_markers.clear(); // Clear the old tracking
    
    // Process all edges
    for (const auto& edge : optimizer->edges()) {
        auto vertices = edge->vertices();
        
        // KeyFrame-MapPoint edges
        auto kf_mp_edge = dynamic_cast<ORB_SLAM3::EdgeSE3ProjectXYZ*>(edge);
        if (kf_mp_edge && vertices.size() >= 2) {
            auto mp_vertex = dynamic_cast<g2o::VertexSBAPointXYZ*>(vertices[0]);
            auto kf_vertex = dynamic_cast<g2o::VertexSE3Expmap*>(vertices[1]);
            
            if (mp_vertex && kf_vertex) {
                // **FIX: Use inverse transform for KeyFrame position**
                g2o::SE3Quat Tcw = kf_vertex->estimate();
                g2o::SE3Quat Twc = Tcw.inverse();  // World-to-camera (camera pose in world)
                Eigen::Vector3d kf_position = Twc.translation();  // Camera position in world
                
                kf_mp_lines.push_back(mp_vertex->estimate());
                kf_mp_lines.push_back(kf_position);  // Use corrected position
            }
        }
        
        // KeyFrame-Plane edges
        auto kf_plane_edge = dynamic_cast<ORB_SLAM3::EdgeVertexPlaneProjectSE3KF*>(edge);
        if (kf_plane_edge && vertices.size() >= 2) {
            auto kf_vertex = dynamic_cast<g2o::VertexSE3Expmap*>(vertices[0]);
            auto plane_vertex = dynamic_cast<g2o::VertexPlane*>(vertices[1]);
            
            if (kf_vertex && plane_vertex) {
                // **FIX: Use inverse transform for KeyFrame position**
                g2o::SE3Quat Tcw = kf_vertex->estimate();
                g2o::SE3Quat Twc = Tcw.inverse();  // World-to-camera (camera pose in world)
                Eigen::Vector3d kf_position = Twc.translation();  // Camera position in world
                
                // Calculate plane center for visualization
                g2o::Plane3D plane = plane_vertex->estimate();
                Eigen::Vector3d plane_center = -plane.coeffs()(3) * plane.normal();
                
                kf_plane_lines.push_back(kf_position);  // Use corrected position
                kf_plane_lines.push_back(plane_center);
            }
        }
        
        // BIM alignment edges (Edge2Planes) - Clean version
        auto plane_plane_edge = dynamic_cast<ORB_SLAM3::Edge2Planes*>(edge);
        if (plane_plane_edge && vertices.size() >= 2) {
            auto plane1_vertex = dynamic_cast<g2o::VertexPlane*>(vertices[0]);
            auto plane2_vertex = dynamic_cast<g2o::VertexPlane*>(vertices[1]);
            
            if (plane1_vertex && plane2_vertex) {
                // **NEW: Try to get actual centroids from original plane objects**
                Eigen::Vector3d center1, center2;
                bool found_centroids = false;
                
                // Look for BIM walls first
                for (const auto& bimWall : bimWalls) {
                    if (bimWall && bimWall->getOpIdG() == plane1_vertex->id()) {
                        center1 = bimWall->getCentroid().cast<double>();
                        found_centroids = true;
                        break;
                    }
                }
                for (const auto& bimWall : bimWalls) {
                    if (bimWall && bimWall->getOpIdG() == plane2_vertex->id()) {
                        center2 = bimWall->getCentroid().cast<double>();
                        break;
                    }
                }
                
                // Look for detected planes if not found in BIM walls
                for (const auto& detPlane : detectedPlanes) {
                    if (detPlane && detPlane->getOpIdG() == plane1_vertex->id()) {
                        center1 = detPlane->getCentroid().cast<double>();
                        found_centroids = true;
                        break;
                    }
                }
                for (const auto& detPlane : detectedPlanes) {
                    if (detPlane && detPlane->getOpIdG() == plane2_vertex->id()) {
                        center2 = detPlane->getCentroid().cast<double>();
                        break;
                    }
                }
                
                // **FALLBACK: Use old calculation if centroids not found**
                if (!found_centroids) {
                    g2o::Plane3D plane1 = plane1_vertex->estimate();
                    g2o::Plane3D plane2 = plane2_vertex->estimate();
                    center1 = -plane1.coeffs()(3) * plane1.normal();
                    center2 = -plane2.coeffs()(3) * plane2.normal();
                }
                
                // **USE CENTROID-BASED DISTANCE FOR VISUALIZATION DECISION**
                double distance = (center1 - center2).norm();
                
                if (distance < 0.01) {
                    // Very close centroids - show cross pattern
                    Eigen::Vector3d center = (center1 + center2) / 2.0;
                    double size = 0.5;
                    
                    // Simple cross pattern
                    bim_edges.push_back(center + Eigen::Vector3d(-size, 0, 0));
                    bim_edges.push_back(center + Eigen::Vector3d(size, 0, 0));
                    bim_edges.push_back(center + Eigen::Vector3d(0, -size, 0));
                    bim_edges.push_back(center + Eigen::Vector3d(0, size, 0));
                    
                    // std::cout << "🔴 BIM alignment (cross) at centroid: " << center.transpose() 
                    //         << " distance: " << distance << std::endl;
                } else {
                    // **LARGER DISTANCE - SHOW LINE BETWEEN CENTROIDS**
                    bim_edges.push_back(center1);
                    bim_edges.push_back(center2);
                    
                    // std::cout << "🔴 BIM alignment (line) between centroids: " 
                    //         << center1.transpose() << " <-> " << center2.transpose()
                    //         << " distance: " << distance << std::endl;
                }
            }
        }
       
    }
    
    // Create line markers for edges
    if (!kf_mp_lines.empty()) {
        auto kf_mp_marker = createLineListMarker(
            frame_id, rclcpp::Time(0), marker_id++, kf_mp_lines, 0.01, {0.5, 0.5, 0.5, 0.3});
        kf_mp_marker.ns = "local_ba/kf_mp_edges";
        edge_array.markers.push_back(kf_mp_marker);
    }
    
    if (!kf_plane_lines.empty()) {
        auto kf_plane_marker = createLineListMarker(
            frame_id, rclcpp::Time(0), marker_id++, kf_plane_lines, 0.02, {0, 1, 0, 0.6});
        kf_plane_marker.ns = "local_ba/kf_plane_edges";
        edge_array.markers.push_back(kf_plane_marker);
    }
    
    // if (!bim_edges.empty()) {   // PRINTING BIM EDGES WITHOUT CENTROID
    //     auto bim_marker = createLineListMarker(
    //         frame_id, rclcpp::Time(0), marker_id++, bim_edges, 0.1, {1, 0, 0, 1.0});
    //     bim_marker.ns = "local_ba/bim_alignment_edges";
    //     edge_array.markers.push_back(bim_marker);
    // }

    // **STEP 2: CREATE NEW BIM EDGE MARKERS AND TRACK THEIR IDs**
    if (!bim_edges.empty()) {   // PRINTING BIM EDGES WITH CENTROID
        int bim_marker_id = marker_id++;
        auto bim_marker = createLineListMarker(
            frame_id, rclcpp::Time(0), bim_marker_id, bim_edges, 0.05, {1, 0, 0, 1.0});
        bim_marker.ns = "local_ba/bim_alignment_edges";
        edge_array.markers.push_back(bim_marker);
        
        // **TRACK THIS MARKER ID FOR FUTURE CLEANUP**
        g_previous_bim_edge_markers.insert(bim_marker_id);
    }
    
    return edge_array;
}
*/

visualization_msgs::msg::MarkerArray visualizeEdges(
    g2o::SparseOptimizer* optimizer,
    const std::vector<ORB_SLAM3::Plane*>& bimWalls,
    const std::vector<ORB_SLAM3::Plane*>& detectedPlanes,
    const std::string& frame_id,
    int& marker_id,
    bool useCentroidsForEdges) {  // **NEW PARAMETER**
    
    visualization_msgs::msg::MarkerArray edge_array;
    
    std::vector<Eigen::Vector3d> kf_mp_lines;
    std::vector<Eigen::Vector3d> kf_plane_lines;
    std::vector<Eigen::Vector3d> bim_edges;
    
    // **STEP 1: CREATE DELETE MARKERS FOR PREVIOUS BIM EDGES**
    for (int old_id : g_previous_bim_edge_markers) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = frame_id;
        delete_marker.header.stamp = rclcpp::Time(0);
        delete_marker.id = old_id;
        delete_marker.ns = "local_ba/bim_alignment_edges";
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        edge_array.markers.push_back(delete_marker);
    }
    g_previous_bim_edge_markers.clear(); // Clear the old tracking
    
    // **CREATE LOOKUP MAPS FOR ORIGINAL PLANE OBJECTS**
    std::unordered_map<int, ORB_SLAM3::Plane*> vertexToBimWall;
    std::unordered_map<int, ORB_SLAM3::Plane*> vertexToDetectedPlane;
    
    for (const auto& bimWall : bimWalls) {
        if (bimWall) {
            vertexToBimWall[bimWall->getOpIdG()] = bimWall;
        }
    }
    
    for (const auto& detPlane : detectedPlanes) {
        if (detPlane) {
            vertexToDetectedPlane[detPlane->getOpIdG()] = detPlane;
        }
    }
    
    // Process all edges
    for (const auto& edge : optimizer->edges()) {
        auto vertices = edge->vertices();
        
        // KeyFrame-MapPoint edges
        auto kf_mp_edge = dynamic_cast<ORB_SLAM3::EdgeSE3ProjectXYZ*>(edge);
        if (kf_mp_edge && vertices.size() >= 2) {
            auto mp_vertex = dynamic_cast<g2o::VertexSBAPointXYZ*>(vertices[0]);
            auto kf_vertex = dynamic_cast<g2o::VertexSE3Expmap*>(vertices[1]);
            
            if (mp_vertex && kf_vertex) {
                // **FIX: Use inverse transform for KeyFrame position**
                g2o::SE3Quat Tcw = kf_vertex->estimate();
                g2o::SE3Quat Twc = Tcw.inverse();  // World-to-camera (camera pose in world)
                Eigen::Vector3d kf_position = Twc.translation();  // Camera position in world
                
                kf_mp_lines.push_back(mp_vertex->estimate());
                kf_mp_lines.push_back(kf_position);  // Use corrected position
            }
        }
        
        // KeyFrame-Plane edges
        auto kf_plane_edge = dynamic_cast<ORB_SLAM3::EdgeVertexPlaneProjectSE3KF*>(edge);
        if (kf_plane_edge && vertices.size() >= 2) {
            auto kf_vertex = dynamic_cast<g2o::VertexSE3Expmap*>(vertices[0]);
            auto plane_vertex = dynamic_cast<g2o::VertexPlane*>(vertices[1]);
            
            if (kf_vertex && plane_vertex) {
                // **FIX: Use inverse transform for KeyFrame position**
                g2o::SE3Quat Tcw = kf_vertex->estimate();
                g2o::SE3Quat Twc = Tcw.inverse();  // World-to-camera (camera pose in world)
                Eigen::Vector3d kf_position = Twc.translation();  // Camera position in world
                
                // **CHOOSE PLANE CENTER BASED ON PARAMETER**
                Eigen::Vector3d plane_center;
                if (useCentroidsForEdges) {
                    // Try to get centroid from original plane object
                    bool found_centroid = false;
                    
                    // Check BIM walls first
                    auto bim_it = vertexToBimWall.find(plane_vertex->id());
                    if (bim_it != vertexToBimWall.end()) {
                        plane_center = bim_it->second->getCentroid().cast<double>();
                        found_centroid = true;
                    } else {
                        // Check detected planes
                        auto det_it = vertexToDetectedPlane.find(plane_vertex->id());
                        if (det_it != vertexToDetectedPlane.end()) {
                            plane_center = det_it->second->getCentroid().cast<double>();
                            found_centroid = true;
                        }
                    }
                    
                    // Fallback to equation if centroid not found
                    if (!found_centroid) {
                        g2o::Plane3D plane = plane_vertex->estimate();
                        plane_center = -plane.coeffs()(3) * plane.normal();
                    }
                } else {
                    // Use optimized plane equation
                    g2o::Plane3D plane = plane_vertex->estimate();
                    plane_center = -plane.coeffs()(3) * plane.normal();
                }
                
                kf_plane_lines.push_back(kf_position);  // Use corrected position
                kf_plane_lines.push_back(plane_center);
            }
        }
        
        // **BIM alignment edges (Edge2Planes) - WITH BOOLEAN OPTION**
        auto plane_plane_edge = dynamic_cast<ORB_SLAM3::Edge2Planes*>(edge);
        if (plane_plane_edge && vertices.size() >= 2) {
            auto plane1_vertex = dynamic_cast<g2o::VertexPlane*>(vertices[0]);
            auto plane2_vertex = dynamic_cast<g2o::VertexPlane*>(vertices[1]);
            
            if (plane1_vertex && plane2_vertex) {
                Eigen::Vector3d center1, center2;
                std::string plane1_type = "Unknown", plane2_type = "Unknown";
                int plane1_id = -1, plane2_id = -1;
                
                if (useCentroidsForEdges) {
                    // **OPTION 1: Use original centroids from plane objects**
                    bool found_center1 = false, found_center2 = false;
                    
                    // Look for BIM walls first
                    auto bim1_it = vertexToBimWall.find(plane1_vertex->id());
                    auto bim2_it = vertexToBimWall.find(plane2_vertex->id());
                    
                    if (bim1_it != vertexToBimWall.end()) {
                        center1 = bim1_it->second->getCentroid().cast<double>();
                        center1.y() += 5.0; // Move BIM wall center down
                        plane1_type = "BIM";
                        plane1_id = bim1_it->second->getBIMId();
                        found_center1 = true;
                    }
                    if (bim2_it != vertexToBimWall.end()) {
                        center2 = bim2_it->second->getCentroid().cast<double>();
                        center2.y() += 5.0;
                        plane2_type = "BIM";
                        plane2_id = bim2_it->second->getBIMId();
                        found_center2 = true;
                    }
                    
                    // Look for detected planes if not found in BIM walls
                    if (!found_center1) {
                        auto det1_it = vertexToDetectedPlane.find(plane1_vertex->id());
                        if (det1_it != vertexToDetectedPlane.end()) {
                            center1 = det1_it->second->getCentroid().cast<double>();
                            plane1_type = "Detected";
                            plane1_id = det1_it->second->getId();
                            found_center1 = true;
                        }
                    }
                    if (!found_center2) {
                        auto det2_it = vertexToDetectedPlane.find(plane2_vertex->id());
                        if (det2_it != vertexToDetectedPlane.end()) {
                            center2 = det2_it->second->getCentroid().cast<double>();
                            plane2_type = "Detected";
                            plane2_id = det2_it->second->getId();
                            found_center2 = true;
                        }
                    }
                    
                    // Fallback to equation if centroids not found
                    if (!found_center1) {
                        g2o::Plane3D plane1 = plane1_vertex->estimate();
                        center1 = -plane1.coeffs()(3) * plane1.normal();
                    }
                    if (!found_center2) {
                        g2o::Plane3D plane2 = plane2_vertex->estimate();
                        center2 = -plane2.coeffs()(3) * plane2.normal();
                    }
                    
                    std::cout << "🔵 Using CENTROIDS for edge visualization" << std::endl;
                    
                } else {
                    // **OPTION 2: Use optimized plane equations**
                    g2o::Plane3D plane1 = plane1_vertex->estimate();
                    g2o::Plane3D plane2 = plane2_vertex->estimate();
                    center1 = -plane1.coeffs()(3) * plane1.normal();
                    center2 = -plane2.coeffs()(3) * plane2.normal();
                    
                    // Still get IDs for debugging
                    auto bim1_it = vertexToBimWall.find(plane1_vertex->id());
                    auto bim2_it = vertexToBimWall.find(plane2_vertex->id());
                    auto det1_it = vertexToDetectedPlane.find(plane1_vertex->id());
                    auto det2_it = vertexToDetectedPlane.find(plane2_vertex->id());
                    
                    if (bim1_it != vertexToBimWall.end()) {
                        plane1_type = "BIM"; plane1_id = bim1_it->second->getBIMId();
                    } else if (det1_it != vertexToDetectedPlane.end()) {
                        plane1_type = "Detected"; plane1_id = det1_it->second->getId();
                    }
                    
                    if (bim2_it != vertexToBimWall.end()) {
                        plane2_type = "BIM"; plane2_id = bim2_it->second->getBIMId();
                    } else if (det2_it != vertexToDetectedPlane.end()) {
                        plane2_type = "Detected"; plane2_id = det2_it->second->getId();
                    }
                    
                    std::cout << "🔴 Using OPTIMIZED EQUATIONS for edge visualization" << std::endl;
                }
                
                // **USE THE CHOSEN CENTERS FOR VISUALIZATION**
                double distance = (center1 - center2).norm();
                
                if (distance < 0.01) {
                    // Very close centers - show cross pattern
                    Eigen::Vector3d center = (center1 + center2) / 2.0;
                    double size = 0.5;
                    
                    // Simple cross pattern
                    bim_edges.push_back(center + Eigen::Vector3d(-size, 0, 0));
                    bim_edges.push_back(center + Eigen::Vector3d(size, 0, 0));
                    bim_edges.push_back(center + Eigen::Vector3d(0, -size, 0));
                    bim_edges.push_back(center + Eigen::Vector3d(0, size, 0));
                    
                    std::cout << "🔴 BIM alignment (cross) between " << plane1_type << "#" << plane1_id 
                              << " ↔ " << plane2_type << "#" << plane2_id 
                              << " at center: " << center.transpose() 
                              << " distance: " << distance << std::endl;
                } else {
                    // **LARGER DISTANCE - SHOW LINE BETWEEN CENTERS**
                    bim_edges.push_back(center1);
                    bim_edges.push_back(center2);
                    
                    std::cout << "🔴 BIM alignment (line) between " << plane1_type << "#" << plane1_id 
                              << " ↔ " << plane2_type << "#" << plane2_id << std::endl;
                    std::cout << "   Center1: " << center1.transpose() << std::endl;
                    std::cout << "   Center2: " << center2.transpose() << std::endl;
                    std::cout << "   Distance: " << distance << std::endl;
                    std::cout << "   Mode: " << (useCentroidsForEdges ? "CENTROID" : "OPTIMIZED") << std::endl;
                }
            }
        }
    }
    
    // Create line markers for edges
    if (!kf_mp_lines.empty()) {
        auto kf_mp_marker = createLineListMarker(
            frame_id, rclcpp::Time(0), marker_id++, kf_mp_lines, 0.01, {0.5, 0.5, 0.5, 0.3});
        kf_mp_marker.ns = "local_ba/kf_mp_edges";
        edge_array.markers.push_back(kf_mp_marker);
    }
    
    if (!kf_plane_lines.empty()) {
        auto kf_plane_marker = createLineListMarker(
            frame_id, rclcpp::Time(0), marker_id++, kf_plane_lines, 0.02, {0, 1, 0, 0.6});
        kf_plane_marker.ns = "local_ba/kf_plane_edges";
        edge_array.markers.push_back(kf_plane_marker);
    }
    
    // **STEP 2: CREATE NEW BIM EDGE MARKERS WITH MODE INDICATION**
    if (!bim_edges.empty()) {
        int bim_marker_id = marker_id++;
        
        // **DIFFERENT COLORS BASED ON MODE**
        Eigen::Vector4d edge_color;
        if (useCentroidsForEdges) {
            edge_color = {1, 0, 0, 1.0}; // RED for centroid mode
        } else {
            edge_color = {1, 0.5, 0, 1.0}; // ORANGE for optimized equation mode
        }
        
        auto bim_marker = createLineListMarker(
            frame_id, rclcpp::Time(0), bim_marker_id, bim_edges, 0.05, edge_color);
        
        // **DIFFERENT NAMESPACE BASED ON MODE**
        if (useCentroidsForEdges) {
            bim_marker.ns = "local_ba/bim_alignment_edges_centroid";
        } else {
            bim_marker.ns = "local_ba/bim_alignment_edges_optimized";
        }
        
        edge_array.markers.push_back(bim_marker);
        
        // **TRACK THIS MARKER ID FOR FUTURE CLEANUP**
        g_previous_bim_edge_markers.insert(bim_marker_id);
    }
    
    return edge_array;
}

visualization_msgs::msg::MarkerArray visualizeLocalBAGraph(
    g2o::SparseOptimizer* optimizer,
    const std::vector<ORB_SLAM3::Plane*>& bimWalls,      // **ADD BIM WALLS**
    const std::vector<ORB_SLAM3::Plane*>& detectedPlanes, // **ADD DETECTED PLANES**
    const std::string& frame_id,
    int initial_id) {
    
    visualization_msgs::msg::MarkerArray complete_array;
    int marker_id = initial_id;
    
    std::cout << "\n=== Visualizing Local BA Graph ===" << std::endl;
    std::cout << "Vertices: " << optimizer->vertices().size() << std::endl;
    std::cout << "Edges: " << optimizer->edges().size() << std::endl;
    std::cout << "BIM Walls: " << bimWalls.size() << std::endl;
    std::cout << "Detected Planes: " << detectedPlanes.size() << std::endl;
    
    // Visualize all components
    auto keyframe_markers = visualizeKeyFrames(optimizer, frame_id, marker_id);
    auto mappoint_markers = visualizeMapPoints(optimizer, frame_id, marker_id);
    // auto plane_markers = visualizePlanes(optimizer, frame_id, marker_id);
    auto marker_markers = visualizeMarkers(optimizer, frame_id, marker_id);
    
    // **NEW: Use the passed BIM walls and detected planes for better visualization**
    auto bim_markers = visualizeBIMWalls(optimizer, bimWalls, frame_id, marker_id);
    // auto detected_plane_markers = visualizeDetectedPlanes(optimizer, detectedPlanes, frame_id, marker_id);
    // For centroid mode
    auto detected_plane_markersCentroid = visualizeDetectedPlanesCentroid(optimizer, detectedPlanes, frame_id, marker_id);
   

    // auto edge_markers = visualizeEdges(optimizer, bimWalls, detectedPlanes, frame_id, marker_id, false); 
    // For centroid mode
    auto edge_markers = visualizeEdges(optimizer, bimWalls, detectedPlanes, frame_id, marker_id, true); 
    auto status_markers = visualizeFixedStatus(optimizer, frame_id, marker_id);
    
    complete_array.markers.insert(complete_array.markers.end(), 
                                 status_markers.markers.begin(), status_markers.markers.end());
   
    // Combine all markers
    complete_array.markers.insert(complete_array.markers.end(), 
                                 keyframe_markers.markers.begin(), keyframe_markers.markers.end());
    complete_array.markers.insert(complete_array.markers.end(), 
                                 mappoint_markers.markers.begin(), mappoint_markers.markers.end());
    // complete_array.markers.insert(complete_array.markers.end(), 
    //                              plane_markers.markers.begin(), plane_markers.markers.end());
    complete_array.markers.insert(complete_array.markers.end(), 
                                 marker_markers.markers.begin(), marker_markers.markers.end());
    complete_array.markers.insert(complete_array.markers.end(), 
                                 bim_markers.markers.begin(), bim_markers.markers.end());
    // complete_array.markers.insert(complete_array.markers.end(), 
    //                              detected_plane_markers.markers.begin(), detected_plane_markers.markers.end());
    // For centroid mode
    complete_array.markers.insert(complete_array.markers.end(), 
                                 detected_plane_markersCentroid.markers.begin(), detected_plane_markersCentroid.markers.end());
    complete_array.markers.insert(complete_array.markers.end(), 
                                 edge_markers.markers.begin(), edge_markers.markers.end());
    
    std::cout << "Total visualization markers: " << complete_array.markers.size() << std::endl;
    
    // Auto-publish the visualization
    publishLocalBAVisualization(complete_array);
    
    std::cout << "=== Local BA Graph Visualization Complete ===" << std::endl;
    
    return complete_array;
}

} // namespace vs_graphs_visualization
