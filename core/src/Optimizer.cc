/**
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2025-2026 SnT, University of Luxembourg
 * Asier Bikandi-Noya, Miguel Fernandez-Cortizas, Muhammad Shaheer, Ali
 * Tourani, Holger Voos, and Jose Luis Sanchez-Lopez.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 * 
 * This file is part of ivS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * ivS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
*/

#include "G2oTypes.h"
#include "Converter.h"
#include "Optimizer.h"
#include "OptimizableTypes.h"
#include "plane3d.h"
#include "vertex_plane.h"
#include "visualization_local_ba.h"
#include <lsqcpp/lsqcpp.hpp>
#include "bim_integration.h"
#include <chrono>
#include <vs_graphs/srv/save_transform.hpp>

#include <mutex>
#include <complex>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"

namespace ORB_SLAM3
{
    // Global BIM matching state
    static bool g_firstPlaneMatched = false;
    static bool g_secondPlaneMatched = false;
    static bool g_matchingComplete = false;
    static int g_firstDetectedPlaneId = -1;
    static int g_secondDetectedPlaneId = -1;

    Eigen::Matrix4d Optimizer::s_T_bim_to_detected = Eigen::Matrix4d::Identity();
    
    Eigen::Matrix4d Optimizer::GetBIMToDetectedTransform() {
        return s_T_bim_to_detected;
    }

    int BIM_wallId_1 = 0;
    int BIM_wallId_2 = 0;
    bool runAgraph = true;
    bool XYZcoord = false;
    bool justInitialAlignment = false;

    int g_BIM_wallId_1 = 0;
    int g_BIM_wallId_2 = 0;

    // static bool g_LOCALfirstPlaneMatched = false;
    // static bool g_LOCALsecondPlaneMatched = false;
    // static bool g_LOCALmatchingComplete = false;
    // static int g_LOCALfirstDetectedPlaneId = -1;
    // static int g_LOCALsecondDetectedPlaneId = -1;

    static std::vector<Plane*> g_localMatchedPlanes;
    static std::mutex g_localMatchedPlanesMutex;

    // NEW: Initial alignment completion state
    static bool g_initialAlignmentComplete = false;

    static std::vector<std::pair<Plane*, Plane*>> g_associations;

    // Optimized KeyFrame poses, exposed for post-optimization visualization
    static std::vector<KeyFrame*> g_optimizedKeyFrames;
    static std::mutex g_optimizedKeyFramesMutex;
    
    std::vector<KeyFrame*> GetOptimizedKeyFrames() {
        std::lock_guard<std::mutex> lock(g_optimizedKeyFramesMutex);
        return g_optimizedKeyFrames; // Returns a copy
    }

    // KeyFrame poses captured before optimization, for before/after comparison
    static std::vector<KeyFrame*> g_BEFOREoptimizedKeyFrames;
    static std::mutex g_BEFOREoptimizedKeyFramesMutex;
    
    std::vector<KeyFrame*> GetBEFOREOptimizedKeyFrames() {
        std::lock_guard<std::mutex> lock(g_BEFOREoptimizedKeyFramesMutex);
        return g_BEFOREoptimizedKeyFrames; // Returns a copy
    }

    //
    // --- ALTERNATIVE: original 7-DOF SE(3) parameterization (kept for reference) ---
    // xval = (qx, qy, qz, qw, tx, ty, tz) — allows out-of-plane 3D rotation.
    // Known issue: with rotated BIM wall normals (~5°+), solver finds a spurious Y-axis
    // tilt instead of the correct XY-plane rotation, causing post-alignment check to fail.
    //
    // struct PlaneError {
    //     static constexpr bool ComputesJacobian = false;
    //     std::vector<Eigen::Vector4d> source_planes;
    //     std::vector<Eigen::Vector4d> target_planes;
    //     PlaneError() = default;
    //     PlaneError(const std::vector<Eigen::Vector4d>& source, const std::vector<Eigen::Vector4d>& target)
    //         : source_planes(source), target_planes(target) {}
    //     template<typename Scalar, int Inputs, int Outputs>
    //     void operator()(const Eigen::Matrix<Scalar, Inputs, 1>& xval,
    //                     Eigen::Matrix<Scalar, Outputs, 1>& fval) const {
    //         Eigen::Quaternion<Scalar> rotation(xval(3), xval(0), xval(1), xval(2));
    //         Eigen::Matrix<Scalar, 3, 1> translation(xval(4), xval(5), xval(6));
    //         rotation.normalize();
    //         fval.resize(source_planes.size() * 4);
    //         for (size_t i = 0; i < source_planes.size(); ++i) {
    //             Eigen::Matrix<Scalar, 4, 1> s_params = source_planes[i].template cast<Scalar>();
    //             Eigen::Matrix<Scalar, 4, 1> a_params = target_planes[i].template cast<Scalar>();
    //             Eigen::Matrix<Scalar, 3, 1> transformed_normal_a = rotation * a_params.template head<3>();
    //             Eigen::Matrix<Scalar, 3, 1> normal_s = s_params.template head<3>();
    //             Scalar transformed_distance_a = -transformed_normal_a.dot(translation) + a_params(3);
    //             Scalar distance_s = s_params(3);
    //             fval.template segment<3>(i * 4) = transformed_normal_a - normal_s;
    //             fval(i * 4 + 3) = transformed_distance_a - distance_s;
    //         }
    //     }
    // };
    // SE(3) initial guess: initialGuess << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  (7 values)
    // SE(3) result extraction:
    //   Eigen::Quaterniond q(result.xval(3), result.xval(0), result.xval(1), result.xval(2));
    //   q.normalize();
    //   Eigen::Vector3d t(result.xval(4), result.xval(5), result.xval(6));
    //   T.block<3,3>(0,0) = q.toRotationMatrix(); T.block<3,1>(0,3) = t;
    // --- END SE(3) alternative ---

    // 3-DOF SE(2) parameterization: xval = (theta, tx, tz) for XYZcoord=false
    // or (theta, tx, ty) for XYZcoord=true.
    // theta is the rotation angle around the vertical axis (Y for ZXY, Z for XYZ).
    // This constrains the solver to pure in-plane rigid body motion.
    struct PlaneError {
        static constexpr bool ComputesJacobian = false;

        std::vector<Eigen::Vector4d> source_planes;
        std::vector<Eigen::Vector4d> target_planes;
        bool xyz_coord;

        PlaneError() = default;

        PlaneError(const std::vector<Eigen::Vector4d>& source, const std::vector<Eigen::Vector4d>& target, bool xyz)
            : source_planes(source), target_planes(target), xyz_coord(xyz) {}

        // LSQ Error Function — 3-DOF: xval(0)=theta, xval(1)=t_horiz, xval(2)=t_depth
        template<typename Scalar, int Inputs, int Outputs>
        void operator()(const Eigen::Matrix<Scalar, Inputs, 1>& xval,
                        Eigen::Matrix<Scalar, Outputs, 1>& fval) const {
            Scalar theta = xval(0);
            Scalar c = cos(theta);
            Scalar s = sin(theta);

            // Build 3D rotation matrix: rotation around vertical axis only
            // XYZcoord=false: vertical=Y, horizontal plane is X-Z → R = Ry(theta)
            // XYZcoord=true:  vertical=Z, horizontal plane is X-Y → R = Rz(theta)
            Eigen::Matrix<Scalar, 3, 3> R;
            Eigen::Matrix<Scalar, 3, 1> t;
            if (xyz_coord) {
                // Rz(theta): rotates X-Y plane
                R <<  c, -s, Scalar(0),
                      s,  c, Scalar(0),
                      Scalar(0), Scalar(0), Scalar(1);
                t << xval(1), xval(2), Scalar(0);
            } else {
                // Ry(theta): rotates X-Z plane
                R <<  c, Scalar(0),  s,
                      Scalar(0), Scalar(1), Scalar(0),
                     -s, Scalar(0),  c;
                t << xval(1), Scalar(0), xval(2);
            }

            fval.resize(source_planes.size() * 4);

            for (size_t i = 0; i < source_planes.size(); ++i) {
                Eigen::Matrix<Scalar, 4, 1> s_params = source_planes[i].template cast<Scalar>();
                Eigen::Matrix<Scalar, 4, 1> a_params = target_planes[i].template cast<Scalar>();

                Eigen::Matrix<Scalar, 3, 1> transformed_normal_a = R * a_params.template head<3>();
                Eigen::Matrix<Scalar, 3, 1> normal_s = s_params.template head<3>();

                Scalar transformed_distance_a = -transformed_normal_a.dot(t) + a_params(3);
                Scalar distance_s = s_params(3);

                fval.template segment<3>(i * 4) = transformed_normal_a - normal_s;
                fval(i * 4 + 3) = transformed_distance_a - distance_s;
            }
        }
    };

    Eigen::Matrix4d ComputeBIMToDetectedTransformation(
        const Plane* bimWall1, const Plane* bimWall3,
        const Plane* detectedPlane1, const Plane* detectedPlane2) {
        
        std::cout << "  → Computing transformation from BIM walls to detected planes using LSQCPP..." << std::endl;
        auto t_start = std::chrono::high_resolution_clock::now();

        // Get plane equations
        g2o::Plane3D bim1_eq = bimWall1->getGlobalEquation();
        g2o::Plane3D bim3_eq = bimWall3->getGlobalEquation();
        g2o::Plane3D det1_eq = detectedPlane1->getGlobalEquation();
        g2o::Plane3D det2_eq = detectedPlane2->getGlobalEquation();
        
        std::cout << "    BIM Wall #1: [" << bim1_eq.coeffs().transpose() << "]" << std::endl;
        std::cout << "    BIM Wall #3: [" << bim3_eq.coeffs().transpose() << "]" << std::endl;
        std::cout << "    Detected #1 (original): [" << det1_eq.coeffs().transpose() << "]" << std::endl;
        std::cout << "    Detected #2 (original): [" << det2_eq.coeffs().transpose() << "]" << std::endl;
        
        Eigen::Vector4d det1_modified_coeffs = det1_eq.coeffs();
        Eigen::Vector4d det2_modified_coeffs = det2_eq.coeffs();
        
        // Set Y component of normals to 0
        if (ORB_SLAM3::XYZcoord) {
            det1_modified_coeffs(2) = 0.0;  // For XYZ
            det2_modified_coeffs(2) = 0.0;  // For XYZ
        }else {
            det1_modified_coeffs(1) = 0.0;  // For Z-X-Y
            det2_modified_coeffs(1) = 0.0;  // For Z-X-Y
        }
        
        // Renormalize the normals after modification
        Eigen::Vector3d det1_normal = det1_modified_coeffs.head<3>();
        Eigen::Vector3d det2_normal = det2_modified_coeffs.head<3>();
        
        det1_normal.normalize();
        det2_normal.normalize();
        
        // Update the coefficients with normalized normals
        det1_modified_coeffs.head<3>() = det1_normal;
        det2_modified_coeffs.head<3>() = det2_normal;
        
        
        // Prepare data for optimization using modified plane equations
        std::vector<Eigen::Vector4d> source_planes;
        std::vector<Eigen::Vector4d> target_planes;
        
        source_planes.push_back(bim1_eq.coeffs());
        source_planes.push_back(bim3_eq.coeffs());
        target_planes.push_back(det1_modified_coeffs);  // Use modified version
        target_planes.push_back(det2_modified_coeffs);  // Use modified version
        
        // 3-DOF SE(2) optimization: (theta, t_horiz, t_depth)
        PlaneError plane_error(source_planes, target_planes, ORB_SLAM3::XYZcoord);

        lsqcpp::LeastSquaresAlgorithm<
            double, -1, -1,
            PlaneError,
            lsqcpp::GaussNewtonMethod<lsqcpp::DenseSVDSolver>,
            lsqcpp::ArmijoBacktracking,
            lsqcpp::CentralDifferences> optimizer;

        optimizer.setObjective(plane_error);

        Eigen::VectorXd initialGuess(3);
        initialGuess << 0.0, 0.0, 0.0;  // theta=0, tx=0, tz=0

        optimizer.setMaximumIterations(100);
        optimizer.setMinimumGradientLength(1e-5);
        optimizer.setMinimumStepLength(1e-5);
        optimizer.setVerbosity(1);

        auto result = optimizer.minimize(initialGuess);

        double theta = result.xval(0);
        double c = std::cos(theta);
        double s = std::sin(theta);

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        if (ORB_SLAM3::XYZcoord) {
            // Rz(theta)
            T(0,0) =  c;  T(0,1) = -s;
            T(1,0) =  s;  T(1,1) =  c;
            T(0,3) = result.xval(1);
            T(1,3) = result.xval(2);
        } else {
            // Ry(theta)
            T(0,0) =  c;  T(0,2) =  s;
            T(2,0) = -s;  T(2,2) =  c;
            T(0,3) = result.xval(1);
            T(2,3) = result.xval(2);
        }
        
        double rotation_angle = std::acos(std::clamp((T.block<3,3>(0,0).trace() - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / M_PI;
        double translation_magnitude = T.block<3,1>(0,3).norm();
        
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        return T;
    }

   
    void PerformBIMDetectedMultiAssociation(
        const std::vector<Plane*>& vpBIMWalls, 
        const std::vector<Plane*>& allPlanesVec,
        double association_threshold,
        double centroid_distance_threshold) {
        
        
        
        std::set<int> usedDetectedPlaneIds;
        for (const auto& assoc : g_associations) {
            if (assoc.second) { // detected plane
                usedDetectedPlaneIds.insert(assoc.second->getId());
            }
        }
        
        auto assoc_it = g_associations.begin();
        while (assoc_it != g_associations.end()) {
            if (assoc_it->second) { // Check detected plane
                bool detectedPlaneExists = false;
                for (const auto& detectedPlane : allPlanesVec) {
                    if (detectedPlane && 
                        detectedPlane->getId() == assoc_it->second->getId() &&
                        detectedPlane->getPlaneType() != Plane::planeVariant::UNDEFINED) {
                        detectedPlaneExists = true;
                        break;
                    }
                }
                
                if (!detectedPlaneExists) {
                    
                    // Remove from used set
                    usedDetectedPlaneIds.erase(assoc_it->second->getId());
                    
                    // Remove association
                    assoc_it = g_associations.erase(assoc_it);
                } else {
                    ++assoc_it;
                }
            } else {
                ++assoc_it;
            }
        }
        
        for (const auto& bimWall : vpBIMWalls) {
            if (!bimWall) continue;
            
            
            
            Eigen::Vector3f bim_centroid = bimWall->getCentroid();
            g2o::Plane3D bim_plane = bimWall->getGlobalEquation();
            
            std::vector<std::pair<Plane*, double>> validCandidates;
            
            // Find ALL matching detected planes for this BIM wall
            for (const auto& detectedPlane : allPlanesVec) {
                if (!detectedPlane || detectedPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                    continue;
                
                if (usedDetectedPlaneIds.find(detectedPlane->getId()) != usedDetectedPlaneIds.end()) {
                    continue;
                }
                
                g2o::Plane3D detected_plane_eq = detectedPlane->getGlobalEquation();
                Eigen::Vector3d error = bim_plane.ominus(detected_plane_eq);
                Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
                double plane_distance = sqrt(error.transpose() * cov * error);
                
                if (std::isnan(plane_distance) || plane_distance < 1e-3) {
                    plane_distance = sqrt(error.transpose() * cov * error);
                }
                
                Eigen::Vector3f detected_centroid = detectedPlane->getCentroid();
                double centroid_distance = (bim_centroid - detected_centroid).cast<double>().norm();
                
                Eigen::Vector3d centroid_diff = (bim_centroid - detected_centroid).cast<double>();
                Eigen::Vector3d detected_normal = detectedPlane->getGlobalEquation().normal().normalized();

                double d_parallel = std::abs(centroid_diff.dot(detected_normal)); // parallel to the normal
                double d_total = centroid_diff.norm();
                double d_perp = std::sqrt(d_total * d_total - d_parallel * d_parallel); // perpendicular to the normal
                double d_percentage = d_perp / d_total;

                
                bool plane_criterion = (plane_distance < association_threshold);
                bool centroid_criterion = (centroid_distance < centroid_distance_threshold);
                // bool centroid_criterion = (d_percentage > 0.9);

                
                if (plane_criterion && centroid_criterion) {
                    // double normalized_plane_dist = plane_distance / association_threshold;
                    // double normalized_centroid_dist = centroid_distance / centroid_distance_threshold;
                    
                    // // Weight factors
                    double plane_weight = 0.7;      // 70% weight on plane equation
                    double centroid_weight = 0.3;   // 30% weight on centroid distance
                    
                    double combined_score = plane_weight * plane_distance + 
                                        centroid_weight * d_perp;

                    
                    validCandidates.push_back(std::make_pair(detectedPlane, combined_score));
                    // print plane_distance and centroid_distance

                } else {
                    // print id
                }
            }
            
            if (!validCandidates.empty()) {
                // Sort candidates by combined score (best first)
                std::sort(validCandidates.begin(), validCandidates.end(), 
                        [](const std::pair<Plane*, double>& a, const std::pair<Plane*, double>& b) {
                            return a.second < b.second; // Lower score is better
                        });
                
                
                for (const auto& candidate : validCandidates) {
                    Plane* detectedPlane = candidate.first;
                    double combined_score = candidate.second;
                    
                    if (usedDetectedPlaneIds.find(detectedPlane->getId()) != usedDetectedPlaneIds.end()) {
                        std::cout << "      ⚠️  Detected Plane #" << detectedPlane->getId() 
                                << " was used by another BIM wall during this iteration - skipping" << std::endl;
                        continue;
                    }
                    
                    g_associations.push_back(std::make_pair(bimWall, detectedPlane));
                    usedDetectedPlaneIds.insert(detectedPlane->getId());
                    
                }
                
                
            } else {
            }
        }

        
        std::map<int, std::vector<std::pair<int, double>>> bimToDetectedMap;
        for (const auto& assoc : g_associations) {
            if (assoc.first && assoc.second) {
                // Calculate score for display (approximate)
                g2o::Plane3D bim_eq = assoc.first->getGlobalEquation();
                g2o::Plane3D det_eq = assoc.second->getGlobalEquation();
                Eigen::Vector3d error = bim_eq.ominus(det_eq);
                double plane_distance = error.norm();
                
                bimToDetectedMap[assoc.first->getBIMId()].push_back(
                    std::make_pair(assoc.second->getId(), plane_distance));
            }
        }
        
        for (const auto& group : bimToDetectedMap) {
            for (size_t i = 0; i < group.second.size(); ++i) {
                if (i > 0) std::cout << ", ";
            }
        }
    }
   
   void PerformBIMDetectedDataAssociation(
        const std::vector<Plane*>& vpBIMWalls, 
        const std::vector<Plane*>& allPlanesVec,
        double association_threshold,
        double centroid_distance_threshold) {
        
        // Track which detected planes are already associated
        std::set<int> usedDetectedPlaneIds;
        for (const auto& assoc : g_associations) {
            if (assoc.second) { // detected plane
                usedDetectedPlaneIds.insert(assoc.second->getId());
            }
        }
        
        // Loop through all BIM walls
        for (const auto& bimWall : vpBIMWalls) {
            if (!bimWall) continue;
            
            
            // Check if this BIM wall is already in associations
            bool alreadyAssociated = false;
            auto assoc_it = g_associations.begin();
            while (assoc_it != g_associations.end()) {
                if (assoc_it->first && assoc_it->first->getBIMId() == bimWall->getBIMId()) {
                    
                    bool detectedPlaneExists = false;
                    if (assoc_it->second) {
                        for (const auto& detectedPlane : allPlanesVec) {
                            if (detectedPlane && 
                                detectedPlane->getId() == assoc_it->second->getId() &&
                                detectedPlane->getPlaneType() != Plane::planeVariant::UNDEFINED) {
                                detectedPlaneExists = true;
                                break;
                            }
                        }
                    }
                    
                    if (detectedPlaneExists) {
                        alreadyAssociated = true;
                        ++assoc_it; // Move to next association
                    } else {
                        
                        // Remove this association from usedDetectedPlaneIds if it was there
                        if (assoc_it->second) {
                            usedDetectedPlaneIds.erase(assoc_it->second->getId());
                        }
                        
                        assoc_it = g_associations.erase(assoc_it); // erase() returns iterator to next element
                    }
                } else {
                    ++assoc_it; // Move to next association
                }
            }
            
            if (alreadyAssociated) {
                continue; // Skip this BIM wall - it has a valid association
            }
            
            Eigen::Vector3f bim_centroid = bimWall->getCentroid();
            g2o::Plane3D bim_plane = bimWall->getGlobalEquation();
            
            
            double best_combined_score = std::numeric_limits<double>::max();
            Plane* best_detected_plane = nullptr;
            double best_plane_distance = 0.0;
            double best_centroid_distance = 0.0;
            
            // Find best matching detected plane
            for (const auto& detectedPlane : allPlanesVec) {
                if (!detectedPlane || detectedPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                    continue;
                
                // Skip planes already used
                if (usedDetectedPlaneIds.find(detectedPlane->getId()) != usedDetectedPlaneIds.end()) {
                    continue;
                }
                
                g2o::Plane3D detected_plane_eq = detectedPlane->getGlobalEquation();
                Eigen::Vector3d error = bim_plane.ominus(detected_plane_eq);
                Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
                double plane_distance = sqrt(error.transpose() * cov * error);
                
                if (std::isnan(plane_distance) || plane_distance < 1e-3) {
                    plane_distance = sqrt(error.transpose() * cov * error);
                }
                
                Eigen::Vector3f detected_centroid = detectedPlane->getCentroid();
                double centroid_distance = (bim_centroid - detected_centroid).cast<double>().norm();
                
                
                bool plane_criterion = (plane_distance < association_threshold);
                bool centroid_criterion = (centroid_distance < centroid_distance_threshold);
                
                if (plane_criterion && centroid_criterion) {
                    double normalized_plane_dist = plane_distance / association_threshold;
                    double normalized_centroid_dist = centroid_distance / centroid_distance_threshold;
                    
                    // Weight factors
                    double plane_weight = 0.7;      // 70% weight on plane equation
                    double centroid_weight = 0.3;   // 30% weight on centroid distance
                    
                    double combined_score = plane_weight * normalized_plane_dist + 
                                        centroid_weight * normalized_centroid_dist;
                    
                    
                    if (combined_score < best_combined_score) {
                        best_combined_score = combined_score;
                        best_detected_plane = detectedPlane;
                        best_plane_distance = plane_distance;
                        best_centroid_distance = centroid_distance;
                    }
                } else {
                }
            }
            
            // Check if best match was found
            if (best_detected_plane) {
                g_associations.push_back(std::make_pair(bimWall, best_detected_plane));
                usedDetectedPlaneIds.insert(best_detected_plane->getId());
                
            } else {
            }
        }
        
        
        // Print all current associations
        for (const auto& assoc : g_associations) {
            if (assoc.first && assoc.second) {
                Eigen::Vector3f bim_c = assoc.first->getCentroid();
                Eigen::Vector3f det_c = assoc.second->getCentroid();
                double final_centroid_dist = (bim_c - det_c).cast<double>().norm();
                
            }
        }
    }
   
 
    static Eigen::Vector4d transform_plane(const Eigen::Vector4d& plane_coeffs,
                                const Eigen::Isometry3d& transform) {
        // Extract normal and distance
        Eigen::Vector3d normal = plane_coeffs.head<3>();
        double distance = plane_coeffs(3);
        
        // Transform the normal
        Eigen::Vector3d transformed_normal = transform.rotation() * normal;
        
        // Transform a point on the plane
        Eigen::Vector3d point_on_plane = -distance * normal.normalized();
        Eigen::Vector3d transformed_point = transform * point_on_plane;
        
        // Calculate new distance
        double new_distance = -transformed_normal.normalized().dot(transformed_point);
        
        // Return transformed plane equation
        Eigen::Vector4d transformed_plane;
        transformed_plane.head<3>() = transformed_normal.normalized();
        transformed_plane(3) = new_distance;
        
        return transformed_plane;
    }
    
    void ApplyTransformationToAllBIMWalls(const std::vector<Plane*>& bimWalls, 
                                const Eigen::Matrix4d& transformation) {
        std::cout << "\n=== Applying Transformation to All BIM Walls ===" << std::endl;
        
        // Convert Matrix4d to Isometry3d for the transform_plane function
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.matrix() = transformation;
        
        for (Plane* bimWall : bimWalls) {
            if (!bimWall) continue;
            
            // Get original plane equation
            g2o::Plane3D original_eq = bimWall->getGlobalEquation();
            Eigen::Vector4d original_coeffs = original_eq.coeffs();
            
            // Get original centroid
            Eigen::Vector3f original_centroid = bimWall->getCentroid();
            
            
            Eigen::Vector4d transformed_coeffs = transform_plane(original_coeffs, transform);
            
            // Create homogeneous coordinates for centroid: [x, y, z, 1]
            Eigen::Vector4d centroid_homogeneous;
            centroid_homogeneous << original_centroid.cast<double>(), 1.0;
            
            // Apply transformation matrix directly (same as transform_plane does for points)
            Eigen::Vector4d transformed_centroid_homo = transformation * centroid_homogeneous;
            
            // Extract 3D coordinates (ignore homogeneous coordinate)
            Eigen::Vector3d transformed_centroid = transformed_centroid_homo.head<3>();
            
            // TODO: the 1.5 offset is hardcoded; derive it from wall height instead
            if (ORB_SLAM3::XYZcoord) {
                transformed_centroid.z() = 1.5;  // For XYZ
            }else {
                transformed_centroid.y() = -1.5;  // For Z-X-Y
            }

            // Create new plane equation
            g2o::Plane3D transformed_eq(transformed_coeffs);
            
            // Update the plane equation
            bimWall->setGlobalEquation(transformed_eq);
            
            bimWall->setCentroid(transformed_centroid.cast<float>());
            
        }
        
    }

    bool CheckBIMAlignment(const Plane* bimWall1, const Plane* bimWall3,
                        const Plane* detectedPlane1, const Plane* detectedPlane2,
                        double threshold) {
        
        // Get transformed BIM equations and detected plane equations
        g2o::Plane3D bim1_eq = bimWall1->getGlobalEquation();      // Now transformed
        g2o::Plane3D bim3_eq = bimWall3->getGlobalEquation();      // Now transformed
        g2o::Plane3D det1_eq = detectedPlane1->getGlobalEquation();
        g2o::Plane3D det2_eq = detectedPlane2->getGlobalEquation();

        // // print values of det1_eq and det2_eq
        
        // Calculate alignment errors using ominus (same as Edge2Planes)
        Eigen::Vector3d error1 = bim1_eq.ominus(det1_eq);  // BIM Wall #1 vs Detected Plane #1
        Eigen::Vector3d error2 = bim3_eq.ominus(det2_eq);  // BIM Wall #3 vs Detected Plane #2
        
        // Calculate distances
        double distance1 = error1.norm();
        double distance2 = error2.norm();
        
        
        bool aligned = (distance1 < threshold) && (distance2 < threshold);
        
        if (aligned) {
            std::cout << "✅ BIM alignment SUCCESSFUL!" << std::endl;
        } else {
            std::cout << "❌ BIM alignment FAILED!" << std::endl;
            std::cout << "    Distances exceed threshold. Consider:" << std::endl;
            std::cout << "    - Increasing threshold" << std::endl;
            std::cout << "    - Improving plane matching" << std::endl;
            std::cout << "    - Checking BIM coordinate system" << std::endl;
        }
        
        return aligned;
    }


    
    bool sortByVal(const pair<MapPoint *, int> &a, const pair<MapPoint *, int> &b)
    {
        return (a.second < b.second);
    }

    // MODIFY
    void Optimizer::GlobalAGraphBundleAdjustment(Atlas *pAtlas, Map *pMap, int nIterations, bool *pbStopFlag,
                                           const unsigned long nLoopKF, const bool bRobust,
                                           double markerImpact)
    {
        std::vector<ORB_SLAM3::Door *> allDoors = pMap->GetAllDoors();
        std::vector<ORB_SLAM3::Room *> allRooms = pMap->GetAllRooms();
        std::vector<ORB_SLAM3::Plane *> allPlanes = pMap->GetAllPlanes();
        // std::vector<ORB_SLAM3::Plane *> allBimPlanes = ->GetAllBIMWalls();
        std::vector<Plane*> bimWalls = pAtlas->GetBIMDatabase().GetWallsBIM();
        std::vector<ORB_SLAM3::Marker *> allMarkers = pMap->GetAllMarkers();
        std::vector<ORB_SLAM3::MapPoint *> allMapPoints = pMap->GetAllMapPoints();
        std::vector<ORB_SLAM3::KeyFrame *> allKeyFrames = pMap->GetAllKeyFrames();

        std::cout << "Global Bundle Adjustment: " << allKeyFrames.size() << " KeyFrames, "
                  << allMapPoints.size() << " MapPoints, "
                  << allMarkers.size() << " Markers, "
                  << allPlanes.size() << " Planes, "
                  << allDoors.size() << " Doors, "
                  << allRooms.size() << " Rooms, "
                  << bimWalls.size() << " BIM Walls" << std::endl;
        // Log the info of the first wall
        if (!bimWalls.empty())
        {
            std::cout << "First BIM Wall: " << bimWalls[0]->getBIMId() << std::endl;
        }

        // BundleAdjustment(allKeyFrames, allMapPoints, allMarkers, allPlanes, allDoors,
                        //  allRooms, nIterations, pbStopFlag, nLoopKF, bRobust, markerImpact);
        AGraphBundleAdjustment(allKeyFrames, allMapPoints, allMarkers, allPlanes, allDoors,
                         allRooms, bimWalls, nIterations, pbStopFlag, nLoopKF, bRobust, markerImpact);
    }

    void Optimizer::AGraphBundleAdjustment (const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                     const vector<Marker *> &allMarkersVec, vector<Plane *> &allPlanesVec,
                                     const vector<Door *> &allDoorsVec, const vector<Room *> &vpRooms, const vector<Plane *> &vpBIMWalls, int nIterations,
                                     bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust, double markerImpact)
    {
        SystemParams *sysParams = SystemParams::GetParams();
        vector<bool> vbNotIncludedMP;
        vbNotIncludedMP.resize(vpMP.size());

        Map *pMap = vpKFs[0]->GetMap();

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        g_BIM_wallId_1 = BIM_wallId_1;
        g_BIM_wallId_2 = BIM_wallId_2;

        
        // Filter out horizontal planes (normal close to Z axis)
        std::vector<Plane*> filteredPlanes;
        for (const auto& plane : allPlanesVec) {
            if (!plane) continue;
            Eigen::Vector3d normal = plane->getGlobalEquation().normal().normalized();

            if (ORB_SLAM3::XYZcoord) {
                // If normal is NOT close to vertical (Z axis), keep it
                if (std::abs(normal.z()) < 0.9) { // adjust threshold as needed
                    filteredPlanes.push_back(plane);
                }
            }else {
                // If normal is NOT close to vertical (Y axis), keep it
                if (std::abs(normal.y()) < 0.9) { // adjust threshold as needed
                    filteredPlanes.push_back(plane);
                }
            } 
        }

        allPlanesVec = filteredPlanes;

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        const int nExpectedSize = (vpKFs.size()) * vpMP.size();

        vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
        vpEdgesBody.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFBody;
        vpEdgeKFBody.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeBody;
        vpMapPointEdgeBody.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        // Set KeyFrame vertices (Global Optimization)
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKF->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKF->mnId);
            // vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
            optimizer.addVertex(vSE3);
            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }

        const float thHuber1D = sqrt(3.841);
        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);

        int nPlanes = 1;
        int nDoors = 1;
        int nRooms = 1;
        int maxOpId = 0;
        int nMarkers = 1;
        int nBIMWalls = 1;

        // Set MapPoint vertices (Global Optimization)
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            MapPoint *pMP = vpMP[i];
            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            const int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            // Update the maxOpId to hold the biggest value
            if (id > maxOpId)
                maxOpId = id;

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            int nEdges = 0;
            // SET EDGES
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
            {
                KeyFrame *pKF = mit->first;
                if (pKF->isBad() || pKF->mnId > maxKFid)
                    continue;
                if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
                    continue;
                nEdges++;

                const int leftIndex = get<0>(mit->second);

                if (leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)] < 0)
                {
                    const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    e->pCamera = pKF->mpCamera;

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKF);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else if (leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0) // Stereo observation
                {
                    const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
                    e->bf = pKF->mbf;

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKF);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                if (pKF->mpCamera2)
                {
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1 && rightIndex < pKF->mvKeysRight.size())
                    {
                        rightIndex -= pKF->NLeft;

                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);

                        Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
                        e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                        e->pCamera = pKF->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKF);
                        vpMapPointEdgeBody.push_back(pMP);
                    }
                }
            }

            if (nEdges == 0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            }
            else
            {
                vbNotIncludedMP[i] = false;
            }
        }

        // Markers (Global Optimization)
        for (const auto &vpMarker : allMarkersVec)
        {
            // Adding a vertex for each marker
            g2o::VertexSE3Expmap *vMarker = new g2o::VertexSE3Expmap();
            vMarker->setEstimate(g2o::SE3Quat(vpMarker->getGlobalPose().unit_quaternion().cast<double>(),
                                              vpMarker->getGlobalPose().translation().cast<double>()));
            int opIdG = maxOpId + nMarkers;
            vMarker->setId(opIdG);
            optimizer.addVertex(vMarker);
            nMarkers++;

            // Setting the Global Optimization ID for the marker
            vpMarker->setOpIdG(opIdG);

            /**
             * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex (SE3)
             * 🚧 [vS-Graphs v.2.0] This edge is not used anymore, in contrast to the previous version.
             * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll, pitch, yaw)
             */
        }

        maxOpId += nMarkers;

        // Planes (Global Optimization)
        for (const auto &vpPlane : allPlanesVec)
        {
            // Skip undefined planes (if not wall for now)
            if (vpPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                continue;
            // Adding a vertex for each plane
            g2o::VertexPlane *vPlane = new g2o::VertexPlane();
            int opIdG = maxOpId + nPlanes;
            vPlane->setId(opIdG);
            vPlane->setEstimate(vpPlane->getGlobalEquation()); //PASS G2O::plane3D type a-graph variable to this function
            
            if (sysParams->optimization.marginalize_planes)
                vPlane->setMarginalized(true);
            // vPlane->setFixed(true);
            optimizer.addVertex(vPlane);
            nPlanes++;

            // Setting the global optimization ID for the plane
            vpPlane->setOpIdG(opIdG);
            vpPlane->setBIMId(0); // Set BIM ID for the plane

            // Adding an edge between the plane and the keyframes
            const map<KeyFrame *, ORB_SLAM3::Plane::Observation> observations = vpPlane->getObservations();
            for (map<KeyFrame *, ORB_SLAM3::Plane::Observation>::const_iterator obsId = observations.begin(), obLast = observations.end(); obsId != obLast; obsId++)
            {
                KeyFrame *pKFi = obsId->first;
                ORB_SLAM3::Plane::Observation obs = obsId->second;

                if (pKFi->isBad())
                {
                    std::cout << "KF is bad" << std::endl;
                    vpPlane->eraseObservation(pKFi);
                    continue;
                }

                g2o::Plane3D planeLocalEquation = obs.localPlane;
                ORB_SLAM3::EdgeVertexPlaneProjectSE3KF *e = new ORB_SLAM3::EdgeVertexPlaneProjectSE3KF();

                if (optimizer.vertex(opIdG) && optimizer.vertex(pKFi->mnId))
                {
                    // adding plane-KF constraints -- is enabled for now
                    if (sysParams->optimization.plane_kf.enabled)
                    {
                        ORB_SLAM3::EdgeVertexPlaneProjectSE3KF *e = new ORB_SLAM3::EdgeVertexPlaneProjectSE3KF();
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                        
                        e->setInformation(Eigen::Matrix<double, 3, 3>::Identity() * 10000);
                        e->setMeasurement(planeLocalEquation);
                        optimizer.addEdge(e);
                    }

                    // adding plane-KF constraints with point observations -- is disabled for now
                    if (sysParams->optimization.plane_point.enabled)
                    {
                        // get the class index of the plane
                        int clsCloudIdx = Utils::getClassIdFromPlaneType(vpPlane->getPlaneType());
                        if (clsCloudIdx != -1)
                        {
                            // add the plane-point constraint
                            ORB_SLAM3::EdgeSE3KFPointToPlane *e = new ORB_SLAM3::EdgeSE3KFPointToPlane();
                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * obs.confidence * sysParams->optimization.plane_point.information_gain);
                            auto planeKF_information = Eigen::Matrix<double, 3, 3>::Identity() * obs.confidence * sysParams->optimization.plane_kf.information_gain;
                            // print Eigen::Matrix<double, 3, 3>::Identity() * obs.confidence * sysParams->optimization.plane_kf.information_gain
                      
                            e->setMeasurement(obs.Gij);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuber1D);
                            optimizer.addEdge(e);
                        }
                    }
                }
            }

            // 🚧 [vS-Graphs v.2.0] in contrast with the first version of visual S-Graphs, where there was an edge between
            // Markers and Planes, in this version we removed that edge
            // vector<Marker *> attachedMarkers = vpPlane->getMarkers();
            // for (const auto &planeMarker : attachedMarkers)
            // {
            //     // Adding an edge between the Plane and the Marker
            //     ORB_SLAM3::EdgeVertexPlaneProjectSE3M *e = new ORB_SLAM3::EdgeVertexPlaneProjectSE3M();
            //     e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
            //     e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(planeMarker->getOpIdG())));
            //     e->setInformation(Eigen::Matrix<double, 4, 4>::Identity());

            //     g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            //     e->setRobustKernel(rk);
            //     rk->setDelta(thHuber2D);
            //     optimizer.addEdge(e);
            // }
        }

        maxOpId += nPlanes;

        // Set Room vertices (Global Optimization)
        for (const auto &vpRoom : vpRooms)
        {
            // Adding a vertex for each room
            g2o::VertexSE3Expmap *vrtxRoom = new g2o::VertexSE3Expmap();

            int opIdG = maxOpId + nRooms;
            vrtxRoom->setId(opIdG);
            vrtxRoom->setEstimate(g2o::SE3Quat(Eigen::Quaterniond::Identity(),
                                               vpRoom->getRoomCenter().cast<double>()));
            optimizer.addVertex(vrtxRoom);
            nRooms++;

            // Setting the global optimization ID for the room
            vpRoom->setOpIdG(opIdG);

            // Get list of walls of the room
            vector<Plane *> walls = vpRoom->getWalls();
            if (walls.size() > 0)
                if (vpRoom->getIsCorridor())
                {
                    // Adding an edge between the room and the two walls
                    ORB_SLAM3::EdgeVertex2PlaneProjectSE3Room *e = new ORB_SLAM3::EdgeVertex2PlaneProjectSE3Room();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[0]->getOpIdG())));
                    e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[1]->getOpIdG())));
                    e->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                    optimizer.addEdge(e);
                }
                else
                {
                    // Adding an edge between the room and the two walls
                    ORB_SLAM3::EdgeVertex4PlaneProjectSE3Room *e = new ORB_SLAM3::EdgeVertex4PlaneProjectSE3Room();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[0]->getOpIdG())));
                    e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[1]->getOpIdG())));
                    e->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[2]->getOpIdG())));
                    e->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[3]->getOpIdG())));
                    e->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                    optimizer.addEdge(e);
                }
        }

        maxOpId += nRooms;

        // Set Door vertices (Global Optimization)
        for (const auto &vpRoom : vpRooms)
        {
            vector<Door *> doors = vpRoom->getDoors();
            for (const auto &door : doors)
            {
                // Adding a vertex for each door
                g2o::VertexSE3Expmap *vDoor = new g2o::VertexSE3Expmap();
                int opIdG = maxOpId + nDoors;
                vDoor->setId(opIdG);
                vDoor->setEstimate(g2o::SE3Quat(door->getGlobalPose().unit_quaternion().cast<double>(),
                                                door->getGlobalPose().translation().cast<double>()));
                optimizer.addVertex(vDoor);
                nDoors++;

                // Setting the local optimization ID for the door
                door->setOpIdG(opIdG);

                // Adding an edge between the room and the door
                ORB_SLAM3::EdgeSE3DoorProjectSE3Room *e = new ORB_SLAM3::EdgeSE3DoorProjectSE3Room();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(door->getOpIdG())));
                e->setInformation(Eigen::MatrixXd::Identity(6, 6));

                Eigen::Isometry3d relativePose = Eigen::Isometry3d::Identity();
                relativePose.matrix() = (dynamic_cast<g2o::VertexSE3Expmap *>((optimizer.vertex(vpRoom->getOpIdG())))->estimate().inverse() *
                                         dynamic_cast<g2o::VertexSE3Expmap *>((optimizer.vertex(door->getOpIdG())))->estimate())
                                            .to_homogeneous_matrix();
                e->setMeasurement(relativePose);

                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);
                optimizer.addEdge(e);
            }
        }

        maxOpId += nDoors;

        
        // MATCHING BETWEEN DETECTED PLANES AND BIM WALLS
        if (!g_initialAlignmentComplete && !vpBIMWalls.empty() && !allPlanesVec.empty()) {
            std::cout << "\n=== BIM-Detected Plane Matching ===" << std::endl;
            // Step 1: Match first detected plane (id=0) with BIM wall (id=1)
            if (!g_firstPlaneMatched) {
                for (const auto& detectedPlane : allPlanesVec) {
                    if (!detectedPlane || detectedPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                        continue;
                    
                    if (detectedPlane) {
                        // Check if we have BIM wall with id=1
                        for (const auto& bimWall : vpBIMWalls) {
                            if (bimWall && bimWall->getBIMId() == g_BIM_wallId_1) {
                                g_firstDetectedPlaneId = detectedPlane->getId();
                                g_firstPlaneMatched = true;
                                
                                std::cout << "✓ FIRST MATCH: Detected Plane #" << detectedPlane->getId() 
                                         << " matched with BIM Wall #" << bimWall->getBIMId() << std::endl;
                                break;
                            }
                        }
                        break;
                    }
                }
            }

            // Step 2: Find perpendicular plane to match with BIM wall id=3
            if (g_firstPlaneMatched && !g_secondPlaneMatched) {
                // Get the first matched plane
                Plane* firstPlane = nullptr;
                for (const auto& detectedPlane : allPlanesVec) {
                    if (detectedPlane && detectedPlane->getId() == g_firstDetectedPlaneId) {
                        firstPlane = detectedPlane;
                        break;
                    }
                }

                if (firstPlane) {
                    g2o::Plane3D firstPlaneEq = firstPlane->getGlobalEquation();
                    Eigen::Vector3d firstNormal = firstPlaneEq.normal();

                    // Look for a perpendicular plane
                    for (const auto& detectedPlane : allPlanesVec) {
                        if (!detectedPlane || detectedPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                            continue;
                        
                        if (detectedPlane->getId() == g_firstDetectedPlaneId)
                            continue; // Skip the first plane itself

                        g2o::Plane3D candidateEq = detectedPlane->getGlobalEquation();
                        Eigen::Vector3d candidateNormal = candidateEq.normal();

                        // Check if planes are perpendicular (dot product close to 0)
                        double dotProduct = abs(firstNormal.dot(candidateNormal));
                        double perpendicularThreshold = 0.2; // Adjust as needed (0 = perfect perpendicular)

                        if (dotProduct < perpendicularThreshold) {
                            // Check if we have BIM wall with id=3
                            for (const auto& bimWall : vpBIMWalls) {
                                if (bimWall && bimWall->getBIMId() == g_BIM_wallId_2) {
                                    g_secondDetectedPlaneId = detectedPlane->getId();
                                    g_secondPlaneMatched = true;
                                    
                                    std::cout << "✓ SECOND MATCH: Detected Plane #" << detectedPlane->getId() 
                                             << " (perpendicular to Plane #" << g_firstDetectedPlaneId 
                                             << ") matched with BIM Wall #" << bimWall->getBIMId() << std::endl;
                                    
                                    std::cout << "  → Dot product: " << dotProduct 
                                             << " (threshold: " << perpendicularThreshold << ")" << std::endl;
                                    
                                    // Print summary of both matches
                                    std::cout << "\n=== MATCHING SUMMARY ===" << std::endl;
                                    std::cout << "✓ Match 1: Detected Plane #" << g_firstDetectedPlaneId 
                                             << " ↔ BIM Wall #1" << std::endl;
                                    std::cout << "✓ Match 2: Detected Plane #" << g_secondDetectedPlaneId 
                                             << " ↔ BIM Wall #3" << std::endl;
                                    std::cout << "=== MATCHING COMPLETE ===" << std::endl;

                                    // Log the optimization id for both planes
                                    std::cout << "Detected Plane #" << g_firstDetectedPlaneId 
                                              << " OpIdG: " << firstPlane->getOpIdG() << std::endl;
                                    std::cout << "Detected Plane #" << g_secondDetectedPlaneId 
                                              << " OpIdG: " << detectedPlane->getOpIdG() << std::endl; 
                                    
                                    goto matching_complete; // Exit all loops
                                }
                            }
                        }
                    }
                }
            }

            matching_complete:
            
            // Status update
            if (!g_firstPlaneMatched) {
                std::cout << "⏳ Waiting for detected plane with id=0..." << std::endl;
            } else if (!g_secondPlaneMatched) {
                std::cout << "⏳ First plane matched. Waiting for perpendicular plane..." << std::endl;
                std::cout << "   → Looking for plane perpendicular to detected plane #" << g_firstDetectedPlaneId << std::endl;
            }
            std::cout << "=== End Matching ===" << std::endl;
        } else {
            if (vpBIMWalls.empty()) {
                std::cout << "❌ No BIM walls available." << std::endl;
            }
            if (allPlanesVec.empty()) {
                std::cout << "❌ No detected planes available." << std::endl;
            }
            std::cout << "=== End Matching ===" << std::endl;
        }

        // Find the matched detected planes and BIM walls
        Plane* detectedPlane1 = nullptr;  // Detected plane #0
        Plane* detectedPlane2 = nullptr;  // Detected plane #7
        Plane* bimWall1 = nullptr;        // BIM wall #1
        Plane* bimWall3 = nullptr;        // BIM wall #3

        if (!g_initialAlignmentComplete) {
            // Find detected planes by ID
            for (const auto& plane : allPlanesVec) {
                if (!plane || plane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                    continue;
                    
                if (plane->getId() == g_firstDetectedPlaneId) {
                    detectedPlane1 = plane;
                }
                if (plane->getId() == g_secondDetectedPlaneId) {
                    detectedPlane2 = plane;
                }
            }
        }
        
        
        // Find BIM walls by ID
        for (const auto& bimWall : vpBIMWalls) {
            if (!bimWall) continue;
            
            if (bimWall->getBIMId() == g_BIM_wallId_1) {
                bimWall1 = bimWall;
                std::cout << "BIM Wall #1: BIM ID = " << bimWall1->getBIMId() << ", OpIdG = " << bimWall1->getOpIdG() << std::endl;
            }
            if (bimWall->getBIMId() == g_BIM_wallId_2) {
                bimWall3 = bimWall;
                std::cout << "BIM Wall #3: BIM ID = " << bimWall3->getBIMId() << ", OpIdG = " << bimWall3->getOpIdG() << std::endl;
            }
        }      

        // Get the TF for intial alignment
        if (!g_initialAlignmentComplete && detectedPlane1 && detectedPlane2 && bimWall1 && bimWall3) {

            Eigen::Matrix4d T_bim_to_detected = ComputeBIMToDetectedTransformation(
                bimWall1, bimWall3,           // Source: BIM walls
                detectedPlane1, detectedPlane2 // Target: Detected planes
            );

            std::cout << "🔄 BIM-to-Detected Transformation Matrix:" << std::endl;
            std::cout << T_bim_to_detected << std::endl;
            
            // Extract rotation and translation for analysis (from the INVERTED matrix)
            Eigen::Matrix3d R1 = T_bim_to_detected.block<3,3>(0,0);
            Eigen::Vector3d t1 = T_bim_to_detected.block<3,1>(0,3);
            
            // Convert to angle-axis for easier interpretation
            Eigen::AngleAxisd angleAxis1(R1);
            std::cout << "📐 Rotation: " << angleAxis1.angle() * 180.0 / M_PI << "° around axis [" 
                    << angleAxis1.axis().transpose() << "]" << std::endl;

            // Invert the transformation to map BIM walls onto detected planes
            T_bim_to_detected = T_bim_to_detected.inverse();
            s_T_bim_to_detected = T_bim_to_detected;
            
            std::cout << "🔄 BIM-to-Detected Transformation Matrix:" << std::endl;
            std::cout << T_bim_to_detected << std::endl;
            
            // Extract rotation and translation for analysis (from the INVERTED matrix)
            Eigen::Matrix3d R = T_bim_to_detected.block<3,3>(0,0);
            Eigen::Vector3d t = T_bim_to_detected.block<3,1>(0,3);
            
            // Convert to angle-axis for easier interpretation
            Eigen::AngleAxisd angleAxis(R);
            std::cout << "📐 INVERTED Rotation: " << angleAxis.angle() * 180.0 / M_PI << "° around axis [" 
                    << angleAxis.axis().transpose() << "]" << std::endl;
            
            ApplyTransformationToAllBIMWalls(vpBIMWalls, T_bim_to_detected);  // ← THIS IS THE KEY FIX!
            
            // print equation for detectedPlane1, detectedPlane2, bimWall1, bimWall3
            std::cout << "Detected Plane #0 Equation: " << detectedPlane1->getGlobalEquation().coeffs().transpose() << std::endl;
            std::cout << "Detected Plane #X Equation: " << detectedPlane2->getGlobalEquation().coeffs().transpose() << std::endl;
            std::cout << "BIM Wall #1 Equation: " << bimWall1->getGlobalEquation().coeffs().transpose() << std::endl;
            std::cout << "BIM Wall #3 Equation: " << bimWall3->getGlobalEquation().coeffs().transpose() << std::endl;
            
            bool isTransformationGood = CheckBIMAlignment(
                bimWall1, bimWall3, 
                detectedPlane1, detectedPlane2,
                0.3 // alignment threshold
            );
            
            if (isTransformationGood) {
                g_initialAlignmentComplete = true;  
                // Set plane IDs for matched BIM walls
                // bimWall1->setId(detectedPlane1->getId()); // Set ID for BIM wall #1
                // bimWall3->setId(detectedPlane2->getId()); // Set ID for
                detectedPlane1->setBIMId(bimWall1->getBIMId()); // Set BIM ID 1 for detected plane #0
                detectedPlane2->setBIMId(bimWall3->getBIMId()); // Set BIM ID 3 for detected plane #x
                std::cout << "🎯 BIM transformation successful! Proceeding with edge creation..." << std::endl;
            } else {
                std::cout << "⚠️  BIM transformation failed. SKIPPING OPTIMIZATION until alignment is correct." << std::endl;
                std::cout << "    → Optimization will be skipped this iteration" << std::endl;
                std::cout << "    → BIM alignment must succeed before optimization can proceed" << std::endl;
                
                return;
            }
        }

        if (justInitialAlignment) {
            std::cout << "=== Initial alignment only - skipping optimization ===" << std::endl;
            return; // Exit if only initial alignment is requested
        }
        
        if (!g_initialAlignmentComplete) {
            std::cout << "❌ Could not create initial alignment edges - matching not complete." << std::endl;
            std::cout << "    → Ensure both detected planes and BIM walls are available." << std::endl;
            return; // Exit if matching is not complete
        }

        // BIM Walls vertices (Global Optimization) - 
        for (const auto &vpBIMWall : vpBIMWalls)
        {
            // Adding a vertex for each BIM wall
            g2o::VertexPlane *vBIMPlane = new g2o::VertexPlane();
            int opIdG = maxOpId + nBIMWalls; // Continue numbering after regular planes
            vBIMPlane->setId(opIdG);
            vBIMPlane->setEstimate(vpBIMWall->getGlobalEquation());
            optimizer.addVertex(vBIMPlane);
            
            vBIMPlane->setFixed(true); // BIM walls are fixed (from external data)
            nBIMWalls++;

            // Setting the global optimization ID for the BIM wall
            vpBIMWall->setOpIdG(opIdG);
            // pritnt the BIM wall ID and OpIdG
        }


        maxOpId += nBIMWalls;

        const bool force_walls = true;

        // CREATE EDGES PLANE-TO-BIM
        if (g_initialAlignmentComplete && !vpBIMWalls.empty()) {

            // Define plane equation and centroid distance threshold
            double centroidDistanceThreshold = 6; // Adjust as needed
            double planeEquationThreshold = 0.75; // Adjust as needed
            
            // Perform data association
            // PerformBIMDetectedDataAssociation(vpBIMWalls, allPlanesVec, planeEquationThreshold, centroidDistanceThreshold); // threshold can be adjusted
            // [TIMING][MATCH] measure cost of continuous BIM wall matching
            size_t nAssocBefore = g_associations.size();
            auto tMatchStart = std::chrono::steady_clock::now();
            PerformBIMDetectedMultiAssociation(vpBIMWalls, allPlanesVec, planeEquationThreshold, centroidDistanceThreshold); // threshold can be adjusted
            auto tMatchEnd = std::chrono::steady_clock::now();
            double matchMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(tMatchEnd - tMatchStart).count();
            // signed delta: matching can shrink g_associations when invalid entries are pruned
            long long nAssocDelta = (long long)g_associations.size() - (long long)nAssocBefore;
            std::cout << "[TIMING][MATCH] ms=" << matchMs
                      << " bim_walls=" << vpBIMWalls.size()
                      << " detected_planes=" << allPlanesVec.size()
                      << " assoc_delta=" << nAssocDelta
                      << " total_assoc=" << g_associations.size()
                      << std::endl;
            for (size_t iA = nAssocBefore; iA < g_associations.size(); ++iA) {
                const auto& a = g_associations[iA];
                int bimId = (a.first ? a.first->getBIMId() : -1);
                int detId = (a.second ? a.second->getId() : -1);
                std::cout << "[TIMING][MATCH][NEW_ASSOC] idx=" << iA
                          << " bim_id=" << bimId
                          << " detected_id=" << detId
                          << std::endl;
            }
            
    
            
            // Create edges for all associations
            
            int edges_created = 0;
            
            for (const auto& association : g_associations) {
                Plane* bimWall = association.first;
                Plane* detectedPlane = association.second;
                
                if (!bimWall || !detectedPlane) {
                    std::cout << "  ⚠️  Null pointer found in association - skipping" << std::endl;
                    continue;
                }
                
                // Verify both vertices exist in the optimizer
                if (!optimizer.vertex(bimWall->getOpIdG()) || !optimizer.vertex(detectedPlane->getOpIdG())) {
                    std::cout << "  ⚠️  Could not find optimizer vertices for BIM Wall #" << bimWall->getBIMId() 
                            << " (OpIdG: " << bimWall->getOpIdG() << ") or Detected Plane #" << detectedPlane->getId() 
                            << " (OpIdG: " << detectedPlane->getOpIdG() << ")" << std::endl;
                    continue;
                }
                
                
                // Create Edge2Planes constraint
                if (force_walls){
                    auto detected_wall_vertex = dynamic_cast<g2o::VertexPlane*>(optimizer.vertex(detectedPlane->getOpIdG()));
                    auto bim_wall_vertex = dynamic_cast<g2o::VertexPlane*>(optimizer.vertex(bimWall->getOpIdG())); 

                    if (!detected_wall_vertex || !bim_wall_vertex) {
                        std::cout << "Could not find detected wall vertex or BIM wall vertex - skipping" << std::endl;
                    continue;} // Skip if detected wall vertex is not found
                    detected_wall_vertex->setFixed(true); // Force detected walls to be fixed
                    detected_wall_vertex->setEstimate(bimWall->getGlobalEquation().coeffs()); // Set estimate to BIM wall equation

                  // continue; // Skip if force_walls is true
                } 
                Edge2Planes *edge = new Edge2Planes();
                
                // BIM wall is fixed (vertex 0), detected plane is optimizable (vertex 1)
                edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(bimWall->getOpIdG())));
                edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(detectedPlane->getOpIdG())));
                
                // Set measurement (target difference should be zero for perfect alignment)
                Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
                edge->setMeasurement(measurement);
                
                // Calculate information weight based on association quality
                g2o::Plane3D bim_eq = bimWall->getGlobalEquation();
                g2o::Plane3D det_eq = detectedPlane->getGlobalEquation();
                Eigen::Vector3d error = bim_eq.ominus(det_eq);
                Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
                double plane_distance = sqrt(error.transpose() * cov * error);
                
                // Calculate centroid perpendicular distance
                Eigen::Vector3f bim_centroid = bimWall->getCentroid();
                Eigen::Vector3f detected_centroid = detectedPlane->getCentroid();
                Eigen::Vector3d centroid_diff = (bim_centroid - detected_centroid).cast<double>();
                Eigen::Vector3d detected_normal = detectedPlane->getGlobalEquation().normal().normalized();
                double d_parallel = std::abs(centroid_diff.dot(detected_normal));
                double d_total = centroid_diff.norm();
                double d_perp = std::sqrt(d_total * d_total - d_parallel * d_parallel);
                
                // Combined score (same as in association function)
                double plane_weight = 0.7;
                double centroid_weight = 0.3;
                double combined_score = plane_weight * plane_distance + centroid_weight * d_perp;
                
                std::cout << "  → Edge: BIM Wall #" << bimWall->getBIMId() 
                        << " (OpIdG: " << bimWall->getOpIdG() << ") ↔ Detected Plane #" << detectedPlane->getId()
                        << " (OpIdG: " << detectedPlane->getOpIdG() << ")" << std::endl;
                std::cout << "    Combined score: " << combined_score << std::endl;

                // Covariance grows with the matching score (Eq. 7): well-matched pairs (low
                // score) get a small covariance, and therefore a strong constraint once inverted
                // into an information matrix; poorly-matched pairs (high score) get a large
                // covariance and are down-weighted. The score is floored to avoid a singular
                // covariance as it approaches 0 (near-perfect match).
                const double kMinMatchingScore = 1e-3;
                double beta = 1e-3;
                double safe_score = std::max(combined_score, kMinMatchingScore);
                Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * beta * safe_score;
                edge->setInformation(covariance.inverse());

                // Add robust kernel to handle potential outliers
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                edge->setRobustKernel(rk);
                rk->setDelta(thHuber3D);
                
                optimizer.addEdge(edge);
                edges_created++;
            }
            
        }

        // // Print plane equations BEFORE optimization
        // if (detectedPlane1) {
        //     g2o::Plane3D eq1 = detectedPlane1->getGlobalEquation();
        // }
        // if (detectedPlane2) {
        //     g2o::Plane3D eq2 = detectedPlane2->getGlobalEquation();
        // }
        // if (bimWall1) {
        //     g2o::Plane3D bim1 = bimWall1->getGlobalEquation();
        // }
        // if (bimWall3) {
        //     g2o::Plane3D bim3 = bimWall3->getGlobalEquation();
        // }
        
        
        optimizer.setVerbose(true);

        // [TIMING][OPT] measure pure solver time of the BIM-aware bundle adjustment
        // Wraps only initializeOptimization() + optimize(N). Excludes graph construction,
        // atlas marshaling, and state recovery, which are base-SLAM costs.
        size_t nG2oVerts = optimizer.vertices().size();
        size_t nG2oEdges = optimizer.edges().size();
        auto tOptStart = std::chrono::steady_clock::now();
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);
        auto tOptEnd = std::chrono::steady_clock::now();
        double optMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(tOptEnd - tOptStart).count();
        std::cout << "[TIMING][OPT] ms=" << optMs
                  << " kfs=" << vpKFs.size()
                  << " mps=" << vpMP.size()
                  << " planes=" << allPlanesVec.size()
                  << " bim_walls=" << vpBIMWalls.size()
                  << " active_assoc=" << g_associations.size()
                  << " g2o_verts=" << nG2oVerts
                  << " g2o_edges=" << nG2oEdges
                  << std::endl;

        Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

        auto visualization_markers = vs_graphs_visualization::visualizeLocalBAGraph(
            &optimizer,        // g2o::SparseOptimizer*
            vpBIMWalls,        // const std::vector<ORB_SLAM3::Plane*>& bimWalls
            allPlanesVec,      // const std::vector<ORB_SLAM3::Plane*>& detectedPlanes
            g_associations,    // const std::vector<std::pair<ORB_SLAM3::Plane*, ORB_SLAM3::Plane*>>& associations
            "world",           // const std::string& frame_id
            6000);             // int initial_id   // Use different ID range to avoid conflicts

        // Recover optimized data (Global Optimization)
        // Globally Optimized Keyframes
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));

            g2o::SE3Quat SE3quat = vSE3->estimate();
            // if (nLoopKF == pMap->GetOriginKF()->mnId)
            // FOR NOW I WANT TO SAVE ALL THE POSES, EVEN IF THEY ARE NOT THE ORIGIN KF
            if (true)
            {
                pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>()));
                // Option 1: Print quaternion coefficients (w, x, y, z)

            }
            else // compare the optimized pose with the one from the loop closure, if more than 1m...
            {
                pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
                pKF->mnBAGlobalForKF = nLoopKF;

                Sophus::SE3f mTwc = pKF->GetPoseInverse();
                Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
                Eigen::Vector3f vector_dist = mTcGBA_c.translation();
                double dist = vector_dist.norm();
                if (dist > 1)
                {
                    int numMonoBadPoints = 0, numMonoOptPoints = 0;
                    int numStereoBadPoints = 0, numStereoOptPoints = 0;
                    vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;

                    for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++)
                    {
                        ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i2];
                        MapPoint *pMP = vpMapPointEdgeMono[i2];
                        KeyFrame *pKFedge = vpEdgeKFMono[i2];

                        if (pKF != pKFedge)
                        {
                            continue;
                        }

                        if (pMP->isBad())
                            continue;

                        if (e->chi2() > 5.991 || !e->isDepthPositive())
                        {
                            numMonoBadPoints++;
                        }
                        else
                        {
                            numMonoOptPoints++;
                            vpMonoMPsOpt.push_back(pMP);
                        }
                    }

                    for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend; i2++)
                    {
                        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i2];
                        MapPoint *pMP = vpMapPointEdgeStereo[i2];
                        KeyFrame *pKFedge = vpEdgeKFMono[i2];

                        if (pKF != pKFedge)
                        {
                            continue;
                        }

                        if (pMP->isBad())
                            continue;

                        if (e->chi2() > 7.815 || !e->isDepthPositive())
                        {
                            numStereoBadPoints++;
                        }
                        else
                        {
                            numStereoOptPoints++;
                            vpStereoMPsOpt.push_back(pMP);
                        }
                    }
                }
            }
        }
        

        // Globally Optimized MapPoints
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            if (vbNotIncludedMP[i])
                continue;

            MapPoint *pMP = vpMP[i];

            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

            if (nLoopKF == pMap->GetOriginKF()->mnId)
            {
                pMP->SetWorldPos(vPoint->estimate().cast<float>());
                pMP->UpdateNormalAndDepth();
            }
            else
            {
                pMP->mPosGBA = vPoint->estimate().cast<float>();
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }

        // Globally Optimized Markers
        for (auto &vpMarker : allMarkersVec)
        {
            g2o::VertexSE3Expmap *vMarker = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(vpMarker->getOpIdG()));
            g2o::SE3Quat SE3quat = vMarker->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
            vpMarker->setGlobalPose(Tiw);
        }

        // Globally Optimized Planes
        for (auto &vpPlane : allPlanesVec)
        {
            if (optimizer.vertex(vpPlane->getOpIdG()))
            {
                g2o::VertexPlane *vPlane = static_cast<g2o::VertexPlane *>(optimizer.vertex(vpPlane->getOpIdG()));
               
                // if (nLoopKF == pMap->GetOriginKF()->mnId)
                if (true) // MODIFY
                {
                    vpPlane->setGlobalEquation(vPlane->estimate());
                }


            }
        }

        // [TODO] - Add recovery of optimized data for Doors and Rooms
    }


    void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag,
                                           const unsigned long nLoopKF, const bool bRobust,
                                           double markerImpact)
    {
        std::vector<ORB_SLAM3::Door *> allDoors = pMap->GetAllDoors();
        std::vector<ORB_SLAM3::Room *> allRooms = pMap->GetAllRooms();
        std::vector<ORB_SLAM3::Plane *> allPlanes = pMap->GetAllPlanes();
        std::vector<ORB_SLAM3::Marker *> allMarkers = pMap->GetAllMarkers();
        std::vector<ORB_SLAM3::MapPoint *> allMapPoints = pMap->GetAllMapPoints();
        std::vector<ORB_SLAM3::KeyFrame *> allKeyFrames = pMap->GetAllKeyFrames();

        BundleAdjustment(allKeyFrames, allMapPoints, allMarkers, allPlanes, allDoors,
                         allRooms, nIterations, pbStopFlag, nLoopKF, bRobust, markerImpact);
    }

    void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                     const vector<Marker *> &allMarkersVec, const vector<Plane *> &allPlanesVec,
                                     const vector<Door *> &allDoorsVec, const vector<Room *> &vpRooms, int nIterations,
                                     bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust, double markerImpact)
    {
        std::cout << "Enter in BundleAdjustment" << std::endl;
        SystemParams *sysParams = SystemParams::GetParams();
        vector<bool> vbNotIncludedMP;
        vbNotIncludedMP.resize(vpMP.size());

        Map *pMap = vpKFs[0]->GetMap();

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        const int nExpectedSize = (vpKFs.size()) * vpMP.size();

        vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
        vpEdgesBody.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFBody;
        vpEdgeKFBody.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeBody;
        vpMapPointEdgeBody.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);
////////////////////////////////////////////////////////////////////////
           // ADD A-GRAPH HERE
        
//////////////////////////////////////////////////////////////////////////
        std::cout << "Start vertices" << std::endl;
        // Set KeyFrame vertices (Global Optimization)
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKF->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKF->mnId);
            vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
            optimizer.addVertex(vSE3);
            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }

        const float thHuber1D = sqrt(3.841);
        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);

        int nPlanes = 1;
        int nDoors = 1;
        int nRooms = 1;
        int maxOpId = 0;
        int nMarkers = 1;

        // Set MapPoint vertices (Global Optimization)
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            MapPoint *pMP = vpMP[i];
            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            const int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            // Update the maxOpId to hold the biggest value
            if (id > maxOpId)
                maxOpId = id;

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            int nEdges = 0;
            // SET EDGES
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
            {
                KeyFrame *pKF = mit->first;
                if (pKF->isBad() || pKF->mnId > maxKFid)
                    continue;
                if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
                    continue;
                nEdges++;

                const int leftIndex = get<0>(mit->second);

                if (leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)] < 0)
                {
                    const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    e->pCamera = pKF->mpCamera;

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKF);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else if (leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0) // Stereo observation
                {
                    const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
                    e->bf = pKF->mbf;

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKF);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                if (pKF->mpCamera2)
                {
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1 && rightIndex < pKF->mvKeysRight.size())
                    {
                        rightIndex -= pKF->NLeft;

                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);

                        Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
                        e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                        e->pCamera = pKF->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKF);
                        vpMapPointEdgeBody.push_back(pMP);
                    }
                }
            }

            if (nEdges == 0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            }
            else
            {
                vbNotIncludedMP[i] = false;
            }
        }

        // Markers (Global Optimization)
        for (const auto &vpMarker : allMarkersVec)
        {
            // Adding a vertex for each marker
            g2o::VertexSE3Expmap *vMarker = new g2o::VertexSE3Expmap();
            vMarker->setEstimate(g2o::SE3Quat(vpMarker->getGlobalPose().unit_quaternion().cast<double>(),
                                              vpMarker->getGlobalPose().translation().cast<double>()));
            int opIdG = maxOpId + nMarkers;
            vMarker->setId(opIdG);
            optimizer.addVertex(vMarker);
            nMarkers++;

            // Setting the Global Optimization ID for the marker
            vpMarker->setOpIdG(opIdG);

            /**
             * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex (SE3)
             * 🚧 [vS-Graphs v.2.0] This edge is not used anymore, in contrast to the previous version.
             * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll, pitch, yaw)
             */
        }

        maxOpId += nMarkers;

        // Planes (Global Optimization)
        for (const auto &vpPlane : allPlanesVec)
        {
            // Skip undefined planes (if not wall for now)
            if (vpPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                continue;
            // Adding a vertex for each plane
            g2o::VertexPlane *vPlane = new g2o::VertexPlane();
            int opIdG = maxOpId + nPlanes;
            vPlane->setId(opIdG);
            vPlane->setEstimate(vpPlane->getGlobalEquation()); //PASS G2O::plane3D type a-graph variable to this function
            if (sysParams->optimization.marginalize_planes)
                vPlane->setMarginalized(true);
            optimizer.addVertex(vPlane);
            nPlanes++;

            // Setting the global optimization ID for the plane
            vpPlane->setOpIdG(opIdG);

            // Adding an edge between the plane and the keyframes
            const map<KeyFrame *, ORB_SLAM3::Plane::Observation> observations = vpPlane->getObservations();
            for (map<KeyFrame *, ORB_SLAM3::Plane::Observation>::const_iterator obsId = observations.begin(), obLast = observations.end(); obsId != obLast; obsId++)
            {
                KeyFrame *pKFi = obsId->first;
                ORB_SLAM3::Plane::Observation obs = obsId->second;

                if (pKFi->isBad())
                {
                    std::cout << "KF is bad" << std::endl;
                    vpPlane->eraseObservation(pKFi);
                    continue;
                }

                g2o::Plane3D planeLocalEquation = obs.localPlane;
                ORB_SLAM3::EdgeVertexPlaneProjectSE3KF *e = new ORB_SLAM3::EdgeVertexPlaneProjectSE3KF();

                if (optimizer.vertex(opIdG) && optimizer.vertex(pKFi->mnId))
                {
                    if (sysParams->optimization.plane_kf.enabled)
                    {
                        ORB_SLAM3::EdgeVertexPlaneProjectSE3KF *e = new ORB_SLAM3::EdgeVertexPlaneProjectSE3KF();
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                        e->setInformation(Eigen::Matrix<double, 3, 3>::Identity() * obs.confidence * sysParams->optimization.plane_kf.information_gain);
                        e->setMeasurement(planeLocalEquation);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                        optimizer.addEdge(e);
                    }

                    // adding plane-point constraints
                    if (sysParams->optimization.plane_point.enabled)
                    {
                        // get the class index of the plane
                        int clsCloudIdx = Utils::getClassIdFromPlaneType(vpPlane->getPlaneType());
                        if (clsCloudIdx != -1)
                        {
                            // add the plane-point constraint
                            ORB_SLAM3::EdgeSE3KFPointToPlane *e = new ORB_SLAM3::EdgeSE3KFPointToPlane();
                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * obs.confidence * sysParams->optimization.plane_point.information_gain);
                            e->setMeasurement(obs.Gij);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuber1D);
                            optimizer.addEdge(e);
                        }
                    }
                }
            }

            // 🚧 [vS-Graphs v.2.0] in contrast with the first version of visual S-Graphs, where there was an edge between
            // Markers and Planes, in this version we removed that edge
            // vector<Marker *> attachedMarkers = vpPlane->getMarkers();
            // for (const auto &planeMarker : attachedMarkers)
            // {
            //     // Adding an edge between the Plane and the Marker
            //     ORB_SLAM3::EdgeVertexPlaneProjectSE3M *e = new ORB_SLAM3::EdgeVertexPlaneProjectSE3M();
            //     e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
            //     e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(planeMarker->getOpIdG())));
            //     e->setInformation(Eigen::Matrix<double, 4, 4>::Identity());

            //     g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            //     e->setRobustKernel(rk);
            //     rk->setDelta(thHuber2D);
            //     optimizer.addEdge(e);
            // }
        }

        maxOpId += nPlanes;

        // Set Room vertices (Global Optimization)
        for (const auto &vpRoom : vpRooms)
        {
            // Adding a vertex for each room
            g2o::VertexSE3Expmap *vrtxRoom = new g2o::VertexSE3Expmap();

            int opIdG = maxOpId + nRooms;
            vrtxRoom->setId(opIdG);
            vrtxRoom->setEstimate(g2o::SE3Quat(Eigen::Quaterniond::Identity(),
                                               vpRoom->getRoomCenter().cast<double>()));
            optimizer.addVertex(vrtxRoom);
            nRooms++;

            // Setting the global optimization ID for the room
            vpRoom->setOpIdG(opIdG);

            // Get list of walls of the room
            vector<Plane *> walls = vpRoom->getWalls();
            if (walls.size() > 0)
                if (vpRoom->getIsCorridor())
                {
                    // Adding an edge between the room and the two walls
                    ORB_SLAM3::EdgeVertex2PlaneProjectSE3Room *e = new ORB_SLAM3::EdgeVertex2PlaneProjectSE3Room();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[0]->getOpIdG())));
                    e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[1]->getOpIdG())));
                    e->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                    optimizer.addEdge(e);
                }
                else
                {
                    // Adding an edge between the room and the two walls
                    ORB_SLAM3::EdgeVertex4PlaneProjectSE3Room *e = new ORB_SLAM3::EdgeVertex4PlaneProjectSE3Room();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[0]->getOpIdG())));
                    e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[1]->getOpIdG())));
                    e->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[2]->getOpIdG())));
                    e->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[3]->getOpIdG())));
                    e->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                    optimizer.addEdge(e);
                }
        }

        maxOpId += nRooms;

        // Set Door vertices (Global Optimization)
        for (const auto &vpRoom : vpRooms)
        {
            vector<Door *> doors = vpRoom->getDoors();
            for (const auto &door : doors)
            {
                // Adding a vertex for each door
                g2o::VertexSE3Expmap *vDoor = new g2o::VertexSE3Expmap();
                int opIdG = maxOpId + nDoors;
                vDoor->setId(opIdG);
                vDoor->setEstimate(g2o::SE3Quat(door->getGlobalPose().unit_quaternion().cast<double>(),
                                                door->getGlobalPose().translation().cast<double>()));
                optimizer.addVertex(vDoor);
                nDoors++;

                // Setting the local optimization ID for the door
                door->setOpIdG(opIdG);

                // Adding an edge between the room and the door
                ORB_SLAM3::EdgeSE3DoorProjectSE3Room *e = new ORB_SLAM3::EdgeSE3DoorProjectSE3Room();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opIdG)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(door->getOpIdG())));
                e->setInformation(Eigen::MatrixXd::Identity(6, 6));

                Eigen::Isometry3d relativePose = Eigen::Isometry3d::Identity();
                relativePose.matrix() = (dynamic_cast<g2o::VertexSE3Expmap *>((optimizer.vertex(vpRoom->getOpIdG())))->estimate().inverse() *
                                         dynamic_cast<g2o::VertexSE3Expmap *>((optimizer.vertex(door->getOpIdG())))->estimate())
                                            .to_homogeneous_matrix();
                e->setMeasurement(relativePose);

                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);
                optimizer.addEdge(e);
            }
        }

        maxOpId += nDoors;

        std::cout << "=== End Analysis ===" << std::endl;

        // Optimize!
        optimizer.setVerbose(true);
        // save the graph here:
        // optimizer.save("before_gba.g2o");
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);
        Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

        // Recover optimized data (Global Optimization)
        // Globally Optimized Keyframes
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));

            g2o::SE3Quat SE3quat = vSE3->estimate();
            if (nLoopKF == pMap->GetOriginKF()->mnId)
            {
                pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>()));
            }
            else
            {
                pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
                pKF->mnBAGlobalForKF = nLoopKF;

                Sophus::SE3f mTwc = pKF->GetPoseInverse();
                Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
                Eigen::Vector3f vector_dist = mTcGBA_c.translation();
                double dist = vector_dist.norm();
                if (dist > 1)
                {
                    int numMonoBadPoints = 0, numMonoOptPoints = 0;
                    int numStereoBadPoints = 0, numStereoOptPoints = 0;
                    vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;

                    for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++)
                    {
                        ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i2];
                        MapPoint *pMP = vpMapPointEdgeMono[i2];
                        KeyFrame *pKFedge = vpEdgeKFMono[i2];

                        if (pKF != pKFedge)
                        {
                            continue;
                        }

                        if (pMP->isBad())
                            continue;

                        if (e->chi2() > 5.991 || !e->isDepthPositive())
                        {
                            numMonoBadPoints++;
                        }
                        else
                        {
                            numMonoOptPoints++;
                            vpMonoMPsOpt.push_back(pMP);
                        }
                    }

                    for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend; i2++)
                    {
                        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i2];
                        MapPoint *pMP = vpMapPointEdgeStereo[i2];
                        KeyFrame *pKFedge = vpEdgeKFMono[i2];

                        if (pKF != pKFedge)
                        {
                            continue;
                        }

                        if (pMP->isBad())
                            continue;

                        if (e->chi2() > 7.815 || !e->isDepthPositive())
                        {
                            numStereoBadPoints++;
                        }
                        else
                        {
                            numStereoOptPoints++;
                            vpStereoMPsOpt.push_back(pMP);
                        }
                    }
                }
            }
        }

        // Globally Optimized MapPoints
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            if (vbNotIncludedMP[i])
                continue;

            MapPoint *pMP = vpMP[i];

            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

            if (nLoopKF == pMap->GetOriginKF()->mnId)
            {
                pMP->SetWorldPos(vPoint->estimate().cast<float>());
                pMP->UpdateNormalAndDepth();
            }
            else
            {
                pMP->mPosGBA = vPoint->estimate().cast<float>();
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }

        // Globally Optimized Markers
        for (auto &vpMarker : allMarkersVec)
        {
            g2o::VertexSE3Expmap *vMarker = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(vpMarker->getOpIdG()));
            g2o::SE3Quat SE3quat = vMarker->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
            vpMarker->setGlobalPose(Tiw);
        }

        // Globally Optimized Planes
        for (auto &vpPlane : allPlanesVec)
        {
            if (optimizer.vertex(vpPlane->getOpIdG()))
            {
                g2o::VertexPlane *vPlane = static_cast<g2o::VertexPlane *>(optimizer.vertex(vpPlane->getOpIdG()));

                if (nLoopKF == pMap->GetOriginKF()->mnId)
                {
                    vpPlane->setGlobalEquation(vPlane->estimate());
                }
                else
                {
                    vpPlane->mPlaneGBA = vPlane->estimate();
                    vpPlane->mnBAGlobalForKF = nLoopKF;
                }
            }
        }

        // [TODO] - Add recovery of optimized data for Doors and Rooms
    }

    void Optimizer::FullInertialBA(Map *pMap, int its, const bool bFixLocal, const long unsigned int nLoopId, bool *pbStopFlag, bool bInit, float priorG, float priorA, Eigen::VectorXd *vSingVal, bool *bHess)
    {
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e-5);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        int nNonFixed = 0;

        // Set KeyFrame vertices
        KeyFrame *pIncKF;
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid)
                continue;
            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            pIncKF = pKFi;
            bool bFixed = false;
            if (bFixLocal)
            {
                bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) || (pKFi->mnBAFixedForKF >= (maxKFid - 1));
                if (!bFixed)
                    nNonFixed++;
                VP->setFixed(bFixed);
            }
            if (i == 0) // Fix the first keyframe at origin
                VP->setFixed(true);
            optimizer.addVertex(VP);

            if (pKFi->bImu)
            {
                VertexVelocity *VV = new VertexVelocity(pKFi);
                VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
                VV->setFixed(bFixed);
                optimizer.addVertex(VV);
                if (!bInit)
                {
                    VertexGyroBias *VG = new VertexGyroBias(pKFi);
                    VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                    VG->setFixed(bFixed);
                    optimizer.addVertex(VG);
                    VertexAccBias *VA = new VertexAccBias(pKFi);
                    VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                    VA->setFixed(bFixed);
                    optimizer.addVertex(VA);
                }
            }
        }

        if (bInit)
        {
            VertexGyroBias *VG = new VertexGyroBias(pIncKF);
            VG->setId(4 * maxKFid + 2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias *VA = new VertexAccBias(pIncKF);
            VA->setId(4 * maxKFid + 3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }

        if (bFixLocal)
        {
            if (nNonFixed < 3)
                return;
        }

        // IMU links
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            if (!pKFi->mPrevKF)
            {
                Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!", Verbose::VERBOSITY_NORMAL);
                continue;
            }

            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid)
            {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                    continue;
                if (pKFi->bImu && pKFi->mPrevKF->bImu)
                {
                    pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                    g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                    g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);

                    g2o::HyperGraph::Vertex *VG1;
                    g2o::HyperGraph::Vertex *VA1;
                    g2o::HyperGraph::Vertex *VG2;
                    g2o::HyperGraph::Vertex *VA2;
                    if (!bInit)
                    {
                        VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
                        VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
                        VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
                        VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
                    }
                    else
                    {
                        VG1 = optimizer.vertex(4 * maxKFid + 2);
                        VA1 = optimizer.vertex(4 * maxKFid + 3);
                    }

                    g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
                    g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);

                    if (!bInit)
                    {
                        if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
                        {
                            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2 << endl;
                            continue;
                        }
                    }
                    else
                    {
                        if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2)
                        {
                            cout << "Error" << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2 << endl;
                            continue;
                        }
                    }

                    EdgeInertial *ei = new EdgeInertial(pKFi->mpImuPreintegrated);
                    ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                    ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                    ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
                    ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
                    ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                    ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

                    g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
                    ei->setRobustKernel(rki);
                    rki->setDelta(sqrt(16.92));

                    optimizer.addEdge(ei);

                    if (!bInit)
                    {
                        EdgeGyroRW *egr = new EdgeGyroRW();
                        egr->setVertex(0, VG1);
                        egr->setVertex(1, VG2);
                        Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
                        egr->setInformation(InfoG);
                        egr->computeError();
                        optimizer.addEdge(egr);

                        EdgeAccRW *ear = new EdgeAccRW();
                        ear->setVertex(0, VA1);
                        ear->setVertex(1, VA2);
                        Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
                        ear->setInformation(InfoA);
                        ear->computeError();
                        optimizer.addEdge(ear);
                    }
                }
                else
                    cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu" << endl;
            }
        }

        if (bInit)
        {
            g2o::HyperGraph::Vertex *VG = optimizer.vertex(4 * maxKFid + 2);
            g2o::HyperGraph::Vertex *VA = optimizer.vertex(4 * maxKFid + 3);

            // Add prior to comon biases
            Eigen::Vector3f bprior;
            bprior.setZero();

            EdgePriorAcc *epa = new EdgePriorAcc(bprior);
            epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
            double infoPriorA = priorA; //
            epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
            optimizer.addEdge(epa);

            EdgePriorGyro *epg = new EdgePriorGyro(bprior);
            epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            double infoPriorG = priorG; //
            epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
            optimizer.addEdge(epg);
        }

        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        const unsigned long iniMPid = maxKFid * 5;

        vector<bool> vbNotIncludedMP(vpMPs.size(), false);

        for (size_t i = 0; i < vpMPs.size(); i++)
        {
            MapPoint *pMP = vpMPs[i];
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            unsigned long id = pMP->mnId + iniMPid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            bool bAllFixed = true;

            // Set edges
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (pKFi->mnId > maxKFid)
                    continue;

                if (!pKFi->isBad())
                {
                    const int leftIndex = get<0>(mit->second);
                    cv::KeyPoint kpUn;

                    if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] < 0) // Monocular observation
                    {
                        kpUn = pKFi->mvKeysUn[leftIndex];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono *e = new EdgeMono(0);

                        g2o::OptimizableGraph::Vertex *VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId));
                        if (bAllFixed)
                            if (!VP->fixed())
                                bAllFixed = false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                    }
                    else if (leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0) // stereo observation
                    {
                        kpUn = pKFi->mvKeysUn[leftIndex];
                        const float kp_ur = pKFi->mvuRight[leftIndex];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        EdgeStereo *e = new EdgeStereo(0);

                        g2o::OptimizableGraph::Vertex *VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId));
                        if (bAllFixed)
                            if (!VP->fixed())
                                bAllFixed = false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        optimizer.addEdge(e);
                    }

                    if (pKFi->mpCamera2)
                    { // Monocular right observation
                        int rightIndex = get<1>(mit->second);

                        if (rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size())
                        {
                            rightIndex -= pKFi->NLeft;

                            Eigen::Matrix<double, 2, 1> obs;
                            kpUn = pKFi->mvKeysRight[rightIndex];
                            obs << kpUn.pt.x, kpUn.pt.y;

                            EdgeMono *e = new EdgeMono(1);

                            g2o::OptimizableGraph::Vertex *VP = dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId));
                            if (bAllFixed)
                                if (!VP->fixed())
                                    bAllFixed = false;

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                            e->setVertex(1, VP);
                            e->setMeasurement(obs);
                            const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberMono);

                            optimizer.addEdge(e);
                        }
                    }
                }
            }

            if (bAllFixed)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            }
        }

        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        optimizer.initializeOptimization();
        optimizer.optimize(its);

        // Recover optimized data
        // Keyframes
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid)
                continue;
            VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
            if (nLoopId == 0)
            {
                Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
                pKFi->SetPose(Tcw);
            }
            else
            {
                pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
                pKFi->mnBAGlobalForKF = nLoopId;
            }
            if (pKFi->bImu)
            {
                VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
                if (nLoopId == 0)
                {
                    pKFi->SetVelocity(VV->estimate().cast<float>());
                }
                else
                {
                    pKFi->mVwbGBA = VV->estimate().cast<float>();
                }

                VertexGyroBias *VG;
                VertexAccBias *VA;
                if (!bInit)
                {
                    VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
                    VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
                }
                else
                {
                    VG = static_cast<VertexGyroBias *>(optimizer.vertex(4 * maxKFid + 2));
                    VA = static_cast<VertexAccBias *>(optimizer.vertex(4 * maxKFid + 3));
                }

                Vector6d vb;
                vb << VG->estimate(), VA->estimate();
                IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
                if (nLoopId == 0)
                {
                    pKFi->SetNewBias(b);
                }
                else
                {
                    pKFi->mBiasGBA = b;
                }
            }
        }

        // Points
        for (size_t i = 0; i < vpMPs.size(); i++)
        {
            if (vbNotIncludedMP[i])
                continue;

            MapPoint *pMP = vpMPs[i];
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + iniMPid + 1));

            if (nLoopId == 0)
            {
                pMP->SetWorldPos(vPoint->estimate().cast<float>());
                pMP->UpdateNormalAndDepth();
            }
            else
            {
                pMP->mPosGBA = vPoint->estimate().cast<float>();
                pMP->mnBAGlobalForKF = nLoopId;
            }
        }

        pMap->IncreaseChangeIndex();
    }

    int Optimizer::PoseOptimization(Frame *pFrame)
    {
        SystemParams *sysParams = SystemParams::GetParams();

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // Set Frame vertex
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pFrame->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // Set MapPoint vertices
        const int N = pFrame->N;

        vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
        vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
        vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
        vpEdgesMono.reserve(N);
        vpEdgesMono_FHR.reserve(N);
        vnIndexEdgeMono.reserve(N);
        vnIndexEdgeRight.reserve(N);

        vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesStereo.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const float deltaMono = sqrt(5.991);
        const float deltaStereo = sqrt(7.815);

        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP)
                {
                    // Conventional SLAM
                    if (!pFrame->mpCamera2)
                    {
                        // Monocular observation
                        if (pFrame->mvuRight[i] < 0)
                        {
                            nInitialCorrespondences++;
                            pFrame->mvbOutlier[i] = false;

                            Eigen::Matrix<double, 2, 1> obs;
                            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                            obs << kpUn.pt.x, kpUn.pt.y;

                            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                            e->setMeasurement(obs);
                            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(deltaMono);

                            e->pCamera = pFrame->mpCamera;
                            e->Xw = pMP->GetWorldPos().cast<double>();

                            optimizer.addEdge(e);

                            vpEdgesMono.push_back(e);
                            vnIndexEdgeMono.push_back(i);
                        }
                        else // Stereo observation
                        {
                            nInitialCorrespondences++;
                            pFrame->mvbOutlier[i] = false;

                            Eigen::Matrix<double, 3, 1> obs;
                            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                            const float &kp_ur = pFrame->mvuRight[i];
                            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                            g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                            e->setMeasurement(obs);
                            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                            e->setInformation(Info);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(deltaStereo);

                            e->fx = pFrame->fx;
                            e->fy = pFrame->fy;
                            e->cx = pFrame->cx;
                            e->cy = pFrame->cy;
                            e->bf = pFrame->mbf;
                            e->Xw = pMP->GetWorldPos().cast<double>();

                            optimizer.addEdge(e);

                            vpEdgesStereo.push_back(e);
                            vnIndexEdgeStereo.push_back(i);
                        }
                    }
                    // SLAM with respect a rigid body
                    else
                    {
                        nInitialCorrespondences++;

                        cv::KeyPoint kpUn;

                        if (i < pFrame->Nleft)
                        { // Left camera observation
                            kpUn = pFrame->mvKeys[i];

                            pFrame->mvbOutlier[i] = false;

                            Eigen::Matrix<double, 2, 1> obs;
                            obs << kpUn.pt.x, kpUn.pt.y;

                            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                            e->setMeasurement(obs);
                            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(deltaMono);

                            e->pCamera = pFrame->mpCamera;
                            e->Xw = pMP->GetWorldPos().cast<double>();

                            optimizer.addEdge(e);

                            vpEdgesMono.push_back(e);
                            vnIndexEdgeMono.push_back(i);
                        }
                        else
                        {
                            kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

                            Eigen::Matrix<double, 2, 1> obs;
                            obs << kpUn.pt.x, kpUn.pt.y;

                            pFrame->mvbOutlier[i] = false;

                            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                            e->setMeasurement(obs);
                            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(deltaMono);

                            e->pCamera = pFrame->mpCamera2;
                            e->Xw = pMP->GetWorldPos().cast<double>();

                            e->mTrl = g2o::SE3Quat(pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(), pFrame->GetRelativePoseTrl().translation().cast<double>());

                            optimizer.addEdge(e);

                            vpEdgesMono_FHR.push_back(e);
                            vnIndexEdgeRight.push_back(i);
                        }
                    }
                }
            }
        }

        if (nInitialCorrespondences < 3)
            return 0;

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        const int its[4] = {10, 10, 10, 10};

        int nBad = 0;
        for (size_t it = 0; it < 4; it++)
        {
            Tcw = pFrame->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));

            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);

            // before the last step, remove bad map points
            KeyFrame *refKF = pFrame->mpReferenceKF;
            if (sysParams->refine_map_points.enabled && refKF && it == 2)
            {
                vector<Plane *> vpPlanes;
                std::unordered_map<int, bool> planeCheck;

                // populate the vector of planes using the covisibility graph of the reference keyframe
                vector<KeyFrame *> vpRefCovKFs = refKF->GetBestCovisibilityKeyFrames(25);
                vpRefCovKFs.push_back(refKF);
                for (const auto &pKFi : vpRefCovKFs)
                {
                    for (const auto &plane : pKFi->GetMapPlanes())
                    {
                        if (!plane)
                            continue;
                        if (plane->getPlaneType() != Plane::planeVariant::UNDEFINED)
                        {
                            if (planeCheck.find(plane->getId()) == planeCheck.end())
                            {
                                vpPlanes.push_back(plane);
                                planeCheck[plane->getId()] = true;
                            }
                        }
                    }
                }

                g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
                Eigen::Isometry3d framePose = vSE3_recov->estimate();
                Eigen::Vector3d camCenter = framePose.inverse().translation();
                for (const auto &pPlane : vpPlanes)
                {
                    if (pPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                        continue;

                    Eigen::Vector4d planeEq = pPlane->getGlobalEquation().coeffs();

                    // if the camera center is behind the plane, skip the plane
                    if (planeEq.head<3>().dot(camCenter) + planeEq(3) < 0)
                        continue;

                    // for each map point in the frame, check if it is on the plane
                    for (size_t j = 0; j < pFrame->N; j++)
                    {
                        MapPoint *pMP = pFrame->mvpMapPoints[j];
                        if (!pMP || pMP->isBad())
                            continue;

                        // calculate distance from the map point to the plane
                        Eigen::Vector3d pMPw = pMP->GetWorldPos().cast<double>();
                        double distance = planeEq.head<3>().dot(pMPw) + planeEq(3);
                        if (distance < -sysParams->refine_map_points.max_distance_for_delete)
                        {
                            // get the intersection point of the line joining the camera center and the map point with the plane
                            Eigen::Vector3d intersect = Utils::lineIntersectsPlane(planeEq, camCenter, pMPw);

                            // check if the map point is in the plane cloud
                            if (pPlane->isPointinPlaneCloud(intersect))
                            {
                                pFrame->mvpMapPoints[j]->SetBadFlag();
                                pFrame->mvpMapPoints[j] = static_cast<MapPoint *>(NULL);
                                pFrame->mvbOutlier[j] = true;
                            }
                        }
                    }
                }
            }

            nBad = 0;
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];
                if (it == 2)
                    e->setRobustKernel(0);

                if (pFrame->mvbOutlier[idx])
                {
                    if (!pFrame->mvpMapPoints[idx])
                    {
                        optimizer.removeEdge(e);
                        nBad++;
                        continue;
                    }
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Mono[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                }
            }

            for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++)
            {
                ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *e = vpEdgesMono_FHR[i];

                const size_t idx = vnIndexEdgeRight[i];
                if (it == 2)
                    e->setRobustKernel(0);

                if (pFrame->mvbOutlier[idx])
                {
                    if (!pFrame->mvpMapPoints[idx])
                    {
                        optimizer.removeEdge(e);
                        nBad++;
                        continue;
                    }
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Mono[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                }
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];
                if (it == 2)
                    e->setRobustKernel(0);

                if (pFrame->mvbOutlier[idx])
                {
                    if (!pFrame->mvpMapPoints[idx])
                    {
                        optimizer.removeEdge(e);
                        nBad++;
                        continue;
                    }
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    e->setLevel(0);
                    pFrame->mvbOutlier[idx] = false;
                }
            }

            if (optimizer.edges().size() < 10)
                break;
        }

        // Recover optimized pose and return number of inliers
        g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                                SE3quat_recov.translation().cast<float>());
        pFrame->SetPose(pose);

        return nInitialCorrespondences - nBad;
    }

    void Optimizer::LocalBundleAdjustment(Atlas *pAtlas, ORB_SLAM3::KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &countFixedKF,
                                          int &num_OptKF, int &num_MPs, int &num_edges, double markerImpact)
    {
        // System parameters
        SystemParams *sysParams = SystemParams::GetParams();

        // Variables
        countFixedKF = 0;
        std::list<ORB_SLAM3::Door *> localDoorList;
        std::list<ORB_SLAM3::Room *> localRoomList;
        ORB_SLAM3::Map *pCurrentMap = pKF->GetMap();
        std::list<ORB_SLAM3::Plane *> localPlaneList;
        std::list<ORB_SLAM3::Marker *> localMarkerList;
        std::list<ORB_SLAM3::KeyFrame *> localKeyFrameList;
        std::list<ORB_SLAM3::MapPoint *> localMapPointList;
        std::vector<ORB_SLAM3::KeyFrame *> neighborKeyFrameVec;
        std::vector<ORB_SLAM3::Room *> allRooms = pCurrentMap->GetAllRooms();

        // Unorderd maps to keep track of the local entities
        std::unordered_map<int, bool> mpLocalDoorId;
        std::unordered_map<int, bool> mpLocalPlaneId;
        std::unordered_map<int, bool> mpLocalMarkerId;
        std::unordered_map<int, bool> mpLocalMapPointId;
        std::unordered_map<int, bool> mpLocalKeyFrameId;

        std::vector<Plane*> vpBIMWalls = pAtlas->GetBIMDatabase().GetWallsBIM();

        // [LBA] Initialize the KeyFrame-related variables
        localKeyFrameList.push_back(pKF);
        mpLocalKeyFrameId[pKF->mnId] = true;
        pKF->mnBALocalForKF = pKF->mnId;
        
        // [LBA] Fill in the neighbor KeyFrames
        if (sysParams->plane_based_covisibility.enabled)
            // Get the KeyFrames that see the same planes
            neighborKeyFrameVec = pKF->GetBestCovisibilityKeyFrames(sysParams->plane_based_covisibility.max_keyframes);
        else
            // Get the KeyFrames that see the same MapPoints
            neighborKeyFrameVec = pKF->GetVectorCovisibleKeyFrames();

        // Iterate through all neighboring KeyFrames
        for (int idx = 0, idxEnd = neighborKeyFrameVec.size(); idx < idxEnd; idx++)
        {
            // Get the current KeyFrame's neighbors
            ORB_SLAM3::KeyFrame *pKFi = neighborKeyFrameVec[idx];
            // Mark the KeyFrame as a part of the current LBA
            pKFi->mnBALocalForKF = pKF->mnId;
            // If the KeyFrame is proper, add it to the list of local KeyFrames for LBA
            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                localKeyFrameList.push_back(pKFi);
                mpLocalKeyFrameId[pKFi->mnId] = true;
            }
        }

        // [LBA] Loop through the local KeyFrames
        for (std::list<ORB_SLAM3::KeyFrame *>::iterator lit = localKeyFrameList.begin(),
                                                        lend = localKeyFrameList.end();
             lit != lend; lit++)
        {
            // Variables
            ORB_SLAM3::KeyFrame *pKFi = *lit;
            std::vector<ORB_SLAM3::Door *> localDoorsVec = pKFi->GetMapDoors();
            std::vector<ORB_SLAM3::Plane *> localPlanesVec = pKFi->GetMapPlanes();
            std::vector<ORB_SLAM3::Marker *> localMarkersVec = pKFi->GetMapMarkers();
            std::vector<ORB_SLAM3::MapPoint *> localMapPointsVec = pKFi->GetMapPointMatches();

            // If the KeyFrame is the initial KeyFrame of the map, mark that as a fixed KeyFrame
            if (pKFi->mnId == pMap->GetInitKFid())
                countFixedKF = 1;

            // [LBA] Loop through all the MapPoints and prepare them for LBA
            for (std::vector<ORB_SLAM3::MapPoint *>::iterator vit = localMapPointsVec.begin(), vend = localMapPointsVec.end();
                 vit != vend; vit++)
            {
                // Variables
                ORB_SLAM3::MapPoint *pMP = *vit;

                // If the MapPoint is proper, add it to the list of local MapPoints for LBA
                if (pMP)
                    if (!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                    {
                        if (pMP->mnBALocalForKF != pKF->mnId)
                        {
                            localMapPointList.push_back(pMP);
                            mpLocalMapPointId[pMP->mnId] = true;
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
                    }
            }

            // [LBA] Loop through all the Markers and prepare them for LBA
            for (std::vector<ORB_SLAM3::Marker *>::iterator idx = localMarkersVec.begin(), vend = localMarkersVec.end();
                 idx != vend; idx++)
            {
                if (mpLocalMarkerId.find((*idx)->getId()) == mpLocalMarkerId.end())
                {
                    ORB_SLAM3::Marker *marker = *idx;
                    localMarkerList.push_back(marker);
                    mpLocalMarkerId[marker->getId()] = true;
                }
            }

            // [LBA] Loop through all the Planes and prepare them for LBA
            for (std::vector<ORB_SLAM3::Plane *>::iterator idx = localPlanesVec.begin(), vend = localPlanesVec.end();
                 idx != vend; idx++)
            {
                ORB_SLAM3::Plane *plane = *idx;
                // If the plane does not exist, skip it
                if (!plane)
                    continue;
                // If the plane is not known, do not add it to the local map
                if (plane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                    continue;
                // Otherwise, add the plane to the local map
                if (mpLocalPlaneId.find(plane->getId()) == mpLocalPlaneId.end())
                {
                    localPlaneList.push_back(plane);
                    mpLocalPlaneId[plane->getId()] = true;
                }
            }

            // [LBA] Loop through all the Doors and prepare them for LBA
            for (std::vector<ORB_SLAM3::Door *>::iterator idx = localDoorsVec.begin(), vend = localDoorsVec.end();
                 idx != vend; idx++)
            {
                if (mpLocalDoorId.find((*idx)->getId()) == mpLocalDoorId.end())
                {
                    ORB_SLAM3::Door *door = *idx;
                    localDoorList.push_back(door);
                    mpLocalDoorId[door->getId()] = true;
                }
            }
        }

        // [LBA] Among all rooms, filter only the ones with a wall in LBA
        for (const auto &room : allRooms)
        {
            // Get the walls of the room
            std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();
            // Add the room to the local map if any of the walls are in the local map
            for (const auto &wall : roomWalls)
                if (mpLocalPlaneId.find(wall->getId()) != mpLocalPlaneId.end())
                {
                    localRoomList.push_back(room);
                    break;
                }
        }

        // [LBA] Loop through all the local Rooms to add all their walls to LBA
        list<Plane *> lRecentLocalMapPlanes;
        for (list<Room *>::iterator idx = localRoomList.begin(), vend = localRoomList.end(); idx != vend; idx++)
        {
            vector<Plane *> roomWalls = (*idx)->getWalls();
            for (const auto &roomWall : roomWalls)
            {
                if (mpLocalPlaneId.find(roomWall->getId()) == mpLocalPlaneId.end())
                {
                    localPlaneList.push_back(roomWall);
                    mpLocalPlaneId[roomWall->getId()] = true;
                    lRecentLocalMapPlanes.push_back(roomWall);
                }
            }
        }

        // Loop through the recently added planes, get all the map points and add them
        // list<MapPoint *> lRecentLocalMapPoints;
        // for (list<Plane *>::iterator idx = lRecentLocalMapPlanes.begin(), vend = lRecentLocalMapPlanes.end(); idx != vend; idx++)
        // {
        //     set<MapPoint *> mapPoints = (*idx)->getMapPoints();
        //     for (const auto &mapPoint : mapPoints)
        //     {
        //         if (mpLocalMapPointId.find(mapPoint->mnId) == mpLocalMapPointId.end())
        //         {
        //             localMapPointList.push_back(mapPoint);
        //             lRecentLocalMapPoints.push_back(mapPoint);
        //             mpLocalMapPointId[mapPoint->mnId] = true;
        //         }
        //     }
        // }

        // Loop through the recently added planes, get all the keyframes and add them
        std::list<ORB_SLAM3::KeyFrame *> lRecentLocalMapKeyFrames;
        for (list<ORB_SLAM3::Plane *>::iterator idx = lRecentLocalMapPlanes.begin(), vend = lRecentLocalMapPlanes.end(); idx != vend; idx++)
        {
            std::map<KeyFrame *, ORB_SLAM3::Plane::Observation> planeObservations = (*idx)->getObservations();
            for (std::map<KeyFrame *, ORB_SLAM3::Plane::Observation>::const_iterator obsId = planeObservations.begin(), obLast = planeObservations.end(); obsId != obLast; obsId++)
            {
                KeyFrame *pKFi = obsId->first;
                if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                {
                    if (mpLocalKeyFrameId.find(pKFi->mnId) == mpLocalKeyFrameId.end())
                    {
                        localKeyFrameList.push_back(pKFi);
                        mpLocalKeyFrameId[pKFi->mnId] = true;
                        pKFi->mnBALocalForKF = pKF->mnId;
                        lRecentLocalMapKeyFrames.push_back(pKFi);
                    }
                }
            }
        }

        // Loop through the recently added keyframes, get all the map points and add them
        for (list<KeyFrame *>::iterator idx = lRecentLocalMapKeyFrames.begin(), vend = lRecentLocalMapKeyFrames.end(); idx != vend; idx++)
        {
            vector<MapPoint *> vpMPs = (*idx)->GetMapPointMatches();
            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                    {
                        if (pMP->mnBALocalForKF != pKF->mnId)
                        {
                            localMapPointList.push_back(pMP);
                            mpLocalMapPointId[pMP->mnId] = true;
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
                    }
            }
        }

        // Loop through the recently added map points, get all the keyframes and add them
        // for (list<MapPoint *>::iterator idx = lRecentLocalMapPoints.begin(), vend = lRecentLocalMapPoints.end(); idx != vend; idx++)
        // {
        //     std::map<KeyFrame *, std::tuple<int, int>> mapPointObservations = (*idx)->GetObservations();
        //     for (std::map<KeyFrame *, std::tuple<int, int>>::const_iterator obsId = mapPointObservations.begin(), obLast = mapPointObservations.end(); obsId != obLast; obsId++)
        //     {
        //         if (mpLocalKeyFrameId.find(obsId->first->mnId) == mpLocalKeyFrameId.end())
        //         {
        //             localKeyFrameList.push_back(obsId->first);
        //             mpLocalKeyFrameId[obsId->first->mnId] = true;
        //         }
        //     }
        // }

        // Fixed Keyframes for MPs (Keyframes that see Local MapPoints but that are not Local Keyframes)
        list<KeyFrame *> lFixedCameras;
        for (list<MapPoint *>::iterator lit = localMapPointList.begin(), lend = localMapPointList.end(); lit != lend; lit++)
        {
            map<KeyFrame *, tuple<int, int>> observations = (*lit)->GetObservations();
            for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
                {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                        lFixedCameras.push_back(pKFi);
                }
            }
        }

        // const size_t maxNewFixed = 5;
        // // Fixed KeyFrames for Planes (Keyframes that see Local Planes but that are not Local Keyframes)
        // for (list<Plane *>::iterator lit = localPlaneList.begin(), lend = localPlaneList.end(); lit != lend; lit++)
        // {
        //     size_t newFixed = 0;
        //     map<KeyFrame *, ORB_SLAM3::Plane::Observation> observations = (*lit)->getObservations();
        //     for (map<KeyFrame *, ORB_SLAM3::Plane::Observation>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        //     {
        //         KeyFrame *pKFi = mit->first;

        //         if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
        //         {
        //             pKFi->mnBAFixedForKF = pKF->mnId;
        //             if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        //             {
        //                 lFixedCameras.push_back(pKFi);
        //                 newFixed++;
        //             }
        //             if (newFixed >= maxNewFixed)
        //                 break;
        //         }
        //     }
        //     if (newFixed >= maxNewFixed)
        //         break;
        // }

        countFixedKF = lFixedCameras.size() + countFixedKF;

        if (countFixedKF == 0)
        {
            Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
            return;
        }

        // Setup optimizer (Local Optimization)
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        if (pMap->IsInertial())
            solver->setUserLambdaInit(100.0);

        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        unsigned long maxKFid = 0;

        pCurrentMap->msOptKFs.clear();
        pCurrentMap->msFixedKFs.clear();

        // Local KeyFrame vertices (Local Optimization)
        for (list<KeyFrame *>::iterator lit = localKeyFrameList.begin(), lend = localKeyFrameList.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKFi->mnId);
            // if (g_initialAlignmentComplete) // NOT working
            // {
            vSE3->setFixed(false);
                // print that the local graph KF is not fixed
            // }
            // else{
            
                // vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
                // vSE3->setFixed(false);
                // if (pKFi->mnId == pMap->GetInitKFid())
                //     }
                // else{
                // }
            // }
            // vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
            pCurrentMap->msOptKFs.insert(pKFi->mnId);
        }
        num_OptKF = localKeyFrameList.size();

        // Fixed KeyFrame vertices (Local Optimization)
        for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
            pCurrentMap->msFixedKFs.insert(pKFi->mnId);
        }

        // MapPoint vertices (Local Optimization)
        const int nExpectedSize = (localKeyFrameList.size() + lFixedCameras.size()) * localMapPointList.size();

        vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
        vpEdgesBody.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFBody;
        vpEdgeKFBody.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeBody;
        vpMapPointEdgeBody.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const int nExpectedSizePlane = (localKeyFrameList.size() + lFixedCameras.size()) * localPlaneList.size();
        vector<EdgeVertexPlaneProjectSE3KF *> vpEdgesPlane;
        vpEdgesPlane.reserve(nExpectedSizePlane);

        vector<KeyFrame *> vpEdgeKFPlane;
        vpEdgeKFPlane.reserve(nExpectedSizePlane);

        vector<Plane *> vpPlaneEdgePlane;
        vpPlaneEdgePlane.reserve(nExpectedSizePlane);

        vector<EdgeSE3KFPointToPlane *> vpEdgesPlanePoint;
        vpEdgesPlanePoint.reserve(nExpectedSizePlane);

        vector<KeyFrame *> vpEdgeKFPlanePoint;
        vpEdgeKFPlanePoint.reserve(nExpectedSizePlane);

        vector<Plane *> vpPlaneEdgePlanePoint;
        vpPlaneEdgePlanePoint.reserve(nExpectedSizePlane);

        const float thHuber1D = sqrt(3.841);
        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        int nDoors = 1;
        int nRooms = 1;
        int nPlanes = 1;
        int nPoints = 0;
        int nMarkers = 1;
        int nBIMWalls = 1;
        int maxOpId = 0;

        // const float thHuber1D = sqrt(3.841);
        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);


        int nEdges = 0;

        //KeyFrame-MapPoint Connections in Local BA
        for (list<MapPoint *>::iterator lit = localMapPointList.begin(), lend = localMapPointList.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);
            nPoints++;

            // Update the maxOpId to hold the biggest value
            if (id > maxOpId)
                maxOpId = id;

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            // Set edges
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                {
                    const int leftIndex = get<0>(mit->second);

                    // Monocular observation
                    if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] < 0)
                    {
                        const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->pCamera = pKFi->mpCamera;

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);

                        nEdges++;
                    }
                    else if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] >= 0) // Stereo observation
                    {
                        const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                        Eigen::Matrix<double, 3, 1> obs;
                        const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                        if (optimizer.vertex(id) && optimizer.vertex(pKFi->mnId))
                        {
                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                            e->setInformation(Info);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberStereo);

                            e->fx = pKFi->fx;
                            e->fy = pKFi->fy;
                            e->cx = pKFi->cx;
                            e->cy = pKFi->cy;
                            e->bf = pKFi->mbf;

                            optimizer.addEdge(e);
                            vpEdgesStereo.push_back(e);
                            vpEdgeKFStereo.push_back(pKFi);
                            vpMapPointEdgeStereo.push_back(pMP);

                            nEdges++;
                        }
                    }

                    if (pKFi->mpCamera2)
                    {
                        int rightIndex = get<1>(mit->second);

                        if (rightIndex != -1)
                        {
                            rightIndex -= pKFi->NLeft;

                            Eigen::Matrix<double, 2, 1> obs;
                            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                            obs << kp.pt.x, kp.pt.y;

                            ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                            if (optimizer.vertex(id) && optimizer.vertex(pKFi->mnId))
                            {
                                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                                e->setMeasurement(obs);
                                const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
                                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                                e->setRobustKernel(rk);
                                rk->setDelta(thHuberMono);

                                Sophus::SE3f Trl = pKFi->GetRelativePoseTrl();
                                e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                                e->pCamera = pKFi->mpCamera2;

                                optimizer.addEdge(e);
                                vpEdgesBody.push_back(e);
                                vpEdgeKFBody.push_back(pKFi);
                                vpMapPointEdgeBody.push_back(pMP);

                                nEdges++;
                            }
                        }
                    }
                }
            }
        }
        num_edges = nEdges;

        // Markers (Local Optimization)
        for (list<Marker *>::iterator idx = localMarkerList.begin(), lend = localMarkerList.end(); idx != lend; idx++)
        {
            // Adding a vertex for each marker
            Marker *pMapMarker = *idx;
            g2o::VertexSE3Expmap *vMarker = new g2o::VertexSE3Expmap();
            vMarker->setEstimate(g2o::SE3Quat(pMapMarker->getGlobalPose().unit_quaternion().cast<double>(),
                                              pMapMarker->getGlobalPose().translation().cast<double>()));
            int opId = maxOpId + nMarkers;
            vMarker->setId(opId);
            optimizer.addVertex(vMarker);
            nMarkers++;

            // Setting the local optimization ID for the marker
            pMapMarker->setOpId(opId);

            // 🚧 [vS-Graphs v.2.0] in contrast with the first version of visual S-Graphs, where there was an edge between
            // the marker and the keyframe, in this version we removed that edge and added an edge between the plane and the
            // keyframe, while still keeping the edge between the marker and the plane.
        }

        maxOpId += nMarkers;

        // Planes (Local Optimization)
        for (list<Plane *>::iterator idx = localPlaneList.begin(), lend = localPlaneList.end(); idx != lend; idx++)
        {
            // Adding a vertex for each plane
            Plane *pMapPlane = *idx;
            g2o::VertexPlane *vPlane = new g2o::VertexPlane();
            int opId = maxOpId + nPlanes;
            vPlane->setId(opId);
            if (sysParams->optimization.marginalize_planes)
                vPlane->setMarginalized(true);
            g2o::Plane3D planeGlobalEquation = pMapPlane->getGlobalEquation();
            vPlane->setEstimate(planeGlobalEquation);
            // if the initial alignment is complete and it the BIM values associated is between 1 and 6, fix the plane
            // if (g_initialAlignmentComplete && pMapPlane->getBIMId() >= 1 && pMapPlane->getBIMId() <= 6)
            // // if (g_initialAlignmentComplete) 
            // {
            //     vPlane->setFixed(true);
            // }
            // else{
            
            //     vPlane->setFixed(false);
            // }
            optimizer.addVertex(vPlane);
            nPlanes++;

            // Setting the local optimization ID for the plane
            pMapPlane->setOpId(opId);

            // Adding edge Plane and MapPoints -- is disabled for now
            if (sysParams->optimization.plane_map_point.enabled && !sysParams->optimization.marginalize_planes)
            {
                std::cout << "Adding Plane-MapPoint edges for Plane #" << pMapPlane->getId() << std::endl;
                set<MapPoint *> sMPs = pMapPlane->getMapPoints();
                for (set<MapPoint *>::iterator lit = sMPs.begin(), lend = sMPs.end(); lit != lend; lit++)
                {
                    MapPoint *pMP = *lit;

                    if (!pMP || pMP->isBad())
                        continue;

                    if (optimizer.vertex(opId) && optimizer.vertex(pMP->mnId + maxKFid + 1))
                    {
                        ORB_SLAM3::EdgeVertexPlaneProjectPointXYZ *e = new ORB_SLAM3::EdgeVertexPlaneProjectPointXYZ();
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pMP->mnId + maxKFid + 1)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opId)));
                        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * sysParams->optimization.plane_map_point.information_gain);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber1D);
                        optimizer.addEdge(e);
                        nEdges++;
                    }
                }
            }

            // Plane-KeyFrame Edges
            const map<KeyFrame *, ORB_SLAM3::Plane::Observation> observations = pMapPlane->getObservations();
            for (map<KeyFrame *, ORB_SLAM3::Plane::Observation>::const_iterator obsId = observations.begin(), obLast = observations.end(); obsId != obLast; obsId++)
            {
                KeyFrame *pKFi = obsId->first;
                ORB_SLAM3::Plane::Observation obs = obsId->second;

                if (pKFi->isBad())
                {
                    std::cout << "KF is bad" << std::endl;
                    pMapPlane->eraseObservation(pKFi);
                    continue;
                }

                if (pKFi->GetMap() != pCurrentMap)
                {
                    std::cout << "KF is not in the current map" << std::endl;
                    continue;
                }

                if (optimizer.vertex(opId) && optimizer.vertex(pKFi->mnId))
                {
                    // adding plane-KF constraints -- is enabled for now
                    if (sysParams->optimization.plane_kf.enabled)
                    {
                        ORB_SLAM3::EdgeVertexPlaneProjectSE3KF *e = new ORB_SLAM3::EdgeVertexPlaneProjectSE3KF();
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opId)));
                        
                        double information_gain = sysParams->optimization.plane_kf.information_gain;
                        
                        // Check if this plane is matched with BIM
                        // bool isMatchedWithBIM = false;
                        // {
                        //     std::lock_guard<std::mutex> lock(g_localMatchedPlanesMutex);
                        //     isMatchedWithBIM = (std::find(g_localMatchedPlanes.begin(), g_localMatchedPlanes.end(), pMapPlane) != g_localMatchedPlanes.end());
                        // }
                        
                        // // if (isMatchedWithBIM && !g_initialAlignmentComplete) {
                        // information_gain *= 10000.0; // Make constraint 10,000x stronger!
                        // // }
                        
                        // e->setInformation(Eigen::Matrix<double, 3, 3>::Identity() * obs.confidence * information_gain);
                        
                        e->setInformation(Eigen::Matrix<double, 3, 3>::Identity() * obs.confidence * sysParams->optimization.plane_kf.information_gain);
                        e->setMeasurement(obs.localPlane);

                         // Try removing this 3 lines
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);
                        
                        optimizer.addEdge(e);
                        nEdges++;

                        vpEdgesPlane.push_back(e);
                        vpEdgeKFPlane.push_back(pKFi);
                        vpPlaneEdgePlane.push_back(pMapPlane);
                    }

                    // adding plane-KF constraints with point observations -- is disabled for now
                    if (sysParams->optimization.plane_point.enabled)
                    {
                        // get the class index of the plane
                        int clsCloudIdx = Utils::getClassIdFromPlaneType(pMapPlane->getPlaneType());
                        if (clsCloudIdx != -1)
                        {
                            std::cout << "Adding Plane-Point edge for Plane #" << pMapPlane->getId() 
                                    << " in KeyFrame #" << pKFi->mnId << std::endl;
                            // add the plane-point constraint
                            ORB_SLAM3::EdgeSE3KFPointToPlane *e = new ORB_SLAM3::EdgeSE3KFPointToPlane();
                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opId)));

                            // double information_gain = sysParams->optimization.plane_kf.information_gain;
                            
                            // Check if this plane is matched with BIM
                            // bool isMatchedWithBIM = false;
                            // {
                            //     std::lock_guard<std::mutex> lock(g_localMatchedPlanesMutex);
                            //     isMatchedWithBIM = (std::find(g_localMatchedPlanes.begin(), g_localMatchedPlanes.end(), pMapPlane) != g_localMatchedPlanes.end());
                            // }
                            
                            // if (isMatchedWithBIM && !g_initialAlignmentComplete) {
                            // information_gain *= 10000.0; // Make constraint 10,000x stronger!
                            // // }
                            
                            // e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * obs.confidence * information_gain);
                            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * obs.confidence * sysParams->optimization.plane_point.information_gain);
                            e->setMeasurement(obs.Gij);

                            // Try removing this 3 lines
                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuber1D);
                            
                            optimizer.addEdge(e);
                            nEdges++;

                            vpEdgesPlanePoint.push_back(e);
                            vpEdgeKFPlanePoint.push_back(pKFi);
                            vpPlaneEdgePlanePoint.push_back(pMapPlane);
                        }
                    }
                }
            }

            // 🚧 [vS-Graphs v.2.0] in contrast with the first version of visual S-Graphs, where there was an edge between
            // Markers and Planes, in this version we removed that edge
            // vector<Marker *> attachedMarkers = pMapPlane->getMarkers();
            // for (const auto &planeMarker : attachedMarkers)
            // {
            //     if (optimizer.vertex(opId) && optimizer.vertex(planeMarker->getOpId()))
            //     {
            //         // Adding an edge between the Plane and the Marker
            //         ORB_SLAM3::EdgeVertexPlaneProjectSE3M *e = new ORB_SLAM3::EdgeVertexPlaneProjectSE3M();
            //         e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opId)));
            //         e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(planeMarker->getOpId())));
            //         e->setInformation(Eigen::Matrix<double, 4, 4>::Identity());

            //         g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            //         e->setRobustKernel(rk);
            //         rk->setDelta(thHuberMono);
            //         optimizer.addEdge(e);
            //     }
            // }
        }

        maxOpId += nPlanes;

        // Rooms (Local Optimization)
        for (list<Room *>::iterator idx = localRoomList.begin(), lend = localRoomList.end(); idx != lend; idx++)
        {
            // Variables
            Room *pMapRoom = *idx;
            std::vector<Plane *> walls = pMapRoom->getWalls();

            // No need to optimize if there are no walls
            if (walls.size() < 2)
                continue;

            // Adding a vertex for each room
            g2o::VertexSE3Expmap *vrtxRoom = new g2o::VertexSE3Expmap();

            int opId = maxOpId + nRooms;
            vrtxRoom->setId(opId);
            vrtxRoom->setEstimate(g2o::SE3Quat(Eigen::Quaterniond::Identity(),
                                               pMapRoom->getRoomCenter().cast<double>()));
            optimizer.addVertex(vrtxRoom);
            nRooms++;

            // Setting the local optimization ID for the room
            pMapRoom->setOpId(opId);

            if (pMapRoom->getIsCorridor())
            {
                if (optimizer.vertex(opId) && optimizer.vertex(walls[0]->getOpId()) && optimizer.vertex(walls[1]->getOpId()))
                {
                    // Adding an edge between the room and the two walls
                    ORB_SLAM3::EdgeVertex2PlaneProjectSE3Room *e = new ORB_SLAM3::EdgeVertex2PlaneProjectSE3Room();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opId)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[0]->getOpId())));
                    e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[1]->getOpId())));
                    e->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);
                    optimizer.addEdge(e);
                }
            }
            else
            {
                if (optimizer.vertex(opId) && optimizer.vertex(walls[0]->getOpId()) && optimizer.vertex(walls[1]->getOpId()) && optimizer.vertex(walls[2]->getOpId()) && optimizer.vertex(walls[2]->getOpId()))
                {
                    // Adding an edge between the room and the four walls
                    ORB_SLAM3::EdgeVertex4PlaneProjectSE3Room *e = new ORB_SLAM3::EdgeVertex4PlaneProjectSE3Room();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opId)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[0]->getOpId())));
                    e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[1]->getOpId())));
                    e->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[2]->getOpId())));
                    e->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(walls[3]->getOpId())));
                    e->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);
                    optimizer.addEdge(e);
                }
            }

            // [TODO] Adding an edge between the Room and the Meta-marker
            // Marker *metaMarker = pMapRoom->getMetaMarker();
            // if (optimizer.vertex(opId) && optimizer.vertex(metaMarker->getOpId()))
            // {
            //     // Adding an edge between the Plane and the Marker
            //     ORB_SLAM3::EdgeVertexSE3RoomProjectSE3Marker *e = new ORB_SLAM3::EdgeVertexSE3RoomProjectSE3Marker();
            //     // e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opId)));
            //     // e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(metaMarker->getOpId())));
            //     // e->setInformation(Eigen::Matrix<double, 4, 4>::Identity());

            //     // g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            //     // e->setRobustKernel(rk);
            //     // rk->setDelta(thHuberMono);
            //     // optimizer.addEdge(e);
            // }
        }
        maxOpId += nRooms;

        // // Doors (Local Optimization)
        // for (list<Room *>::iterator idx = localRoomList.begin(), lend = localRoomList.end(); idx != lend; idx++)
        // {
        //     Room *pMapRoom = *idx;
        //     vector<Door *> doors = pMapRoom->getDoors();
        //     for (const auto &door : doors)
        //     {
        //         // Adding a vertex for each door
        //         g2o::VertexSE3Expmap *vDoor = new g2o::VertexSE3Expmap();
        //         int opId = maxOpId + nDoors;
        //         vDoor->setId(opId);
        //         vDoor->setEstimate(g2o::SE3Quat(door->getGlobalPose().unit_quaternion().cast<double>(),
        //                                         door->getGlobalPose().translation().cast<double>()));
        //         optimizer.addVertex(vDoor);
        //         nDoors++;

        //         // Setting the local optimization ID for the door
        //         door->setOpId(opId);

        //         if (optimizer.vertex(opId) && optimizer.vertex(door->getOpId()))
        //         {
        //             // Adding an edge between the room and the door
        //             ORB_SLAM3::EdgeSE3DoorProjectSE3Room *e = new ORB_SLAM3::EdgeSE3DoorProjectSE3Room();
        //             e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(opId)));
        //             e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(door->getOpId())));
        //             e->setInformation(Eigen::MatrixXd::Identity(6, 6));

        //             Eigen::Isometry3d relativePose = Eigen::Isometry3d::Identity();
        //             relativePose.matrix() = (dynamic_cast<g2o::VertexSE3Expmap *>((optimizer.vertex(pMapRoom->getOpId())))->estimate().inverse() *
        //                                      dynamic_cast<g2o::VertexSE3Expmap *>((optimizer.vertex(door->getOpId())))->estimate())
        //                                         .to_homogeneous_matrix();
        //             e->setMeasurement(relativePose);

        //             g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        //             e->setRobustKernel(rk);
        //             rk->setDelta(thHuberMono);
        //             optimizer.addEdge(e);
        //         }
        //     }
        // }
        // maxOpId += nDoors;

        
        /*
        
        // BIM Walls vertices (Global Optimization) - 
        for (const auto &vpBIMWall : vpBIMWalls)
        {
            // Adding a vertex for each BIM wall
            g2o::VertexPlane *vBIMPlane = new g2o::VertexPlane();
            int opId = maxOpId + nBIMWalls; // Continue numbering after regular planes
            vBIMPlane->setId(opId);
            vBIMPlane->setEstimate(vpBIMWall->getGlobalEquation());
            optimizer.addVertex(vBIMPlane);
            
            vBIMPlane->setFixed(true); // BIM walls are fixed (from external data)
            nBIMWalls++;

            // Setting the global optimization ID for the BIM wall
            vpBIMWall->setOpId(opId);
        }

        maxOpId += nBIMWalls;

        
        // Step 1: Match first detected plane with BIM wall id=1, then find perpendicular plane for BIM wall id=3
        
        // DO MATCHING BETWEEN DETECTED PLANES AND BIM WALLS
        if (!g_initialAlignmentComplete && !vpBIMWalls.empty() && !localPlaneList.empty()) {
            std::cout << "\n=== BIM-Detected Plane Matching ===" << std::endl;
            // Step 1: Match first detected plane (id=0) with BIM wall (id=1)
            if (!g_LOCALfirstPlaneMatched) {
                for (const auto& detectedPlane : localPlaneList) {
                    if (!detectedPlane || detectedPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                        continue;
                    
                    if (detectedPlane) {
                        // Check if we have BIM wall with id=1
                        for (const auto& bimWall : vpBIMWalls) {
                            if (bimWall && bimWall->getBIMId() == g_BIM_wallId_1) {
                                g_LOCALfirstDetectedPlaneId = detectedPlane->getId();
                                g_LOCALfirstPlaneMatched = true;

                                {
                                    std::lock_guard<std::mutex> lock(g_localMatchedPlanesMutex);
                                    if (std::find(g_localMatchedPlanes.begin(), g_localMatchedPlanes.end(), detectedPlane) == g_localMatchedPlanes.end()) {
                                        g_localMatchedPlanes.push_back(detectedPlane);
                                    }
                                }
                                
                                std::cout << "✓ FIRST MATCH: Detected Plane #" << detectedPlane->getId() 
                                         << " matched with BIM Wall #" << bimWall->getBIMId() << std::endl;
                                break;
                            }
                        }
                        break;
                    }
                }
            }

            // Step 2: Find perpendicular plane to match with BIM wall id=3
            if (g_LOCALfirstPlaneMatched && !g_LOCALsecondPlaneMatched) {
                // Get the first matched plane
                Plane* firstPlane = nullptr;
                for (const auto& detectedPlane : localPlaneList) {
                    if (detectedPlane && detectedPlane->getId() == g_LOCALfirstDetectedPlaneId) {
                        firstPlane = detectedPlane;
                        break;
                    }
                }

                if (firstPlane) {
                    g2o::Plane3D firstPlaneEq = firstPlane->getGlobalEquation();
                    Eigen::Vector3d firstNormal = firstPlaneEq.normal();

                    // Look for a perpendicular plane
                    for (const auto& detectedPlane : localPlaneList) {
                        if (!detectedPlane || detectedPlane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                            continue;
                        
                        if (detectedPlane->getId() == g_LOCALfirstDetectedPlaneId)
                            continue; // Skip the first plane itself

                        g2o::Plane3D candidateEq = detectedPlane->getGlobalEquation();
                        Eigen::Vector3d candidateNormal = candidateEq.normal();

                        // Check if planes are perpendicular (dot product close to 0)
                        double dotProduct = abs(firstNormal.dot(candidateNormal));
                        double perpendicularThreshold = 0.2; // Adjust as needed (0 = perfect perpendicular)

                        if (dotProduct < perpendicularThreshold) {
                            // Check if we have BIM wall with id=3
                            for (const auto& bimWall : vpBIMWalls) {
                                if (bimWall && bimWall->getBIMId() == g_BIM_wallId_2) {
                                    g_LOCALsecondDetectedPlaneId = detectedPlane->getId();
                                    g_LOCALsecondPlaneMatched = true;
                                    g_LOCALmatchingComplete = true; 

                                    {
                                        std::lock_guard<std::mutex> lock(g_localMatchedPlanesMutex);
                                        if (std::find(g_localMatchedPlanes.begin(), g_localMatchedPlanes.end(), detectedPlane) == g_localMatchedPlanes.end()) {
                                            g_localMatchedPlanes.push_back(detectedPlane);
                                        }
                                    }
                                    
                                    std::cout << "✓ SECOND MATCH: Detected Plane #" << detectedPlane->getId() 
                                             << " (perpendicular to Plane #" << g_LOCALfirstDetectedPlaneId 
                                             << ") matched with BIM Wall #" << bimWall->getBIMId() << std::endl;
                                    
                                      
                                    // Print summary of both matches
                                    std::cout << "\n=== MATCHING SUMMARY ===" << std::endl;
                                    std::cout << "✓ Match 1: Detected Plane #" << g_LOCALfirstDetectedPlaneId 
                                             << " ↔ BIM Wall #1" << std::endl;
                                    std::cout << "✓ Match 2: Detected Plane #" << g_LOCALsecondDetectedPlaneId 
                                             << " ↔ BIM Wall #3" << std::endl;
                                    std::cout << "=== MATCHING COMPLETE ===" << std::endl;

                                    // Log the optimization id for both planes
                                    std::cout << "Detected Plane #" << g_LOCALfirstDetectedPlaneId 
                                              << " OpIdG: " << firstPlane->getOpId() << std::endl;
                                    std::cout << "Detected Plane #" << g_LOCALsecondDetectedPlaneId 
                                              << " OpIdG: " << detectedPlane->getOpId() << std::endl; 
                                    std::cout << "BIM Wall #1 OpIdG: " << bimWall->getOpId() << std::endl;
                                    std::cout << "BIM Wall #3 OpIdG: " << bimWall->getOpId() << std::endl;
                                    
                                    goto matching_complete; // Exit all loops
                                }
                            }
                        }
                    }
                }
            }

            matching_complete:
            
            // Status update
            if (!g_LOCALfirstPlaneMatched) {
                std::cout << "⏳ Waiting for detected plane with id=0..." << std::endl;
            } else if (!g_LOCALsecondPlaneMatched) {
                std::cout << "⏳ First plane matched. Waiting for perpendicular plane..." << std::endl;
                std::cout << "   → Looking for plane perpendicular to detected plane #" << g_LOCALfirstDetectedPlaneId << std::endl;
            }
            std::cout << "=== End Matching ===" << std::endl;
        }

        // Find the matched detected planes and BIM walls
        Plane* detectedPlane1 = nullptr;  // Detected plane #0
        Plane* detectedPlane2 = nullptr;  // Detected plane #7
        Plane* bimWall1 = nullptr;        // BIM wall #1
        Plane* bimWall3 = nullptr;        // BIM wall #3

        // Find detected planes by ID
        for (const auto& plane : localPlaneList) {
            if (!plane || plane->getPlaneType() == Plane::planeVariant::UNDEFINED)
                continue;
            std::cout << "Local graph Plane " << plane->getId() << " is with BIM:" << plane->getBIMId() << " is with OpId:" << plane->getOpId() << std::endl;
            // print g_LOCALfirstDetectedPlaneId and g_LOCALsecondDetectedPlaneId
            if (plane->getId() == g_LOCALfirstDetectedPlaneId) {
                detectedPlane1 = plane;
            }
            if (plane->getId() == g_LOCALsecondDetectedPlaneId) {
                detectedPlane2 = plane;
            }
        }
        
        // Find BIM walls by ID
        for (const auto& bimWall : vpBIMWalls) {
            if (!bimWall) continue;
            
            if (bimWall->getBIMId() == g_BIM_wallId_1) {
                bimWall1 = bimWall;
            }
            if (bimWall->getBIMId() == g_BIM_wallId_2) {
                bimWall3 = bimWall;
            }
        }      

        // CREATE EDGES FOR INITAL ALIGNMENT
        if (g_LOCALmatchingComplete) {
            std::cout << "\n=== Creating Plane-to-BIM Alignment Edges ===" << std::endl;
            
            // Verify all planes were found
            if (detectedPlane1 && detectedPlane2 && bimWall1 && bimWall3) {
                std::cout << "✓ All matched planes found - creating alignment edges" << std::endl;
                
                detectedPlane1->setBIMId(bimWall1->getBIMId()); // Set BIM ID 1 for detected plane #0
                detectedPlane2->setBIMId(bimWall3->getBIMId()); // Set BIM ID 3 for detected plane #1

                
                std::cout << "✓ BIM IDs assigned:" << std::endl;
                std::cout << "  - Detected Plane #" << g_LOCALfirstDetectedPlaneId 
                        << " assigned BIM ID: " << detectedPlane1->getBIMId() << std::endl;
                std::cout << "  - Detected Plane #" << g_LOCALsecondDetectedPlaneId 
                        << " assigned BIM ID: " << detectedPlane2->getBIMId() << std::endl;
    

                // Create edge between detected plane #0 and BIM wall #1
                if (optimizer.vertex(detectedPlane1->getOpId()) && optimizer.vertex(bimWall1->getOpId())) {
                    std::cout<<" inside dge creation"<<std::endl;
                    Edge2Planes *edge1 = new Edge2Planes();
                    edge1->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(bimWall1->getOpId())));
                    edge1->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(detectedPlane1->getOpId())));
                                        
                    // Set high information matrix for strong alignment
                    edge1->setInformation(Eigen::Matrix3d::Identity() * 1e10); // Strong constraint
                    
                    // Add robust kernel
                    g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
                    edge1->setRobustKernel(rk1);
                    rk1->setDelta(thHuber3D);
                    
                    optimizer.addEdge(edge1);
                    std::cout << "  → Edge created: Detected Plane #" << g_LOCALfirstDetectedPlaneId 
                              << " ↔ BIM Wall #1" << std::endl;
                }
                
                // Create edge between detected plane #X and BIM wall #3
                if (optimizer.vertex(detectedPlane2->getOpId()) && optimizer.vertex(bimWall3->getOpId())) {
                    Edge2Planes *edge2 = new Edge2Planes();
                    edge2->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(bimWall3 ->getOpId())));
                    edge2->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(detectedPlane2->getOpId())));
                    
                    // Set measurement - difference should be zero for perfect alignment
                    Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
                    edge2->setMeasurement(measurement);
                    
                    // Set high information matrix for strong alignment
                    edge2->setInformation(Eigen::Matrix3d::Identity() * 1e10); // Strong constraint
                    
                    // Add robust kernel
                    g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
                    edge2->setRobustKernel(rk2);
                    rk2->setDelta(thHuber3D);
                    
                    optimizer.addEdge(edge2);
                }

            }
        }

         // Print plane equations BEFORE optimization
        std::cout << "\n=== PLANE EQUATIONS BEFORE OPTIMIZATION ===" << std::endl;
        if (detectedPlane1) {
            g2o::Plane3D eq1 = detectedPlane1->getGlobalEquation();
            std::cout << "Detected Plane #" << g_firstDetectedPlaneId << " equation: [" 
                      << eq1.coeffs().transpose() << "]" << std::endl;
        }
        if (detectedPlane2) {
            g2o::Plane3D eq2 = detectedPlane2->getGlobalEquation();
            std::cout << "Detected Plane #" << g_secondDetectedPlaneId << " equation: [" 
                      << eq2.coeffs().transpose() << "]" << std::endl;
        }
        if (bimWall1) {
            g2o::Plane3D bim1 = bimWall1->getGlobalEquation();
            std::cout << "BIM Wall #1 equation: [" 
                      << bim1.coeffs().transpose() << "]" << std::endl;
        }
        if (bimWall3) {
            g2o::Plane3D bim3 = bimWall3->getGlobalEquation();
            std::cout << "BIM Wall #3 equation: [" 
                      << bim3.coeffs().transpose() << "]" << std::endl;
        }
        */

        // abort if no edges
        if (nEdges == 0)
        {
            Verbose::PrintMess("LM-LBA: There are 0 edges in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
            return;
        }

        if (pbStopFlag)
            if (*pbStopFlag)
                return;
        
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        // Visualize and auto-publish Local BA Graph
        // auto visualization_markers = vs_graphs_visualization::visualizeLocalBAGraph(
        //     &optimizer, "world", 5000);

        // print that local optimization is done

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() + vpEdgesStereo.size());

        vector<pair<KeyFrame *, Plane *>> vToErasePlane;
        vToErasePlane.reserve(vpEdgesPlane.size() * 2);

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
            MapPoint *pMP = vpMapPointEdgeBody[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFBody[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesPlane.size(); i < iend; i++)
        {
            ORB_SLAM3::EdgeVertexPlaneProjectSE3KF *e = vpEdgesPlane[i];
            Plane *vpPlane = vpPlaneEdgePlane[i];

            if (e->chi2() > 7.815 || !e->isDistanceCorrect())
            {

                // if not already in ToErase, add it
                std::pair<KeyFrame *, Plane *> pKFPlane = make_pair(vpEdgeKFPlane[i], vpPlane);
                if (std::find(vToErasePlane.begin(), vToErasePlane.end(), pKFPlane) == vToErasePlane.end())
                    vToErasePlane.push_back(pKFPlane);
            }
        }

        for (size_t i = 0, iend = vpEdgesPlanePoint.size(); i < iend; i++)
        {
            ORB_SLAM3::EdgeSE3KFPointToPlane *e = vpEdgesPlanePoint[i];
            Plane *vpPlane = vpPlaneEdgePlanePoint[i];

            if (e->chi2() > 3.841 || !e->isDistanceCorrect())
            {
                // if not already in ToErase, add it
                std::pair<KeyFrame *, Plane *> pKFPlane = make_pair(vpEdgeKFPlanePoint[i], vpPlane);
                if (std::find(vToErasePlane.begin(), vToErasePlane.end(), pKFPlane) == vToErasePlane.end())
                    vToErasePlane.push_back(pKFPlane);
            }
        }

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        if (!vToErase.empty())
        {
            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        if (!vToErasePlane.empty())
        {
            for (size_t i = 0; i < vToErasePlane.size(); i++)
            {
                KeyFrame *pKFi = vToErasePlane[i].first;
                Plane *pPlane = vToErasePlane[i].second;
                pPlane->eraseObservation(pKFi);
                pKFi->RemoveMapPlane(pPlane);
            }
        }

        // Recovering optimized data (Local Optimization)
        // Locally Optimized Keyframes
        for (list<KeyFrame *>::iterator lit = localKeyFrameList.begin(), lend = localKeyFrameList.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
            pKFi->SetPose(Tiw);
        }

        // Locally Optimized Points
        for (list<MapPoint *>::iterator lit = localMapPointList.begin(), lend = localMapPointList.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }

        // Locally Optimized Markers
        for (list<Marker *>::iterator idx = localMarkerList.begin(), lend = localMarkerList.end(); idx != lend; idx++)
        {
            Marker *pMapMarker = *idx;
            g2o::VertexSE3Expmap *vMarker = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pMapMarker->getOpId()));
            g2o::SE3Quat SE3quat = vMarker->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
            pMapMarker->setGlobalPose(Tiw);
        }

        // Locally Optimized Planes
        for (list<Plane *>::iterator idx = localPlaneList.begin(), lend = localPlaneList.end(); idx != lend; idx++)
        {
            Plane *pMapPlane = *idx;
            g2o::VertexPlane *vPlane = static_cast<g2o::VertexPlane *>(optimizer.vertex(pMapPlane->getOpId()));
            g2o::Plane3D planePlane = vPlane->estimate();
            pMapPlane->setGlobalEquation(planePlane);
        }

        // Locally Optimized Rooms
        for (list<Room *>::iterator idx = localRoomList.begin(), lend = localRoomList.end(); idx != lend; idx++)
        {
            Room *pMapRoom = *idx;
            g2o::VertexSE3Expmap *vrtxRoom = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pMapRoom->getOpId()));
            g2o::SE3Quat SE3quat = vrtxRoom->estimate();
            pMapRoom->setRoomCenter(SE3quat.translation());

            // Locally Optimized Doors
            for (const auto door : pMapRoom->getDoors())
            {
                Door *pMapDoor = door;
                g2o::VertexSE3Expmap *vDoor = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pMapDoor->getOpId()));
                g2o::SE3Quat SE3quat = vDoor->estimate();
                Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
                pMapDoor->setGlobalPose(Tiw);
            }
        }
        pMap->IncreaseChangeIndex();

        //  // NEW: Check initial alignment directly after optimization (only if matching complete but alignment not confirmed)
        // if (g_LOCALmatchingComplete && !g_initialAlignmentComplete) {
        //     const double alignmentThreshold = 0.15; // Slightly more lenient threshold
        //     if (detectedPlane1 && detectedPlane2 && bimWall1 && bimWall3) {
        //         // Get optimized plane equations
        //         g2o::Plane3D detectedEq1 = detectedPlane1->getGlobalEquation();
        //         g2o::Plane3D detectedEq2 = detectedPlane2->getGlobalEquation();
        //         g2o::Plane3D bimEq1 = bimWall1->getGlobalEquation();
        //         g2o::Plane3D bimEq3 = bimWall3->getGlobalEquation();
                
        //         // Use ominus function to compute plane differences (same as Edge2Planes)
        //         Eigen::Vector3d error1 = detectedEq1.ominus(bimEq1);
        //         Eigen::Vector3d error2 = detectedEq2.ominus(bimEq3);
                
        //         // Calculate distances using norm of ominus result
        //         double distance1 = error1.norm();
        //         double distance2 = error2.norm();
                
        //         // Check if both planes are well aligned
        //         bool aligned = (distance1 < alignmentThreshold) && (distance2 < alignmentThreshold);
                
        //         if (aligned) {
        //             g_initialAlignmentComplete = true;
        //         }
        //     }
        // }
        
    }

    void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *>> &LoopConnections, const bool &bFixScale)
    {
        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
            new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
        g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        solver->setUserLambdaInit(1e-16);
        optimizer.setAlgorithm(solver);

        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

        const unsigned int nMaxKFid = pMap->GetMaxKFid();

        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
        vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

        vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1); // For debugging
        Eigen::Vector3d z_vec;
        z_vec << 0.0, 0.0, 1.0;

        const int minFeat = 100;

        // Set KeyFrame vertices
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

            const int nIDi = pKF->mnId;

            LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

            if (it != CorrectedSim3.end())
            {
                vScw[nIDi] = it->second;
                VSim3->setEstimate(it->second);
            }
            else
            {
                Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
                g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
                vScw[nIDi] = Siw;
                VSim3->setEstimate(Siw);
            }

            if (pKF->mnId == pMap->GetInitKFid())
                VSim3->setFixed(true);

            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = bFixScale;

            optimizer.addVertex(VSim3);
            vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec; // For debugging

            vpVertices[nIDi] = VSim3;
        }

        set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

        const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

        // Set Loop edges
        int count_loop = 0;
        for (map<KeyFrame *, set<KeyFrame *>>::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            const long unsigned int nIDi = pKF->mnId;
            const set<KeyFrame *> &spConnections = mit->second;
            const g2o::Sim3 Siw = vScw[nIDi];
            const g2o::Sim3 Swi = Siw.inverse();

            for (set<KeyFrame *>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++)
            {
                const long unsigned int nIDj = (*sit)->mnId;
                if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat)
                    continue;

                const g2o::Sim3 Sjw = vScw[nIDj];
                const g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                e->setMeasurement(Sji);

                e->information() = matLambda;

                optimizer.addEdge(e);
                count_loop++;
                sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
            }
        }

        // Set normal edges
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];

            const int nIDi = pKF->mnId;

            g2o::Sim3 Swi;

            LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

            if (iti != NonCorrectedSim3.end())
                Swi = (iti->second).inverse();
            else
                Swi = vScw[nIDi].inverse();

            KeyFrame *pParentKF = pKF->GetParent();

            // Spanning tree edge
            if (pParentKF)
            {
                int nIDj = pParentKF->mnId;

                g2o::Sim3 Sjw;

                LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

                if (itj != NonCorrectedSim3.end())
                    Sjw = itj->second;
                else
                    Sjw = vScw[nIDj];

                g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                e->setMeasurement(Sji);
                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // Loop edges
            const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
            for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++)
            {
                KeyFrame *pLKF = *sit;
                if (pLKF->mnId < pKF->mnId)
                {
                    g2o::Sim3 Slw;

                    LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                    if (itl != NonCorrectedSim3.end())
                        Slw = itl->second;
                    else
                        Slw = vScw[pLKF->mnId];

                    g2o::Sim3 Sli = Slw * Swi;
                    g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                    el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                }
            }

            // Covisibility graph edges
            const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
            for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++)
            {
                KeyFrame *pKFn = *vit;
                if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/)
                {
                    if (!pKFn->isBad() && pKFn->mnId < pKF->mnId)
                    {
                        if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId))))
                            continue;

                        g2o::Sim3 Snw;

                        LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                        if (itn != NonCorrectedSim3.end())
                            Snw = itn->second;
                        else
                            Snw = vScw[pKFn->mnId];

                        g2o::Sim3 Sni = Snw * Swi;

                        g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                        en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
                        en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                    }
                }
            }

            // Inertial edges if inertial
            if (pKF->bImu && pKF->mPrevKF)
            {
                g2o::Sim3 Spw;
                LoopClosing::KeyFrameAndPose::const_iterator itp = NonCorrectedSim3.find(pKF->mPrevKF);
                if (itp != NonCorrectedSim3.end())
                    Spw = itp->second;
                else
                    Spw = vScw[pKF->mPrevKF->mnId];

                g2o::Sim3 Spi = Spw * Swi;
                g2o::EdgeSim3 *ep = new g2o::EdgeSim3();
                ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mPrevKF->mnId)));
                ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                ep->setMeasurement(Spi);
                ep->information() = matLambda;
                optimizer.addEdge(ep);
            }
        }

        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        // optimizer.save("orbslam3_posegraph.g2o");
        optimizer.optimize(20);
        optimizer.computeActiveErrors();
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            const int nIDi = pKFi->mnId;

            g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
            g2o::Sim3 CorrectedSiw = VSim3->estimate();
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
            double s = CorrectedSiw.scale();

            Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(), CorrectedSiw.translation().cast<float>() / s);
            pKFi->SetPose(Tiw);
        }

        // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMPs[i];

            if (pMP->isBad())
                continue;

            int nIDr;
            if (pMP->mnCorrectedByKF == pCurKF->mnId)
            {
                nIDr = pMP->mnCorrectedReference;
            }
            else
            {
                KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
                nIDr = pRefKF->mnId;
            }

            g2o::Sim3 Srw = vScw[nIDr];
            g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

            Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
            Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
            pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

            pMP->UpdateNormalAndDepth();
        }

        pMap->IncreaseChangeIndex();
    }

    void Optimizer::OptimizeEssentialGraph(KeyFrame *pCurKF, vector<KeyFrame *> &vpFixedKFs,
                                           vector<KeyFrame *> &vpFixedCorrectedKFs, vector<KeyFrame *> &vpNonFixedKFs,
                                           vector<MapPoint *> &vpNonCorrectedMPs, vector<Door *> &vpCurrentMapDoors,
                                           vector<Plane *> &vpCurrentMapPlanes, vector<Marker *> &vpCurrentMapMarkers,
                                           vector<Room *> &vpCurrentDetMapRooms, vector<Room *> &vpCurrentMrkMapRooms,
                                           vector<vector<Eigen::Vector3d>> &vpClusterPoints)
    {
        // Variables
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);

        // Linear solver and block solver
        g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
            new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
        g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        solver->setUserLambdaInit(1e-16);
        optimizer.setAlgorithm(solver);

        // Get map
        Map *pMap = pCurKF->GetMap();
        const unsigned int nMaxKFid = pMap->GetMaxKFid();

        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
        vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

        vector<bool> vpGoodPose(nMaxKFid + 1);
        vector<bool> vpBadPose(nMaxKFid + 1);

        const int minFeat = 100;

        // Loop over fixed KeyFrames
        for (KeyFrame *pKFi : vpFixedKFs)
        {
            if (pKFi->isBad())
                continue;

            g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

            const int nIDi = pKFi->mnId;

            Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

            vCorrectedSwc[nIDi] = Siw.inverse();
            VSim3->setEstimate(Siw);

            VSim3->setFixed(true);

            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = true;

            optimizer.addVertex(VSim3);

            vpVertices[nIDi] = VSim3;

            vpGoodPose[nIDi] = true;
            vpBadPose[nIDi] = false;
        }

        // Loop over fixed corrected KeyFrames
        set<unsigned long> sIdKF;
        for (KeyFrame *pKFi : vpFixedCorrectedKFs)
        {
            if (pKFi->isBad())
                continue;

            g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

            const int nIDi = pKFi->mnId;

            Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

            vCorrectedSwc[nIDi] = Siw.inverse();
            VSim3->setEstimate(Siw);

            Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
            vScw[nIDi] = g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

            VSim3->setFixed(true);

            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);

            optimizer.addVertex(VSim3);

            vpVertices[nIDi] = VSim3;

            sIdKF.insert(nIDi);

            vpGoodPose[nIDi] = true;
            vpBadPose[nIDi] = true;
        }

        // Loop over non-fixed KeyFrames
        for (KeyFrame *pKFi : vpNonFixedKFs)
        {
            if (pKFi->isBad())
                continue;

            const int nIDi = pKFi->mnId;

            if (sIdKF.count(nIDi)) // It has already added in the corrected merge KFs
                continue;

            g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

            Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);

            VSim3->setFixed(false);

            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);

            optimizer.addVertex(VSim3);

            vpVertices[nIDi] = VSim3;

            sIdKF.insert(nIDi);

            vpGoodPose[nIDi] = false;
            vpBadPose[nIDi] = true;
        }

        vector<KeyFrame *> vpKFs;
        vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() + vpNonFixedKFs.size());
        vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
        vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(), vpFixedCorrectedKFs.end());
        vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
        set<KeyFrame *> spKFs(vpKFs.begin(), vpKFs.end());

        const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

        for (KeyFrame *pKFi : vpKFs)
        {
            int num_connections = 0;
            const int nIDi = pKFi->mnId;

            g2o::Sim3 correctedSwi;
            g2o::Sim3 Swi;

            if (vpGoodPose[nIDi])
                correctedSwi = vCorrectedSwc[nIDi];
            if (vpBadPose[nIDi])
                Swi = vScw[nIDi].inverse();

            KeyFrame *pParentKFi = pKFi->GetParent();

            // Spanning tree edge
            if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end())
            {
                int nIDj = pParentKFi->mnId;

                g2o::Sim3 Sjw;
                bool bHasRelation = false;

                if (vpGoodPose[nIDi] && vpGoodPose[nIDj])
                {
                    Sjw = vCorrectedSwc[nIDj].inverse();
                    bHasRelation = true;
                }
                else if (vpBadPose[nIDi] && vpBadPose[nIDj])
                {
                    Sjw = vScw[nIDj];
                    bHasRelation = true;
                }

                if (bHasRelation)
                {
                    g2o::Sim3 Sji = Sjw * Swi;

                    g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    e->setMeasurement(Sji);

                    e->information() = matLambda;
                    optimizer.addEdge(e);
                    num_connections++;
                }
            }

            // Loop edges
            const set<KeyFrame *> sLoopEdges = pKFi->GetLoopEdges();
            for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++)
            {
                KeyFrame *pLKF = *sit;
                if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId)
                {
                    g2o::Sim3 Slw;
                    bool bHasRelation = false;

                    if (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId])
                    {
                        Slw = vCorrectedSwc[pLKF->mnId].inverse();
                        bHasRelation = true;
                    }
                    else if (vpBadPose[nIDi] && vpBadPose[pLKF->mnId])
                    {
                        Slw = vScw[pLKF->mnId];
                        bHasRelation = true;
                    }

                    if (bHasRelation)
                    {
                        g2o::Sim3 Sli = Slw * Swi;
                        g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                        el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
                        el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                        el->setMeasurement(Sli);
                        el->information() = matLambda;
                        optimizer.addEdge(el);
                        num_connections++;
                    }
                }
            }

            // Covisibility graph edges
            const vector<KeyFrame *> vpConnectedKFs = pKFi->GetCovisiblesByWeight(minFeat);
            for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++)
            {
                KeyFrame *pKFn = *vit;
                if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) && !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end())
                {
                    if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId)
                    {

                        g2o::Sim3 Snw = vScw[pKFn->mnId];
                        bool bHasRelation = false;

                        if (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId])
                        {
                            Snw = vCorrectedSwc[pKFn->mnId].inverse();
                            bHasRelation = true;
                        }
                        else if (vpBadPose[nIDi] && vpBadPose[pKFn->mnId])
                        {
                            Snw = vScw[pKFn->mnId];
                            bHasRelation = true;
                        }

                        if (bHasRelation)
                        {
                            g2o::Sim3 Sni = Snw * Swi;

                            g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                            en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
                            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                            en->setMeasurement(Sni);
                            en->information() = matLambda;
                            optimizer.addEdge(en);
                            num_connections++;
                        }
                    }
                }
            }

            if (num_connections == 0)
                Verbose::PrintMess("Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections", Verbose::VERBOSITY_DEBUG);
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(20);

        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // Inform the user
        std::cout << "\n[Optimizer]" << std::endl;

        // Loop over non-fixed KeyFrames to correct them
        // [hint] Sim3 [sR t | 0 1] --> SE3 [R t/s| 0 1]
        std::cout << "- Correcting the poses of KeyFrames ..." << std::endl;
        for (KeyFrame *pKFi : vpNonFixedKFs)
        {
            if (pKFi->isBad())
                continue;

            const int nIDi = pKFi->mnId;

            g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
            g2o::Sim3 CorrectedSiw = VSim3->estimate();
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
            double s = CorrectedSiw.scale();
            Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation() / s);

            pKFi->mTcwBefMerge = pKFi->GetPose();
            pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
            pKFi->SetPose(Tiw.cast<float>());
        }

        // Keep a corrected KeyFrame as the reference
        Sophus::SE3f TCorectedSampleKF;
        KeyFrame *pSampleRefKF = nullptr;

        for (KeyFrame *pKFi : vpNonFixedKFs)
        {
            if (!pKFi || pKFi->isBad())
                continue;

            pSampleRefKF = pKFi;
            break;
        }

        if (pSampleRefKF)
        {
            Sophus::SE3f TNonCorrectedSampleKF = pSampleRefKF->mTwcBefMerge;
            TCorectedSampleKF = pSampleRefKF->GetPoseInverse() * TNonCorrectedSampleKF.inverse();
        }

        // Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        std::cout << "- Correcting the poses of 3D mapped points ..." << std::endl;
        for (MapPoint *pMPi : vpNonCorrectedMPs)
        {
            if (pMPi->isBad())
                continue;

            KeyFrame *pRefKF = pMPi->GetReferenceKeyFrame();
            while (pRefKF->isBad())
            {
                if (!pRefKF)
                {
                    Verbose::PrintMess("MP " + to_string(pMPi->mnId) + " without a valid reference KF", Verbose::VERBOSITY_DEBUG);
                    break;
                }

                pMPi->EraseObservation(pRefKF);
                pRefKF = pMPi->GetReferenceKeyFrame();
            }

            if (vpBadPose[pRefKF->mnId])
            {
                Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
                Sophus::SE3f Twr = pRefKF->GetPoseInverse();

                Eigen::Vector3f eigCorrectedP3Dw = Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
                pMPi->SetWorldPos(eigCorrectedP3Dw);

                pMPi->UpdateNormalAndDepth();
            }
        }

        // Correct the pose of the detected planes in the new map
        std::cout << "- Correcting the poses of planes ..." << std::endl;
        for (Plane *pPlane : vpCurrentMapPlanes)
        {
            // Get a reference KeyFrame related to the plane
            std::map<KeyFrame *, ORB_SLAM3::Plane::Observation> observations = pPlane->getObservations();

            // If the plane has no observations, continue
            if (observations.empty())
                continue;

            // Set the first KeyFrame as the reference KeyFrame
            KeyFrame *pRefKF = observations.begin()->first;

            // Find a valid reference KeyFrame
            while (pRefKF->isBad())
            {
                if (!pRefKF)
                    break;

                pPlane->eraseObservation(pRefKF);
                if (observations.empty())
                    break;

                pRefKF = observations.begin()->first;
            }

            if (!pRefKF || pRefKF->isBad())
                continue;

            // Get the corrected pose of the reference KeyFrame
            if (vpBadPose[pRefKF->mnId])
            {
                Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
                Sophus::SE3f Tcorc = pRefKF->GetPoseInverse() * TNonCorrectedwr.inverse();

                // Transform the local pose to the world coordinate system
                g2o::Plane3D globalEquation =
                    Utils::applyPoseToPlane(Tcorc.matrix().cast<double>(),
                                            pPlane->getGlobalEquation());

                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud = pPlane->getMapClouds();
                pcl::transformPointCloud(*planeCloud, *planeCloud, Tcorc.matrix().cast<float>());

                // Update the global equation of the plane
                pPlane->setGlobalEquation(globalEquation);

                // Update the plane centroid
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*planeCloud, centroid);
                pPlane->setCentroid(centroid.head<3>());
            }
        }

        // Correct the pose of the detected markers in the new map
        std::cout << "- Correcting the poses of markers ..." << std::endl;
        for (Marker *pMarker : vpCurrentMapMarkers)
        {
            // Get a reference KeyFrame related to the marker
            std::map<KeyFrame *, Sophus::SE3f> observations = pMarker->getObservations();

            // If the marker has no observations, continue
            if (observations.empty())
                continue;

            // Set the first KeyFrame as the reference KeyFrame
            KeyFrame *pRefKF = observations.begin()->first;

            // Find a valid reference KeyFrame
            while (pRefKF->isBad())
            {
                if (!pRefKF)
                    break;

                observations.erase(pRefKF);
                if (observations.empty())
                    break;

                pRefKF = observations.begin()->first;
            }

            if (!pRefKF || pRefKF->isBad())
                continue;

            // Get the corrected pose of the reference KeyFrame
            if (vpBadPose[pRefKF->mnId])
            {
                Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
                Sophus::SE3f Tcorc = pRefKF->GetPoseInverse() * TNonCorrectedwr.inverse();

                // Transform the local pose to the world coordinate system
                Sophus::SE3f correctedGlobalPose = Tcorc * pMarker->getLocalPose();

                // Set the corrected global pose to the marker
                pMarker->setGlobalPose(correctedGlobalPose);
            }
        }

        // Correct the pose of the detected doors in the new map
        std::cout << "- Correcting the poses of doors ..." << std::endl;
        for (Door *pDoor : vpCurrentMapDoors)
        {
            // Get the marker related to the door
            Marker *pMarker = pDoor->getMarker();

            // If the door has no marker, continue
            if (!pMarker)
                continue;

            // Update the global pose of the door
            pDoor->setGlobalPose(pMarker->getGlobalPose());
        }

        // Correct the pose of the marker-based rooms in the new map
        std::cout << "- Correcting the poses of the marker-based room candidates ..." << std::endl;
        for (Room *pRoom : vpCurrentMrkMapRooms)
        {
            // Get the marker related to the room
            Marker *pMarker = pRoom->getMetaMarker();

            // If the room has no marker, continue
            if (!pMarker)
                continue;

            // Update the global pose of the room
            pRoom->setRoomCenter(pMarker->getGlobalPose().translation().cast<double>());
        }

        // Correct the pose of the detected rooms in the new map
        std::cout << "- Correcting the poses of the detected rooms ..." << std::endl;
        for (Room *pRoom : vpCurrentDetMapRooms)
        {
            // Get the current room center
            Eigen::Vector3d roomCenter = pRoom->getRoomCenter();

            // Check if there is a valid reference KeyFrame
            if (pSampleRefKF)
            {
                // Apply the transformation to the room center
                Eigen::Vector4d homogeneousCenter(roomCenter.x(), roomCenter.y(), roomCenter.z(), 1.0);
                Eigen::Vector4d transformedCenter = TCorectedSampleKF.matrix().cast<double>() * homogeneousCenter;

                // Convert back to a 3D point (ignore the homogeneous coordinate)
                Eigen::Vector3d correctedRoomCenter = transformedCenter.head<3>();

                // Update the global pose of the room
                pRoom->setRoomCenter(correctedRoomCenter);
            }
            else
                std::cout << "-- Skipped fixing room#" << pRoom->getId()
                          << " due to improper reference KeyFrame!" << std::endl;
        }

        // Correct the pose of the cluster points in the new map
        std::cout << "- Correcting the poses of cluster points ..." << std::endl;
        // Then, use the KeyFrame to get the corrected pose of cluster points
        if (pSampleRefKF)
        {
            for (std::vector<Eigen::Vector3d> &cluster : vpClusterPoints)
                for (Eigen::Vector3d &pPoint : cluster)
                {
                    // Apply the transformation to each point
                    Eigen::Vector4d homogeneousPoint(pPoint.x(), pPoint.y(), pPoint.z(), 1.0);
                    Eigen::Vector4d transformedPoint = TCorectedSampleKF.matrix().cast<double>() * homogeneousPoint;

                    // Update the original point with the transformed coordinates
                    pPoint = transformedPoint.head<3>();
                }
        }
        else
            std::cout << "-- Skipped fixing cluster points due to improper reference KeyFrame!" << std::endl;

        std::cout << "- Corrections finished!" << std::endl;
    }

    int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2,
                                const bool bFixScale, Eigen::Matrix<double, 7, 7> &mAcumHessian, const bool bAllPoints)
    {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // Camera poses
        const Eigen::Matrix3f R1w = pKF1->GetRotation();
        const Eigen::Vector3f t1w = pKF1->GetTranslation();
        const Eigen::Matrix3f R2w = pKF2->GetRotation();
        const Eigen::Vector3f t2w = pKF2->GetTranslation();

        // Set Sim3 vertex
        ORB_SLAM3::VertexSim3Expmap *vSim3 = new ORB_SLAM3::VertexSim3Expmap();
        vSim3->_fix_scale = bFixScale;
        vSim3->setEstimate(g2oS12);
        vSim3->setId(0);
        vSim3->setFixed(false);
        vSim3->pCamera1 = pKF1->mpCamera;
        vSim3->pCamera2 = pKF2->mpCamera;
        optimizer.addVertex(vSim3);

        // Set MapPoint vertices
        const int N = vpMatches1.size();
        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
        vector<ORB_SLAM3::EdgeSim3ProjectXYZ *> vpEdges12;
        vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ *> vpEdges21;
        vector<size_t> vnIndexEdge;
        vector<bool> vbIsInKF2;

        vnIndexEdge.reserve(2 * N);
        vpEdges12.reserve(2 * N);
        vpEdges21.reserve(2 * N);
        vbIsInKF2.reserve(2 * N);

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;
        int nBadMPs = 0;
        int nInKF2 = 0;
        int nOutKF2 = 0;
        int nMatchWithoutMP = 0;

        vector<int> vIdsOnlyInKF2;

        for (int i = 0; i < N; i++)
        {
            if (!vpMatches1[i])
                continue;

            MapPoint *pMP1 = vpMapPoints1[i];
            MapPoint *pMP2 = vpMatches1[i];

            const int id1 = 2 * i + 1;
            const int id2 = 2 * (i + 1);

            const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));

            Eigen::Vector3f P3D1c;
            Eigen::Vector3f P3D2c;

            if (pMP1 && pMP2)
            {
                if (!pMP1->isBad() && !pMP2->isBad())
                {
                    g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
                    Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
                    P3D1c = R1w * P3D1w + t1w;
                    vPoint1->setEstimate(P3D1c.cast<double>());
                    vPoint1->setId(id1);
                    vPoint1->setFixed(true);
                    optimizer.addVertex(vPoint1);

                    g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                    Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                    P3D2c = R2w * P3D2w + t2w;
                    vPoint2->setEstimate(P3D2c.cast<double>());
                    vPoint2->setId(id2);
                    vPoint2->setFixed(true);
                    optimizer.addVertex(vPoint2);
                }
                else
                {
                    nBadMPs++;
                    continue;
                }
            }
            else
            {
                nMatchWithoutMP++;

                // TODO The 3D position in KF1 doesn't exist
                if (!pMP2->isBad())
                {
                    g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                    Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                    P3D2c = R2w * P3D2w + t2w;
                    vPoint2->setEstimate(P3D2c.cast<double>());
                    vPoint2->setId(id2);
                    vPoint2->setFixed(true);
                    optimizer.addVertex(vPoint2);

                    vIdsOnlyInKF2.push_back(id2);
                }
                continue;
            }

            if (i2 < 0 && !bAllPoints)
            {
                Verbose::PrintMess("    Remove point -> i2: " + to_string(i2) + "; bAllPoints: " + to_string(bAllPoints), Verbose::VERBOSITY_DEBUG);
                continue;
            }

            if (P3D2c(2) < 0)
            {
                Verbose::PrintMess("Sim3: Z coordinate is negative", Verbose::VERBOSITY_DEBUG);
                continue;
            }

            nCorrespondences++;

            // Set edge x1 = S12*X2
            Eigen::Matrix<double, 2, 1> obs1;
            const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
            obs1 << kpUn1.pt.x, kpUn1.pt.y;

            ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();

            e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
            e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e12->setMeasurement(obs1);
            const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
            e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

            g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e12);

            // Set edge x2 = S21*X1
            Eigen::Matrix<double, 2, 1> obs2;
            cv::KeyPoint kpUn2;
            bool inKF2;
            if (i2 >= 0)
            {
                kpUn2 = pKF2->mvKeysUn[i2];
                obs2 << kpUn2.pt.x, kpUn2.pt.y;
                inKF2 = true;

                nInKF2++;
            }
            else
            {
                float invz = 1 / P3D2c(2);
                float x = P3D2c(0) * invz;
                float y = P3D2c(1) * invz;

                obs2 << x, y;
                kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

                inKF2 = false;
                nOutKF2++;
            }

            ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();

            e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
            e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e21->setMeasurement(obs2);
            float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

            g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
            e21->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e21);

            vpEdges12.push_back(e12);
            vpEdges21.push_back(e21);
            vnIndexEdge.push_back(i);

            vbIsInKF2.push_back(inKF2);
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // Check inliers
        int nBad = 0;
        int nBadOutKF2 = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++)
        {
            ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
                optimizer.removeEdge(e12);
                optimizer.removeEdge(e21);
                vpEdges12[i] = static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ *>(NULL);
                vpEdges21[i] = static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ *>(NULL);
                nBad++;

                if (!vbIsInKF2[i])
                {
                    nBadOutKF2++;
                }
                continue;
            }

            // Check if remove the robust adjustment improve the result
            e12->setRobustKernel(0);
            e21->setRobustKernel(0);
        }

        int nMoreIterations;
        if (nBad > 0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        if (nCorrespondences - nBad < 10)
            return 0;

        // Optimize again only with inliers
        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
        for (size_t i = 0; i < vpEdges12.size(); i++)
        {
            ORB_SLAM3::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            ORB_SLAM3::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            e12->computeError();
            e21->computeError();

            if (e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
            }
            else
            {
                nIn++;
            }
        }

        // Recover optimized Sim3
        g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        g2oS12 = vSim3_recov->estimate();

        return nIn;
    }

    void Optimizer::LocalInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &countFixedKF, int &num_OptKF, int &num_MPs, int &num_edges, bool bLarge, bool bRecInit)
    {
        Map *pCurrentMap = pKF->GetMap();

        int maxOpt = 10;
        int opt_it = 10;
        if (bLarge)
        {
            maxOpt = 25;
            opt_it = 4;
        }
        const int Nd = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
        const unsigned long maxKFid = pKF->mnId;

        vector<KeyFrame *> vpOptimizableKFs;
        const vector<KeyFrame *> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
        list<KeyFrame *> lpOptVisKFs;

        vpOptimizableKFs.reserve(Nd);
        vpOptimizableKFs.push_back(pKF);
        pKF->mnBALocalForKF = pKF->mnId;
        for (int i = 1; i < Nd; i++)
        {
            if (vpOptimizableKFs.back()->mPrevKF)
            {
                vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
                vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
            }
            else
                break;
        }

        int N = vpOptimizableKFs.size();

        // Optimizable points seen by temporal optimizable keyframes
        list<MapPoint *> localMapPointList;
        for (int i = 0; i < N; i++)
        {
            vector<MapPoint *> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->mnBALocalForKF != pKF->mnId)
                        {
                            localMapPointList.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
            }
        }

        // Fixed Keyframe: First frame previous KF to optimization window)
        list<KeyFrame *> lFixedKeyFrames;
        if (vpOptimizableKFs.back()->mPrevKF)
        {
            lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
        }
        else
        {
            vpOptimizableKFs.back()->mnBALocalForKF = 0;
            vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
            lFixedKeyFrames.push_back(vpOptimizableKFs.back());
            vpOptimizableKFs.pop_back();
        }

        // Optimizable visual KFs
        const int maxCovKF = 0;
        for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++)
        {
            if (lpOptVisKFs.size() >= maxCovKF)
                break;

            KeyFrame *pKFi = vpNeighsKFs[i];
            if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
                continue;
            pKFi->mnBALocalForKF = pKF->mnId;
            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                lpOptVisKFs.push_back(pKFi);

                vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
                for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
                {
                    MapPoint *pMP = *vit;
                    if (pMP)
                        if (!pMP->isBad())
                            if (pMP->mnBALocalForKF != pKF->mnId)
                            {
                                localMapPointList.push_back(pMP);
                                pMP->mnBALocalForKF = pKF->mnId;
                            }
                }
            }
        }

        // Fixed KFs which are not covisible optimizable
        const int maxFixKF = 200;

        for (list<MapPoint *>::iterator lit = localMapPointList.begin(), lend = localMapPointList.end(); lit != lend; lit++)
        {
            map<KeyFrame *, tuple<int, int>> observations = (*lit)->GetObservations();
            for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
                {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad())
                    {
                        lFixedKeyFrames.push_back(pKFi);
                        break;
                    }
                }
            }
            if (lFixedKeyFrames.size() >= maxFixKF)
                break;
        }

        bool bNonFixed = (lFixedKeyFrames.size() == 0);

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        if (bLarge)
        {
            g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            solver->setUserLambdaInit(1e-2); // to avoid iterating for finding optimal lambda
            optimizer.setAlgorithm(solver);
        }
        else
        {
            g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            solver->setUserLambdaInit(1e0);
            optimizer.setAlgorithm(solver);
        }

        // Set Local temporal KeyFrame vertices
        N = vpOptimizableKFs.size();
        for (int i = 0; i < N; i++)
        {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(false);
            optimizer.addVertex(VP);

            if (pKFi->bImu)
            {
                VertexVelocity *VV = new VertexVelocity(pKFi);
                VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
                VV->setFixed(false);
                optimizer.addVertex(VV);
                VertexGyroBias *VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                VG->setFixed(false);
                optimizer.addVertex(VG);
                VertexAccBias *VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                VA->setFixed(false);
                optimizer.addVertex(VA);
            }
        }

        // Set Local visual KeyFrame vertices
        for (list<KeyFrame *>::iterator it = lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it != itEnd; it++)
        {
            KeyFrame *pKFi = *it;
            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(false);
            optimizer.addVertex(VP);
        }

        // Set Fixed KeyFrame vertices
        for (list<KeyFrame *>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            if (pKFi->bImu) // This should be done only for keyframe just before temporal window
            {
                VertexVelocity *VV = new VertexVelocity(pKFi);
                VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
                VV->setFixed(true);
                optimizer.addVertex(VV);
                VertexGyroBias *VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                VG->setFixed(true);
                optimizer.addVertex(VG);
                VertexAccBias *VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                VA->setFixed(true);
                optimizer.addVertex(VA);
            }
        }

        // Create intertial constraints
        vector<EdgeInertial *> vei(N, (EdgeInertial *)NULL);
        vector<EdgeGyroRW *> vegr(N, (EdgeGyroRW *)NULL);
        vector<EdgeAccRW *> vear(N, (EdgeAccRW *)NULL);

        for (int i = 0; i < N; i++)
        {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            if (!pKFi->mPrevKF)
            {
                cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
                continue;
            }
            if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
            {
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
                g2o::HyperGraph::Vertex *VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
                g2o::HyperGraph::Vertex *VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
                g2o::HyperGraph::Vertex *VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
                g2o::HyperGraph::Vertex *VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

                if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
                {
                    cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2 << endl;
                    continue;
                }

                vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

                vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
                vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
                vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

                if (i == N - 1 || bRecInit)
                {
                    // All inertial residuals are included without robust cost function, but not that one linking the
                    // last optimizable keyframe inside of the local window and the first fixed keyframe out. The
                    // information matrix for this measurement is also downweighted. This is done to avoid accumulating
                    // error due to fixing variables.
                    g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
                    vei[i]->setRobustKernel(rki);
                    if (i == N - 1)
                        vei[i]->setInformation(vei[i]->information() * 1e-2);
                    rki->setDelta(sqrt(16.92));
                }
                optimizer.addEdge(vei[i]);

                vegr[i] = new EdgeGyroRW();
                vegr[i]->setVertex(0, VG1);
                vegr[i]->setVertex(1, VG2);
                Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
                vegr[i]->setInformation(InfoG);
                optimizer.addEdge(vegr[i]);

                vear[i] = new EdgeAccRW();
                vear[i]->setVertex(0, VA1);
                vear[i]->setVertex(1, VA2);
                Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
                vear[i]->setInformation(InfoA);

                optimizer.addEdge(vear[i]);
            }
            else
                cout << "ERROR building inertial edge" << endl;
        }

        // Set MapPoint vertices
        const int nExpectedSize = (N + lFixedKeyFrames.size()) * localMapPointList.size();

        // Mono
        vector<EdgeMono *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        // Stereo
        vector<EdgeStereo *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuberMono = sqrt(5.991);
        const float chi2Mono2 = 5.991;
        const float thHuberStereo = sqrt(7.815);
        const float chi2Stereo2 = 7.815;

        const unsigned long iniMPid = maxKFid * 5;

        map<int, int> mVisEdges;
        for (int i = 0; i < N; i++)
        {
            KeyFrame *pKFi = vpOptimizableKFs[i];
            mVisEdges[pKFi->mnId] = 0;
        }
        for (list<KeyFrame *>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++)
        {
            mVisEdges[(*lit)->mnId] = 0;
        }

        for (list<MapPoint *>::iterator lit = localMapPointList.begin(), lend = localMapPointList.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

            unsigned long id = pMP->mnId + iniMPid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);
            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            // Create visual constraints
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
                    continue;

                if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                {
                    const int leftIndex = get<0>(mit->second);

                    cv::KeyPoint kpUn;

                    // Monocular left observation
                    if (leftIndex != -1 && pKFi->mvuRight[leftIndex] < 0)
                    {
                        mVisEdges[pKFi->mnId]++;

                        kpUn = pKFi->mvKeysUn[leftIndex];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono *e = new EdgeMono(0);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                    // Stereo-observation
                    else if (leftIndex != -1) // Stereo observation
                    {
                        kpUn = pKFi->mvKeysUn[leftIndex];
                        mVisEdges[pKFi->mnId]++;

                        const float kp_ur = pKFi->mvuRight[leftIndex];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        EdgeStereo *e = new EdgeStereo(0);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);
                    }

                    // Monocular right observation
                    if (pKFi->mpCamera2)
                    {
                        int rightIndex = get<1>(mit->second);

                        if (rightIndex != -1)
                        {
                            rightIndex -= pKFi->NLeft;
                            mVisEdges[pKFi->mnId]++;

                            Eigen::Matrix<double, 2, 1> obs;
                            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                            obs << kp.pt.x, kp.pt.y;

                            EdgeMono *e = new EdgeMono(1);

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                            e->setMeasurement(obs);

                            // Add here uncerteinty
                            const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave] / unc2;
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberMono);

                            optimizer.addEdge(e);
                            vpEdgesMono.push_back(e);
                            vpEdgeKFMono.push_back(pKFi);
                            vpMapPointEdgeMono.push_back(pMP);
                        }
                    }
                }
            }
        }

        // cout << "Total map points: " << localMapPointList.size() << endl;
        for (map<int, int>::iterator mit = mVisEdges.begin(), mend = mVisEdges.end(); mit != mend; mit++)
        {
            assert(mit->second >= 3);
        }

        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        float err = optimizer.activeRobustChi2();
        optimizer.optimize(opt_it); // Originally to 2
        float err_end = optimizer.activeRobustChi2();
        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        // Mono
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeMono *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];
            bool bClose = pMP->mTrackDepth < 10.f;

            if (pMP->isBad())
                continue;

            if ((e->chi2() > chi2Mono2 && !bClose) || (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Stereo
        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            EdgeStereo *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > chi2Stereo2)
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex and erase outliers
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // TODO: Some convergence problems have been detected here
        if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) // bGN)
        {
            cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
            return;
        }

        if (!vToErase.empty())
        {
            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        for (list<KeyFrame *>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++)
            (*lit)->mnBAFixedForKF = 0;

        // Recover optimized data
        // Local temporal Keyframes
        N = vpOptimizableKFs.size();
        for (int i = 0; i < N; i++)
        {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);
            pKFi->mnBALocalForKF = 0;

            if (pKFi->bImu)
            {
                VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
                pKFi->SetVelocity(VV->estimate().cast<float>());
                VertexGyroBias *VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
                VertexAccBias *VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
                Vector6d b;
                b << VG->estimate(), VA->estimate();
                pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
            }
        }

        // Local visual KeyFrame
        for (list<KeyFrame *>::iterator it = lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it != itEnd; it++)
        {
            KeyFrame *pKFi = *it;
            VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);
            pKFi->mnBALocalForKF = 0;
        }

        // Points
        for (list<MapPoint *>::iterator lit = localMapPointList.begin(), lend = localMapPointList.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + iniMPid + 1));
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }

        pMap->IncreaseChangeIndex();
    }

    Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end)
    {
        // Goal
        // a  | ab | ac       a*  | 0 | ac*
        // ba | b  | bc  -->  0   | 0 | 0
        // ca | cb | c        ca* | 0 | c*

        // Size of block before block to marginalize
        const int a = start;
        // Size of block to marginalize
        const int b = end - start + 1;
        // Size of block after block to marginalize
        const int c = H.cols() - (end + 1);

        // Reorder as follows:
        // a  | ab | ac       a  | ac | ab
        // ba | b  | bc  -->  ca | c  | cb
        // ca | cb | c        ba | bc | b

        Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
        if (a > 0)
        {
            Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
            Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
            Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
        }
        if (a > 0 && c > 0)
        {
            Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
            Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
        }
        if (c > 0)
        {
            Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
            Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
            Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
        }
        Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

        // Perform marginalization (Schur complement)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
        for (int i = 0; i < b; ++i)
        {
            if (singularValues_inv(i) > 1e-6)
                singularValues_inv(i) = 1.0 / singularValues_inv(i);
            else
                singularValues_inv(i) = 0;
        }
        Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
        Hn.block(0, 0, a + c, a + c) = Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
        Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
        Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
        Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

        // Inverse reorder
        // a*  | ac* | 0       a*  | 0 | ac*
        // ca* | c*  | 0  -->  0   | 0 | 0
        // 0   | 0   | 0       ca* | 0 | c*
        Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
        if (a > 0)
        {
            res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
            res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
            res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
        }
        if (a > 0 && c > 0)
        {
            res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
            res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
        }
        if (c > 0)
        {
            res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
            res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
            res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
        }

        res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

        return res;
    }

    void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd &covInertial, bool bFixedVel, bool bGauss, float priorG, float priorA)
    {
        Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
        int its = 200;
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        if (priorG != 0.f)
            solver->setUserLambdaInit(1e3);

        optimizer.setAlgorithm(solver);

        // Set KeyFrame vertices (fixed poses and optimizable velocities)
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid)
                continue;
            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + (pKFi->mnId) + 1);
            if (bFixedVel)
                VV->setFixed(true);
            else
                VV->setFixed(false);

            optimizer.addVertex(VV);
        }

        // Biases
        VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
        VG->setId(maxKFid * 2 + 2);
        if (bFixedVel)
            VG->setFixed(true);
        else
            VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias *VA = new VertexAccBias(vpKFs.front());
        VA->setId(maxKFid * 2 + 3);
        if (bFixedVel)
            VA->setFixed(true);
        else
            VA->setFixed(false);

        optimizer.addVertex(VA);
        // prior acc bias
        Eigen::Vector3f bprior;
        bprior.setZero();

        EdgePriorAcc *epa = new EdgePriorAcc(bprior);
        epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
        double infoPriorA = priorA;
        epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);
        EdgePriorGyro *epg = new EdgePriorGyro(bprior);
        epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
        double infoPriorG = priorG;
        epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);

        // Gravity and scale
        VertexGDir *VGDir = new VertexGDir(Rwg);
        VGDir->setId(maxKFid * 2 + 4);
        VGDir->setFixed(false);
        optimizer.addVertex(VGDir);
        VertexScale *VS = new VertexScale(scale);
        VS->setId(maxKFid * 2 + 5);
        VS->setFixed(!bMono); // Fixed for stereo case
        optimizer.addVertex(VS);

        // Graph edges
        // IMU links with gravity and scale
        vector<EdgeInertialGS *> vpei;
        vpei.reserve(vpKFs.size());
        vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
        vppUsedKF.reserve(vpKFs.size());

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid)
            {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                    continue;
                if (!pKFi->mpImuPreintegrated)
                    std::cout << "Not preintegrated measurement" << std::endl;

                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
                g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid * 2 + 2);
                g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
                g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
                g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
                if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
                {
                    cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS << endl;

                    continue;
                }
                EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
                ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDir));
                ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VS));

                vpei.push_back(ei);

                vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
                optimizer.addEdge(ei);
            }
        }

        // Compute error for different scales
        std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();

        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(its);

        scale = VS->estimate();

        // Recover optimized data
        // Biases
        VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid * 2 + 2));
        VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid * 2 + 3));
        Vector6d vb;
        vb << VG->estimate(), VA->estimate();
        bg << VG->estimate();
        ba << VA->estimate();
        scale = VS->estimate();

        IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
        Rwg = VGDir->estimate().Rwg;

        // Keyframes velocities and biases
        const int N = vpKFs.size();
        for (size_t i = 0; i < N; i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid)
                continue;

            VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
            Eigen::Vector3d Vw = VV->estimate(); // Velocity is scaled after
            pKFi->SetVelocity(Vw.cast<float>());

            if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01)
            {
                pKFi->SetNewBias(b);
                if (pKFi->mpImuPreintegrated)
                    pKFi->mpImuPreintegrated->Reintegrate();
            }
            else
                pKFi->SetNewBias(b);
        }
    }

    void Optimizer::InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG, float priorA)
    {
        int its = 200; // Check number of iterations
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e3);

        optimizer.setAlgorithm(solver);

        // Set KeyFrame vertices (fixed poses and optimizable velocities)
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid)
                continue;
            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + (pKFi->mnId) + 1);
            VV->setFixed(false);

            optimizer.addVertex(VV);
        }

        // Biases
        VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
        VG->setId(maxKFid * 2 + 2);
        VG->setFixed(false);
        optimizer.addVertex(VG);

        VertexAccBias *VA = new VertexAccBias(vpKFs.front());
        VA->setId(maxKFid * 2 + 3);
        VA->setFixed(false);

        optimizer.addVertex(VA);
        // prior acc bias
        Eigen::Vector3f bprior;
        bprior.setZero();

        EdgePriorAcc *epa = new EdgePriorAcc(bprior);
        epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
        double infoPriorA = priorA;
        epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);
        EdgePriorGyro *epg = new EdgePriorGyro(bprior);
        epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
        double infoPriorG = priorG;
        epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);

        // Gravity and scale
        VertexGDir *VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
        VGDir->setId(maxKFid * 2 + 4);
        VGDir->setFixed(true);
        optimizer.addVertex(VGDir);
        VertexScale *VS = new VertexScale(1.0);
        VS->setId(maxKFid * 2 + 5);
        VS->setFixed(true); // Fixed since scale is obtained from already well initialized map
        optimizer.addVertex(VS);

        // Graph edges
        // IMU links with gravity and scale
        vector<EdgeInertialGS *> vpei;
        vpei.reserve(vpKFs.size());
        vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
        vppUsedKF.reserve(vpKFs.size());

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid)
            {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                    continue;

                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + (pKFi->mPrevKF->mnId) + 1);
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
                g2o::HyperGraph::Vertex *VG = optimizer.vertex(maxKFid * 2 + 2);
                g2o::HyperGraph::Vertex *VA = optimizer.vertex(maxKFid * 2 + 3);
                g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
                g2o::HyperGraph::Vertex *VS = optimizer.vertex(maxKFid * 2 + 5);
                if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
                {
                    cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", " << VA << ", " << VP2 << ", " << VV2 << ", " << VGDir << ", " << VS << endl;

                    continue;
                }
                EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
                ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDir));
                ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VS));

                vpei.push_back(ei);

                vppUsedKF.push_back(make_pair(pKFi->mPrevKF, pKFi));
                optimizer.addEdge(ei);
            }
        }

        // Compute error for different scales
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(its);

        // Recover optimized data
        // Biases
        VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid * 2 + 2));
        VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid * 2 + 3));
        Vector6d vb;
        vb << VG->estimate(), VA->estimate();
        bg << VG->estimate();
        ba << VA->estimate();

        IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

        // Keyframes velocities and biases
        const int N = vpKFs.size();
        for (size_t i = 0; i < N; i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid)
                continue;

            VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
            Eigen::Vector3d Vw = VV->estimate();
            pKFi->SetVelocity(Vw.cast<float>());

            if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01)
            {
                pKFi->SetNewBias(b);
                if (pKFi->mpImuPreintegrated)
                    pKFi->mpImuPreintegrated->Reintegrate();
            }
            else
                pKFi->SetNewBias(b);
        }
    }

    void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale)
    {
        int its = 10;
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        optimizer.setAlgorithm(solver);

        // Set KeyFrame vertices (all variables are fixed)
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid)
                continue;
            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 1 + (pKFi->mnId));
            VV->setFixed(true);
            optimizer.addVertex(VV);

            // Vertex of fixed biases
            VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
            VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias *VA = new VertexAccBias(vpKFs.front());
            VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }

        // Gravity and scale
        VertexGDir *VGDir = new VertexGDir(Rwg);
        VGDir->setId(4 * (maxKFid + 1));
        VGDir->setFixed(false);
        optimizer.addVertex(VGDir);
        VertexScale *VS = new VertexScale(scale);
        VS->setId(4 * (maxKFid + 1) + 1);
        VS->setFixed(false);
        optimizer.addVertex(VS);

        // Graph edges
        int count_edges = 0;
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid)
            {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                    continue;

                g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VV1 = optimizer.vertex((maxKFid + 1) + pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex *VV2 = optimizer.vertex((maxKFid + 1) + pKFi->mnId);
                g2o::HyperGraph::Vertex *VG = optimizer.vertex(2 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VA = optimizer.vertex(3 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(4 * (maxKFid + 1));
                g2o::HyperGraph::Vertex *VS = optimizer.vertex(4 * (maxKFid + 1) + 1);
                if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
                {
                    Verbose::PrintMess("Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) + ", " + to_string(VG->id()) + ", " + to_string(VA->id()) + ", " + to_string(VP2->id()) + ", " + to_string(VV2->id()) + ", " + to_string(VGDir->id()) + ", " + to_string(VS->id()), Verbose::VERBOSITY_NORMAL);

                    continue;
                }
                count_edges++;
                EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
                ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDir));
                ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VS));
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rk);
                rk->setDelta(1.f);
                optimizer.addEdge(ei);
            }
        }

        // Compute error for different scales
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        float err = optimizer.activeRobustChi2();
        optimizer.optimize(its);
        optimizer.computeActiveErrors();
        float err_end = optimizer.activeRobustChi2();
        // Recover optimized data
        scale = VS->estimate();
        Rwg = VGDir->estimate().Rwg;
    }

    void Optimizer::LoopClosureLocalBundleAdjustment(KeyFrame *pMainKF, vector<KeyFrame *> vpAdjustKF,
                                                     vector<KeyFrame *> vpFixedKF, bool *pbStopFlag)
    {
        // Variables
        vector<MapPoint *> vpMPs;
        set<KeyFrame *> spKeyFrameBA;
        long unsigned int maxKFid = 0;
        g2o::SparseOptimizer optimizer;

        // Define a linear solver to solve the linear system arising while optimization
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // Force stop flag
        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        // Get the current map
        Map *pCurrentMap = pMainKF->GetMap();

        // Set fixed KeyFrame vertices
        int numInsertedPoints = 0;
        for (KeyFrame *pKFi : vpFixedKF)
        {
            // Skip the KeyFrame if it is bad or is not in the current map
            if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            {
                Verbose::PrintMess("[Error in LoopClosureLocalBundleAdjustment] KeyFrame is bad or is not in the current map!", Verbose::VERBOSITY_NORMAL);
                continue;
            }

            // Set the local BA id for the KeyFrame
            pKFi->mnBALocalForMerge = pMainKF->mnId;

            // Create a new vertex for the KeyFrame
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;

            // Get the map points observed by the KeyFrame
            set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
            for (MapPoint *pMPi : spViewMPs)
                if (pMPi)
                    if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)
                        if (pMPi->mnBALocalForMerge != pMainKF->mnId)
                        {
                            // Add the map point to the list of optimizable map points
                            vpMPs.push_back(pMPi);
                            pMPi->mnBALocalForMerge = pMainKF->mnId;
                            numInsertedPoints++;
                        }

            spKeyFrameBA.insert(pKFi);
        }

        // Set non-fixed KeyFrame vertices
        set<KeyFrame *> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
        numInsertedPoints = 0;
        for (KeyFrame *pKFi : vpAdjustKF)
        {
            if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                continue;

            pKFi->mnBALocalForMerge = pMainKF->mnId;

            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKFi->mnId);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;

            set<MapPoint *> spViewMPs = pKFi->GetMapPoints();
            for (MapPoint *pMPi : spViewMPs)
                if (pMPi)
                    if (!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)
                        if (pMPi->mnBALocalForMerge != pMainKF->mnId)
                        {
                            vpMPs.push_back(pMPi);
                            pMPi->mnBALocalForMerge = pMainKF->mnId;
                            numInsertedPoints++;
                        }

            spKeyFrameBA.insert(pKFi);
        }

        const int nExpectedSize = (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

        vector<ORB_SLAM3::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);

        // Set MapPoint vertices
        map<KeyFrame *, int> mpObsKFs;
        map<KeyFrame *, int> mpObsFinalKFs;
        map<MapPoint *, int> mpObsMPs;
        for (unsigned int i = 0; i < vpMPs.size(); ++i)
        {
            MapPoint *pMPi = vpMPs[i];
            if (pMPi->isBad())
                continue;

            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
            const int id = pMPi->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, tuple<int, int>> observations = pMPi->GetObservations();
            int nEdges = 0;
            // SET EDGES
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
            {
                KeyFrame *pKF = mit->first;
                if (pKF->isBad() || pKF->mnId > maxKFid || pKF->mnBALocalForMerge != pMainKF->mnId || !pKF->GetMapPoint(get<0>(mit->second)))
                    continue;

                nEdges++;

                const cv::KeyPoint &kpUn = pKF->mvKeysUn[get<0>(mit->second)];

                if (pKF->mvuRight[get<0>(mit->second)] < 0) // Monocular
                {
                    mpObsMPs[pMPi]++;
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ *e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);

                    e->pCamera = pKF->mpCamera;

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKF);
                    vpMapPointEdgeMono.push_back(pMPi);

                    mpObsKFs[pKF]++;
                }
                else // RGBD or Stereo
                {
                    mpObsMPs[pMPi] += 2;
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
                    e->bf = pKF->mbf;

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKF);
                    vpMapPointEdgeStereo.push_back(pMPi);

                    mpObsKFs[pKF]++;
                }
            }
        }

        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        optimizer.initializeOptimization();
        optimizer.optimize(5);

        bool bDoMore = true;

        if (pbStopFlag)
            if (*pbStopFlag)
                bDoMore = false;

        map<unsigned long int, int> mWrongObsKF;
        if (bDoMore)
        {
            // Check inlier observations
            int badMonoMP = 0, badStereoMP = 0;
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 5.991 || !e->isDepthPositive())
                {
                    e->setLevel(1);
                    badMonoMP++;
                }
                e->setRobustKernel(0);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
                MapPoint *pMP = vpMapPointEdgeStereo[i];

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 7.815 || !e->isDepthPositive())
                {
                    e->setLevel(1);
                    badStereoMP++;
                }

                e->setRobustKernel(0);
            }
            Verbose::PrintMess("[BA]: First optimization(Huber), there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP) + " stereo bad edges", Verbose::VERBOSITY_DEBUG);

            optimizer.initializeOptimization(0);
            optimizer.optimize(10);
        }

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
        set<MapPoint *> spErasedMPs;
        set<KeyFrame *> spErasedKFs;

        // Check inlier observations
        int badMonoMP = 0, badStereoMP = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
                mWrongObsKF[pKFi->mnId]++;
                badMonoMP++;

                spErasedMPs.insert(pMP);
                spErasedKFs.insert(pKFi);
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
                mWrongObsKF[pKFi->mnId]++;
                badStereoMP++;

                spErasedMPs.insert(pMP);
                spErasedKFs.insert(pKFi);
            }
        }

        Verbose::PrintMess("[BA]: Second optimization, there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP) + " sterero bad edges", Verbose::VERBOSITY_DEBUG);

        // Get Map Mutex
        unique_lock<mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

        if (!vToErase.empty())
        {
            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }
        for (unsigned int i = 0; i < vpMPs.size(); ++i)
        {
            MapPoint *pMPi = vpMPs[i];
            if (pMPi->isBad())
                continue;

            const map<KeyFrame *, tuple<int, int>> observations = pMPi->GetObservations();
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
            {
                KeyFrame *pKF = mit->first;
                if (pKF->isBad() || pKF->mnId > maxKFid || pKF->mnBALocalForKF != pMainKF->mnId || !pKF->GetMapPoint(get<0>(mit->second)))
                    continue;

                if (pKF->mvuRight[get<0>(mit->second)] < 0) // Monocular
                {
                    mpObsFinalKFs[pKF]++;
                }
                else // RGBD or Stereo
                {
                    mpObsFinalKFs[pKF]++;
                }
            }
        }

        // Recover optimized data
        // Keyframes
        for (KeyFrame *pKFi : vpAdjustKF)
        {
            if (pKFi->isBad())
                continue;

            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());

            int numMonoBadPoints = 0, numMonoOptPoints = 0;
            int numStereoBadPoints = 0, numStereoOptPoints = 0;
            vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;
            vector<MapPoint *> vpMonoMPsBad, vpStereoMPsBad;

            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                ORB_SLAM3::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];
                KeyFrame *pKFedge = vpEdgeKFMono[i];

                if (pKFi != pKFedge)
                {
                    continue;
                }

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 5.991 || !e->isDepthPositive())
                {
                    numMonoBadPoints++;
                    vpMonoMPsBad.push_back(pMP);
                }
                else
                {
                    numMonoOptPoints++;
                    vpMonoMPsOpt.push_back(pMP);
                }
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
                MapPoint *pMP = vpMapPointEdgeStereo[i];
                KeyFrame *pKFedge = vpEdgeKFMono[i];

                if (pKFi != pKFedge)
                {
                    continue;
                }

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 7.815 || !e->isDepthPositive())
                {
                    numStereoBadPoints++;
                    vpStereoMPsBad.push_back(pMP);
                }
                else
                {
                    numStereoOptPoints++;
                    vpStereoMPsOpt.push_back(pMP);
                }
            }

            pKFi->SetPose(Tiw);
        }

        // Points
        for (MapPoint *pMPi : vpMPs)
        {
            if (pMPi->isBad())
                continue;

            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMPi->mnId + maxKFid + 1));
            pMPi->SetWorldPos(vPoint->estimate().cast<float>());
            pMPi->UpdateNormalAndDepth();
        }
    }

    void Optimizer::MergeInertialBA(KeyFrame *pCurrKF, KeyFrame *pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses)
    {
        const int Nd = 6;
        const unsigned long maxKFid = pCurrKF->mnId;

        vector<KeyFrame *> vpOptimizableKFs;
        vpOptimizableKFs.reserve(2 * Nd);

        // For cov KFS, inertial parameters are not optimized
        const int maxCovKF = 30;
        vector<KeyFrame *> vpOptimizableCovKFs;
        vpOptimizableCovKFs.reserve(maxCovKF);

        // Add sliding window for current KF
        vpOptimizableKFs.push_back(pCurrKF);
        pCurrKF->mnBALocalForKF = pCurrKF->mnId;
        for (int i = 1; i < Nd; i++)
        {
            if (vpOptimizableKFs.back()->mPrevKF)
            {
                vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
                vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
            }
            else
                break;
        }

        list<KeyFrame *> lFixedKeyFrames;
        if (vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
        }
        else
        {
            vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
            vpOptimizableKFs.pop_back();
        }

        // Add temporal neighbours to merge KF (previous and next KFs)
        vpOptimizableKFs.push_back(pMergeKF);
        pMergeKF->mnBALocalForKF = pCurrKF->mnId;

        // Previous KFs
        for (int i = 1; i < (Nd / 2); i++)
        {
            if (vpOptimizableKFs.back()->mPrevKF)
            {
                vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
                vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
            }
            else
                break;
        }

        // We fix just once the old map
        if (vpOptimizableKFs.back()->mPrevKF)
        {
            lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
        }
        else
        {
            vpOptimizableKFs.back()->mnBALocalForKF = 0;
            vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
            lFixedKeyFrames.push_back(vpOptimizableKFs.back());
            vpOptimizableKFs.pop_back();
        }

        // Next KFs
        if (pMergeKF->mNextKF)
        {
            vpOptimizableKFs.push_back(pMergeKF->mNextKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }

        while (vpOptimizableKFs.size() < (2 * Nd))
        {
            if (vpOptimizableKFs.back()->mNextKF)
            {
                vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
                vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
            }
            else
                break;
        }

        int N = vpOptimizableKFs.size();

        // Optimizable points seen by optimizable keyframes
        list<MapPoint *> localMapPointList;
        map<MapPoint *, int> mLocalObs;
        for (int i = 0; i < N; i++)
        {
            vector<MapPoint *> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                // Using mnBALocalForKF we avoid redundance here, one MP can not be added several times to localMapPointList
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->mnBALocalForKF != pCurrKF->mnId)
                        {
                            mLocalObs[pMP] = 1;
                            localMapPointList.push_back(pMP);
                            pMP->mnBALocalForKF = pCurrKF->mnId;
                        }
                        else
                        {
                            mLocalObs[pMP]++;
                        }
            }
        }

        std::vector<std::pair<MapPoint *, int>> pairs;
        pairs.reserve(mLocalObs.size());
        for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
            pairs.push_back(*itr);
        sort(pairs.begin(), pairs.end(), sortByVal);

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        int i = 0;
        for (vector<pair<MapPoint *, int>>::iterator lit = pairs.begin(), lend = pairs.end(); lit != lend; lit++, i++)
        {
            map<KeyFrame *, tuple<int, int>> observations = lit->first->GetObservations();
            if (i >= maxCovKF)
                break;
            for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (pKFi->mnBALocalForKF != pCurrKF->mnId && pKFi->mnBAFixedForKF != pCurrKF->mnId) // If optimizable or already included...
                {
                    pKFi->mnBALocalForKF = pCurrKF->mnId;
                    if (!pKFi->isBad())
                    {
                        vpOptimizableCovKFs.push_back(pKFi);
                        break;
                    }
                }
            }
        }

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        solver->setUserLambdaInit(1e3);

        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // Set Local KeyFrame vertices
        N = vpOptimizableKFs.size();
        for (int i = 0; i < N; i++)
        {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(false);
            optimizer.addVertex(VP);

            if (pKFi->bImu)
            {
                VertexVelocity *VV = new VertexVelocity(pKFi);
                VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
                VV->setFixed(false);
                optimizer.addVertex(VV);
                VertexGyroBias *VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                VG->setFixed(false);
                optimizer.addVertex(VG);
                VertexAccBias *VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                VA->setFixed(false);
                optimizer.addVertex(VA);
            }
        }

        // Set Local cov keyframes vertices
        int Ncov = vpOptimizableCovKFs.size();
        for (int i = 0; i < Ncov; i++)
        {
            KeyFrame *pKFi = vpOptimizableCovKFs[i];

            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(false);
            optimizer.addVertex(VP);

            if (pKFi->bImu)
            {
                VertexVelocity *VV = new VertexVelocity(pKFi);
                VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
                VV->setFixed(false);
                optimizer.addVertex(VV);
                VertexGyroBias *VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                VG->setFixed(false);
                optimizer.addVertex(VG);
                VertexAccBias *VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                VA->setFixed(false);
                optimizer.addVertex(VA);
            }
        }

        // Set Fixed KeyFrame vertices
        for (list<KeyFrame *>::iterator lit = lFixedKeyFrames.begin(), lend = lFixedKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            if (pKFi->bImu)
            {
                VertexVelocity *VV = new VertexVelocity(pKFi);
                VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
                VV->setFixed(true);
                optimizer.addVertex(VV);
                VertexGyroBias *VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                VG->setFixed(true);
                optimizer.addVertex(VG);
                VertexAccBias *VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                VA->setFixed(true);
                optimizer.addVertex(VA);
            }
        }

        // Create intertial constraints
        vector<EdgeInertial *> vei(N, (EdgeInertial *)NULL);
        vector<EdgeGyroRW *> vegr(N, (EdgeGyroRW *)NULL);
        vector<EdgeAccRW *> vear(N, (EdgeAccRW *)NULL);
        for (int i = 0; i < N; i++)
        {
            // cout << "inserting inertial edge " << i << endl;
            KeyFrame *pKFi = vpOptimizableKFs[i];

            if (!pKFi->mPrevKF)
            {
                Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!", Verbose::VERBOSITY_NORMAL);
                continue;
            }
            if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
            {
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
                g2o::HyperGraph::Vertex *VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
                g2o::HyperGraph::Vertex *VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex *VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
                g2o::HyperGraph::Vertex *VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
                g2o::HyperGraph::Vertex *VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

                if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
                {
                    cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", " << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", " << VA2 << endl;
                    continue;
                }

                vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

                vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
                vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
                vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

                // TODO Uncomment
                g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));
                optimizer.addEdge(vei[i]);

                vegr[i] = new EdgeGyroRW();
                vegr[i]->setVertex(0, VG1);
                vegr[i]->setVertex(1, VG2);
                Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
                vegr[i]->setInformation(InfoG);
                optimizer.addEdge(vegr[i]);

                vear[i] = new EdgeAccRW();
                vear[i]->setVertex(0, VA1);
                vear[i]->setVertex(1, VA2);
                Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
                vear[i]->setInformation(InfoA);
                optimizer.addEdge(vear[i]);
            }
            else
                Verbose::PrintMess("ERROR building inertial edge", Verbose::VERBOSITY_NORMAL);
        }

        Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);

        // Set MapPoint vertices
        const int nExpectedSize = (N + Ncov + lFixedKeyFrames.size()) * localMapPointList.size();

        // Mono
        vector<EdgeMono *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        // Stereo
        vector<EdgeStereo *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuberMono = sqrt(5.991);
        const float chi2Mono2 = 5.991;
        const float thHuberStereo = sqrt(7.815);
        const float chi2Stereo2 = 7.815;

        const unsigned long iniMPid = maxKFid * 5;

        for (list<MapPoint *>::iterator lit = localMapPointList.begin(), lend = localMapPointList.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            if (!pMP)
                continue;

            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

            unsigned long id = pMP->mnId + iniMPid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            // Create visual constraints
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi)
                    continue;

                if ((pKFi->mnBALocalForKF != pCurrKF->mnId) && (pKFi->mnBAFixedForKF != pCurrKF->mnId))
                    continue;

                if (pKFi->mnId > maxKFid)
                {
                    continue;
                }

                if (optimizer.vertex(id) == NULL || optimizer.vertex(pKFi->mnId) == NULL)
                    continue;

                if (!pKFi->isBad())
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[get<0>(mit->second)];

                    if (pKFi->mvuRight[get<0>(mit->second)] < 0) // Monocular observation
                    {
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono *e = new EdgeMono();
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);
                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                    else // stereo observation
                    {
                        const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        EdgeStereo *e = new EdgeStereo();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);
                    }
                }
            }
        }

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        optimizer.initializeOptimization();
        optimizer.optimize(8);

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        // Mono
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeMono *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > chi2Mono2)
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Stereo
        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            EdgeStereo *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > chi2Stereo2)
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex and erase outliers
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        if (!vToErase.empty())
        {
            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // Recover optimized data
        // Keyframes
        for (int i = 0; i < N; i++)
        {
            KeyFrame *pKFi = vpOptimizableKFs[i];

            VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);

            Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            corrPoses[pKFi] = g2oSiw;

            if (pKFi->bImu)
            {
                VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
                pKFi->SetVelocity(VV->estimate().cast<float>());
                VertexGyroBias *VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
                VertexAccBias *VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
                Vector6d b;
                b << VG->estimate(), VA->estimate();
                pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
            }
        }

        for (int i = 0; i < Ncov; i++)
        {
            KeyFrame *pKFi = vpOptimizableCovKFs[i];

            VertexPose *VP = static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);

            Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            corrPoses[pKFi] = g2oSiw;

            if (pKFi->bImu)
            {
                VertexVelocity *VV = static_cast<VertexVelocity *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
                pKFi->SetVelocity(VV->estimate().cast<float>());
                VertexGyroBias *VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
                VertexAccBias *VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
                Vector6d b;
                b << VG->estimate(), VA->estimate();
                pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
            }
        }

        // Points
        for (list<MapPoint *>::iterator lit = localMapPointList.begin(), lend = localMapPointList.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + iniMPid + 1));
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }

        pMap->IncreaseChangeIndex();
    }

    int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
    {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        optimizer.setVerbose(false);
        optimizer.setAlgorithm(solver);

        int nInitialMonoCorrespondences = 0;
        int nInitialStereoCorrespondences = 0;
        int nInitialCorrespondences = 0;

        // Set Frame vertex
        VertexPose *VP = new VertexPose(pFrame);
        VP->setId(0);
        VP->setFixed(false);
        optimizer.addVertex(VP);
        VertexVelocity *VV = new VertexVelocity(pFrame);
        VV->setId(1);
        VV->setFixed(false);
        optimizer.addVertex(VV);
        VertexGyroBias *VG = new VertexGyroBias(pFrame);
        VG->setId(2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias *VA = new VertexAccBias(pFrame);
        VA->setId(3);
        VA->setFixed(false);
        optimizer.addVertex(VA);

        // Set MapPoint vertices
        const int N = pFrame->N;
        const int Nleft = pFrame->Nleft;
        const bool bRight = (Nleft != -1);

        vector<EdgeMonoOnlyPose *> vpEdgesMono;
        vector<EdgeStereoOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeMono;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesMono.reserve(N);
        vpEdgesStereo.reserve(N);
        vnIndexEdgeMono.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP)
                {
                    cv::KeyPoint kpUn;

                    // Left monocular observation
                    if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft)
                    {
                        if (i < Nleft) // pair left-right
                            kpUn = pFrame->mvKeys[i];
                        else
                            kpUn = pFrame->mvKeysUn[i];

                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                    // Stereo observation
                    else if (!bRight)
                    {
                        nInitialStereoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysUn[i];
                        const float kp_ur = pFrame->mvuRight[i];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        EdgeStereoOnlyPose *e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                        const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }

                    // Right monocular observation
                    if (bRight && i >= Nleft)
                    {
                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysRight[i - Nleft];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                }
            }
        }
        nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

        KeyFrame *pKF = pFrame->mpLastKeyFrame;
        VertexPose *VPk = new VertexPose(pKF);
        VPk->setId(4);
        VPk->setFixed(true);
        optimizer.addVertex(VPk);
        VertexVelocity *VVk = new VertexVelocity(pKF);
        VVk->setId(5);
        VVk->setFixed(true);
        optimizer.addVertex(VVk);
        VertexGyroBias *VGk = new VertexGyroBias(pKF);
        VGk->setId(6);
        VGk->setFixed(true);
        optimizer.addVertex(VGk);
        VertexAccBias *VAk = new VertexAccBias(pKF);
        VAk->setId(7);
        VAk->setFixed(true);
        optimizer.addVertex(VAk);

        EdgeInertial *ei = new EdgeInertial(pFrame->mpImuPreintegrated);

        ei->setVertex(0, VPk);
        ei->setVertex(1, VVk);
        ei->setVertex(2, VGk);
        ei->setVertex(3, VAk);
        ei->setVertex(4, VP);
        ei->setVertex(5, VV);
        optimizer.addEdge(ei);

        EdgeGyroRW *egr = new EdgeGyroRW();
        egr->setVertex(0, VGk);
        egr->setVertex(1, VG);
        Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        egr->setInformation(InfoG);
        optimizer.addEdge(egr);

        EdgeAccRW *ear = new EdgeAccRW();
        ear->setVertex(0, VAk);
        ear->setVertex(1, VA);
        Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        ear->setInformation(InfoA);
        optimizer.addEdge(ear);

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
        float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

        int its[4] = {10, 10, 10, 10};

        int nBad = 0;
        int nBadMono = 0;
        int nBadStereo = 0;
        int nInliersMono = 0;
        int nInliersStereo = 0;
        int nInliers = 0;
        for (size_t it = 0; it < 4; it++)
        {
            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);

            nBad = 0;
            nBadMono = 0;
            nBadStereo = 0;
            nInliers = 0;
            nInliersMono = 0;
            nInliersStereo = 0;
            float chi2close = 1.5 * chi2Mono[it];

            // For monocular observations
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                EdgeMonoOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();
                bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

                if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive())
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBadMono++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersMono++;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            // For stereo observations
            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                EdgeStereoOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1); // not included in next optimization
                    nBadStereo++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersStereo++;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            nInliers = nInliersMono + nInliersStereo;
            nBad = nBadMono + nBadStereo;

            if (optimizer.edges().size() < 10)
            {
                break;
            }
        }

        // If not too much tracks, recover not too bad points
        if ((nInliers < 30) && !bRecInit)
        {
            nBad = 0;
            const float chi2MonoOut = 18.f;
            const float chi2StereoOut = 24.f;
            EdgeMonoOnlyPose *e1;
            EdgeStereoOnlyPose *e2;
            for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++)
            {
                const size_t idx = vnIndexEdgeMono[i];
                e1 = vpEdgesMono[i];
                e1->computeError();
                if (e1->chi2() < chi2MonoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
            for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++)
            {
                const size_t idx = vnIndexEdgeStereo[i];
                e2 = vpEdgesStereo[i];
                e2->computeError();
                if (e2->chi2() < chi2StereoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
        }

        // Recover optimized pose, velocity and biases
        pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
        Vector6d b;
        b << VG->estimate(), VA->estimate();
        pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

        // Recover Hessian, marginalize keyFframe states and generate new prior for frame
        Eigen::Matrix<double, 15, 15> H;
        H.setZero();

        H.block<9, 9>(0, 0) += ei->GetHessian2();
        H.block<3, 3>(9, 9) += egr->GetHessian2();
        H.block<3, 3>(12, 12) += ear->GetHessian2();

        int tot_in = 0, tot_out = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeMonoOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (!pFrame->mvbOutlier[idx])
            {
                H.block<6, 6>(0, 0) += e->GetHessian();
                tot_in++;
            }
            else
                tot_out++;
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            EdgeStereoOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (!pFrame->mvbOutlier[idx])
            {
                H.block<6, 6>(0, 0) += e->GetHessian();
                tot_in++;
            }
            else
                tot_out++;
        }

        pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(), VA->estimate(), H);

        return nInitialCorrespondences - nBad;
    }

    int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
    {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        int nInitialMonoCorrespondences = 0;
        int nInitialStereoCorrespondences = 0;
        int nInitialCorrespondences = 0;

        // Set Current Frame vertex
        VertexPose *VP = new VertexPose(pFrame);
        VP->setId(0);
        VP->setFixed(false);
        optimizer.addVertex(VP);
        VertexVelocity *VV = new VertexVelocity(pFrame);
        VV->setId(1);
        VV->setFixed(false);
        optimizer.addVertex(VV);
        VertexGyroBias *VG = new VertexGyroBias(pFrame);
        VG->setId(2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias *VA = new VertexAccBias(pFrame);
        VA->setId(3);
        VA->setFixed(false);
        optimizer.addVertex(VA);

        // Set MapPoint vertices
        const int N = pFrame->N;
        const int Nleft = pFrame->Nleft;
        const bool bRight = (Nleft != -1);

        vector<EdgeMonoOnlyPose *> vpEdgesMono;
        vector<EdgeStereoOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeMono;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesMono.reserve(N);
        vpEdgesStereo.reserve(N);
        vnIndexEdgeMono.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP)
                {
                    cv::KeyPoint kpUn;
                    // Left monocular observation
                    if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft)
                    {
                        if (i < Nleft) // pair left-right
                            kpUn = pFrame->mvKeys[i];
                        else
                            kpUn = pFrame->mvKeysUn[i];

                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                    // Stereo observation
                    else if (!bRight)
                    {
                        nInitialStereoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysUn[i];
                        const float kp_ur = pFrame->mvuRight[i];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        EdgeStereoOnlyPose *e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                        const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }

                    // Right monocular observation
                    if (bRight && i >= Nleft)
                    {
                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysRight[i - Nleft];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                }
            }
        }

        nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

        // Set Previous Frame Vertex
        Frame *pFp = pFrame->mpPrevFrame;

        VertexPose *VPk = new VertexPose(pFp);
        VPk->setId(4);
        VPk->setFixed(false);
        optimizer.addVertex(VPk);
        VertexVelocity *VVk = new VertexVelocity(pFp);
        VVk->setId(5);
        VVk->setFixed(false);
        optimizer.addVertex(VVk);
        VertexGyroBias *VGk = new VertexGyroBias(pFp);
        VGk->setId(6);
        VGk->setFixed(false);
        optimizer.addVertex(VGk);
        VertexAccBias *VAk = new VertexAccBias(pFp);
        VAk->setId(7);
        VAk->setFixed(false);
        optimizer.addVertex(VAk);

        EdgeInertial *ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

        ei->setVertex(0, VPk);
        ei->setVertex(1, VVk);
        ei->setVertex(2, VGk);
        ei->setVertex(3, VAk);
        ei->setVertex(4, VP);
        ei->setVertex(5, VV);
        optimizer.addEdge(ei);

        EdgeGyroRW *egr = new EdgeGyroRW();
        egr->setVertex(0, VGk);
        egr->setVertex(1, VG);
        Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        egr->setInformation(InfoG);
        optimizer.addEdge(egr);

        EdgeAccRW *ear = new EdgeAccRW();
        ear->setVertex(0, VAk);
        ear->setVertex(1, VA);
        Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        ear->setInformation(InfoA);
        optimizer.addEdge(ear);

        if (!pFp->mpcpi)
            Verbose::PrintMess("pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId), Verbose::VERBOSITY_NORMAL);

        EdgePriorPoseImu *ep = new EdgePriorPoseImu(pFp->mpcpi);

        ep->setVertex(0, VPk);
        ep->setVertex(1, VVk);
        ep->setVertex(2, VGk);
        ep->setVertex(3, VAk);
        g2o::RobustKernelHuber *rkp = new g2o::RobustKernelHuber;
        ep->setRobustKernel(rkp);
        rkp->setDelta(5);
        optimizer.addEdge(ep);

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
        const int its[4] = {10, 10, 10, 10};

        int nBad = 0;
        int nBadMono = 0;
        int nBadStereo = 0;
        int nInliersMono = 0;
        int nInliersStereo = 0;
        int nInliers = 0;
        for (size_t it = 0; it < 4; it++)
        {
            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);

            nBad = 0;
            nBadMono = 0;
            nBadStereo = 0;
            nInliers = 0;
            nInliersMono = 0;
            nInliersStereo = 0;
            float chi2close = 1.5 * chi2Mono[it];

            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                EdgeMonoOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];
                bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive())
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBadMono++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersMono++;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                EdgeStereoOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBadStereo++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersStereo++;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            nInliers = nInliersMono + nInliersStereo;
            nBad = nBadMono + nBadStereo;

            if (optimizer.edges().size() < 10)
            {
                break;
            }
        }

        if ((nInliers < 30) && !bRecInit)
        {
            nBad = 0;
            const float chi2MonoOut = 18.f;
            const float chi2StereoOut = 24.f;
            EdgeMonoOnlyPose *e1;
            EdgeStereoOnlyPose *e2;
            for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++)
            {
                const size_t idx = vnIndexEdgeMono[i];
                e1 = vpEdgesMono[i];
                e1->computeError();
                if (e1->chi2() < chi2MonoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
            for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++)
            {
                const size_t idx = vnIndexEdgeStereo[i];
                e2 = vpEdgesStereo[i];
                e2->computeError();
                if (e2->chi2() < chi2StereoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
        }

        nInliers = nInliersMono + nInliersStereo;

        // Recover optimized pose, velocity and biases
        pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
        Vector6d b;
        b << VG->estimate(), VA->estimate();
        pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

        // Recover Hessian, marginalize previous frame states and generate new prior for frame
        Eigen::Matrix<double, 30, 30> H;
        H.setZero();

        H.block<24, 24>(0, 0) += ei->GetHessian();

        Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
        H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
        H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
        H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
        H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

        Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
        H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
        H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
        H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
        H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

        H.block<15, 15>(0, 0) += ep->GetHessian();

        int tot_in = 0, tot_out = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeMonoOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (!pFrame->mvbOutlier[idx])
            {
                H.block<6, 6>(15, 15) += e->GetHessian();
                tot_in++;
            }
            else
                tot_out++;
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            EdgeStereoOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (!pFrame->mvbOutlier[idx])
            {
                H.block<6, 6>(15, 15) += e->GetHessian();
                tot_in++;
            }
            else
                tot_out++;
        }

        H = Marginalize(H, 0, 14);

        pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(), VA->estimate(), H.block<15, 15>(15, 15));
        delete pFp->mpcpi;
        pFp->mpcpi = NULL;

        return nInitialCorrespondences - nBad;
    }

    void Optimizer::OptimizeEssentialGraph4DoF(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                               const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                               const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                               const map<KeyFrame *, set<KeyFrame *>> &LoopConnections)
    {
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<4, 4>> BlockSolver_4_4;

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        g2o::BlockSolverX::LinearSolverType *linearSolver =
            new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        optimizer.setAlgorithm(solver);

        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

        const unsigned int nMaxKFid = pMap->GetMaxKFid();

        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);

        vector<VertexPose4DoF *> vpVertices(nMaxKFid + 1);

        const int minFeat = 100;
        // Set KeyFrame vertices
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;

            VertexPose4DoF *V4DoF;

            const int nIDi = pKF->mnId;

            LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

            if (it != CorrectedSim3.end())
            {
                vScw[nIDi] = it->second;
                const g2o::Sim3 Swc = it->second.inverse();
                Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
                Eigen::Vector3d twc = Swc.translation();
                V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
            }
            else
            {
                Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
                g2o::Sim3 Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

                vScw[nIDi] = Siw;
                V4DoF = new VertexPose4DoF(pKF);
            }

            if (pKF == pLoopKF)
                V4DoF->setFixed(true);

            V4DoF->setId(nIDi);
            V4DoF->setMarginalized(false);

            optimizer.addVertex(V4DoF);
            vpVertices[nIDi] = V4DoF;
        }
        set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

        // Edge used in posegraph has still 6Dof, even if updates of camera poses are just in 4DoF
        Eigen::Matrix<double, 6, 6> matLambda = Eigen::Matrix<double, 6, 6>::Identity();
        matLambda(0, 0) = 1e3;
        matLambda(1, 1) = 1e3;
        matLambda(0, 0) = 1e3;

        // Set Loop edges
        Edge4DoF *e_loop;
        for (map<KeyFrame *, set<KeyFrame *>>::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            const long unsigned int nIDi = pKF->mnId;
            const set<KeyFrame *> &spConnections = mit->second;
            const g2o::Sim3 Siw = vScw[nIDi];

            for (set<KeyFrame *>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++)
            {
                const long unsigned int nIDj = (*sit)->mnId;
                if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat)
                    continue;

                const g2o::Sim3 Sjw = vScw[nIDj];
                const g2o::Sim3 Sij = Siw * Sjw.inverse();
                Eigen::Matrix4d Tij;
                Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
                Tij.block<3, 1>(0, 3) = Sij.translation();
                Tij(3, 3) = 1.;

                Edge4DoF *e = new Edge4DoF(Tij);
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));

                e->information() = matLambda;
                e_loop = e;
                optimizer.addEdge(e);

                sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
            }
        }

        // 1. Set normal edges
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];

            const int nIDi = pKF->mnId;

            g2o::Sim3 Siw;

            // Use noncorrected poses for posegraph edges
            LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

            if (iti != NonCorrectedSim3.end())
                Siw = iti->second;
            else
                Siw = vScw[nIDi];

            // 1.1.0 Spanning tree edge
            KeyFrame *pParentKF = static_cast<KeyFrame *>(NULL);
            if (pParentKF)
            {
                int nIDj = pParentKF->mnId;

                g2o::Sim3 Swj;

                LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

                if (itj != NonCorrectedSim3.end())
                    Swj = (itj->second).inverse();
                else
                    Swj = vScw[nIDj].inverse();

                g2o::Sim3 Sij = Siw * Swj;
                Eigen::Matrix4d Tij;
                Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
                Tij.block<3, 1>(0, 3) = Sij.translation();
                Tij(3, 3) = 1.;

                Edge4DoF *e = new Edge4DoF(Tij);
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // 1.1.1 Inertial edges
            KeyFrame *prevKF = pKF->mPrevKF;
            if (prevKF)
            {
                int nIDj = prevKF->mnId;

                g2o::Sim3 Swj;

                LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(prevKF);

                if (itj != NonCorrectedSim3.end())
                    Swj = (itj->second).inverse();
                else
                    Swj = vScw[nIDj].inverse();

                g2o::Sim3 Sij = Siw * Swj;
                Eigen::Matrix4d Tij;
                Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
                Tij.block<3, 1>(0, 3) = Sij.translation();
                Tij(3, 3) = 1.;

                Edge4DoF *e = new Edge4DoF(Tij);
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // 1.2 Loop edges
            const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
            for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++)
            {
                KeyFrame *pLKF = *sit;
                if (pLKF->mnId < pKF->mnId)
                {
                    g2o::Sim3 Swl;

                    LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                    if (itl != NonCorrectedSim3.end())
                        Swl = itl->second.inverse();
                    else
                        Swl = vScw[pLKF->mnId].inverse();

                    g2o::Sim3 Sil = Siw * Swl;
                    Eigen::Matrix4d Til;
                    Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
                    Til.block<3, 1>(0, 3) = Sil.translation();
                    Til(3, 3) = 1.;

                    Edge4DoF *e = new Edge4DoF(Til);
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
                    e->information() = matLambda;
                    optimizer.addEdge(e);
                }
            }

            // 1.3 Covisibility graph edges
            const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
            for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++)
            {
                KeyFrame *pKFn = *vit;
                if (pKFn && pKFn != pParentKF && pKFn != prevKF && pKFn != pKF->mNextKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
                {
                    if (!pKFn->isBad() && pKFn->mnId < pKF->mnId)
                    {
                        if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId))))
                            continue;

                        g2o::Sim3 Swn;

                        LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                        if (itn != NonCorrectedSim3.end())
                            Swn = itn->second.inverse();
                        else
                            Swn = vScw[pKFn->mnId].inverse();

                        g2o::Sim3 Sin = Siw * Swn;
                        Eigen::Matrix4d Tin;
                        Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
                        Tin.block<3, 1>(0, 3) = Sin.translation();
                        Tin(3, 3) = 1.;
                        Edge4DoF *e = new Edge4DoF(Tin);
                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
                        e->information() = matLambda;
                        optimizer.addEdge(e);
                    }
                }
            }
        }

        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        optimizer.optimize(20);

        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            const int nIDi = pKFi->mnId;

            VertexPose4DoF *Vi = static_cast<VertexPose4DoF *>(optimizer.vertex(nIDi));
            Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
            Eigen::Vector3d ti = Vi->estimate().tcw[0];

            g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();

            Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
            pKFi->SetPose(Tiw.cast<float>());
        }

        // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMPs[i];

            if (pMP->isBad())
                continue;

            int nIDr;

            KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;

            g2o::Sim3 Srw = vScw[nIDr];
            g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

            Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->GetWorldPos().cast<double>();
            Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
            pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

            pMP->UpdateNormalAndDepth();
        }
        pMap->IncreaseChangeIndex();
    }

} // namespace ORB_SLAM
