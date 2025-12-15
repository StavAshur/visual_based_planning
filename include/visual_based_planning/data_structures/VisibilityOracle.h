#pragma once

#include <vector>
#include <string>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <functional>
#include <random>

#include "../common/Types.h"

// Eigen for 3D Math
#include <Eigen/Geometry>
#include <Eigen/Dense>

// Boost Geometry for robust geometric types
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>

namespace bg = boost::geometry;

// Define Types to match framework
typedef bg::model::point<double, 3, bg::cs::cartesian> Point3D;
typedef bg::model::box<Point3D> Box3D;
typedef bg::model::segment<Point3D> Segment3D;

namespace visual_planner {

class VisibilityOracle {
public:
    // 1. Tool Parameters Struct
    struct VisibilityToolParams {
        double beam_angle;  // Half-angle in radians (cutoff from Z-axis)
        double beam_length; // Max range in meters
    };

    // 2. Visibility Method Enum
    enum class VisibilityMethod {
        BF,     // Brute Force
        GPUBF,  // GPU Brute Force (Future)
        KDTree  // KD-Tree (Future)
    };

private:
    // Storage for Brute Force
    std::vector<Box3D> obstacles_;

    // Beam Visibility Members
    VisibilityToolParams tool_params_;
    VisibilityMethod method_; // Current method selection
    int num_samples_ = 100;
    std::function<Eigen::Isometry3d(const std::vector<double>&)> fk_solver_;
    std::mt19937 rng_; // Random number generator

public:
    VisibilityOracle() : rng_(std::random_device{}()), method_(VisibilityMethod::BF) {
        // Default params
        tool_params_ = {M_PI / 12.0, 1.0}; // 15 deg half-angle, 1m length
    }
    ~VisibilityOracle() = default;

    /**
     * @brief Adds an AABB obstacle to the oracle.
     */
    void addObstacle(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) {
        Box3D b(Point3D(min_x, min_y, min_z), Point3D(max_x, max_y, max_z));
        obstacles_.push_back(b);
    }

    // --- Configuration Setters ---

    void setVisibilityToolParams(VisibilityToolParams params) {
        tool_params_=params;
    }

    void setVisibilityToolParams(double beam_angle_rad, double beam_length_m) {
        tool_params_.beam_angle = beam_angle_rad;
        tool_params_.beam_length = beam_length_m;
    }

    void setNumSamples(int n) {
        num_samples_ = n;
    }

    void setFKSolver(std::function<Eigen::Isometry3d(const std::vector<double>&)> solver) {
        fk_solver_ = solver;
    }

    void setMethod(VisibilityMethod method) {
        method_ = method;
    }

    VisibilityMethod getMethod() const {
        return method_;
    }

    // --- Beam Visibility API ---

    /**
     * @brief Public wrapper to check if a point is illuminated by the tool.
     * 1. Single Point (Joints)
     */
    bool checkPointBeamVisibility(const std::vector<double>& joints, const std::vector<double>& point) {
        Eigen::Vector3d target(point[0], point[1], point[2]);
        return isPointInBeam(joints, target);
    }
    

    // 2. Single Point (Pose) - Helper for external samplers
    bool checkPointBeamVisibility(const Eigen::Isometry3d& sensor_pose, const Eigen::Vector3d& point) {
        return isPointInBeam(sensor_pose, point);
    }

    // 3. Point Set / Distribution (Pose) - Returns fraction visible [0..1]
    double checkVisibleFraction(const Eigen::Isometry3d& sensor_pose, const std::vector<Eigen::Vector3d>& points) {
        if (points.empty()) return 0.0;
        int visible_count = 0;
        for (const auto& p : points) {
            if (isPointInBeam(sensor_pose, p)) {
                visible_count++;
            }
        }
        return static_cast<double>(visible_count) / points.size();
    }




    /**
     * @brief Checks what fraction of a ball is illuminated by the beam (Pose Variant).
     * @param sensor_pose The pose of the sensor/tool
     * @param center Ball center (x,y,z)
     * @param radius Ball radius
     * @return double Fraction [0.0, 1.0]
     */
    double checkBallBeamVisibility(const Eigen::Isometry3d& sensor_pose, const Eigen::Vector3d center, double radius) {

        // --- DEBUG START ---
        // Eigen::Vector3d start = sensor_pose.translation();
        // Eigen::Vector3d z_dir = sensor_pose.linear().col(2); 
        // double len = (start - center).norm();
        // Eigen::Vector3d end = start + (z_dir * len);

        // ROS_WARN("[DEBUG] Beam Segment Check:\n  Start (Tool): [%.3f, %.3f, %.3f]\n  End (Z-proj): [%.3f, %.3f, %.3f]\n  Target Center: [%.3f, %.3f, %.3f]\n  Length: %.3f",
        //          start.x(), start.y(), start.z(),
        //          end.x(), end.y(), end.z(),
        //          center.x(), center.y(), center.z(),
        //          len);
        // --- DEBUG END ---

        int visible_count = 0;
        std::uniform_real_distribution<double> dist(-1.0, 1.0);

        for (int i = 0; i < num_samples_; ++i) {
            // Generate random point in unit sphere using rejection sampling
            Eigen::Vector3d offset;
            do {
                offset = Eigen::Vector3d(dist(rng_), dist(rng_), dist(rng_));
            } while (offset.squaredNorm() > 1.0);

            // Scale and shift
            Eigen::Vector3d sample = center + (offset * radius);

            // Check visibility using the Pose overload
            if (isPointInBeam(sensor_pose, sample)) {
                visible_count++;
            }
        }

        return static_cast<double>(visible_count) / num_samples_;
    }

    /**
     * @brief Checks what fraction of a ball is illuminated by the beam (Joint Space Variant).
     * @param joints Robot configuration
     * @param center Ball center (x,y,z)
     * @param radius Ball radius
     * @return double Fraction [0.0, 1.0]
     */
    double checkBallBeamVisibility(const std::vector<double>& joints, const Eigen::Vector3d center, double radius) {
        if (!fk_solver_) {
            throw std::runtime_error("VisibilityOracle: FK Solver not set.");
        }
        // Calculate Pose
        Eigen::Isometry3d pose = fk_solver_(joints);
        
        // Delegate to Pose variant
        return checkBallBeamVisibility(pose, center, radius);
    }

    /**
     * @brief Checks what fraction of a ball is illuminated by the beam (Source Point Variant).
     * Assumes the beam is oriented directly towards the ball center.
     * @param source The position of the light source (x,y,z)
     * @param center Ball center (x,y,z)
     * @param radius Ball radius
     * @return double Fraction [0.0, 1.0]
     */
    double checkBallBeamVisibility(const Eigen::Vector3d source, const Eigen::Vector3d center, double radius) {

        Eigen::Vector3d dir = center - source;
        double dist = dir.norm();

        // Handle singularity (Source is inside/at center)
        if (dist < 1e-4) {
            return 1.0; // Assume fully visible if coincident
        }

        // 1. Construct Look-At Rotation
        // Z-axis points from Source -> Center
        Eigen::Vector3d z_axis = dir / dist;
        
        // Handle Up vector singularity
        Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();
        if (std::abs(z_axis.dot(world_up)) > 0.99) {
            world_up = Eigen::Vector3d::UnitX();
        }

        Eigen::Vector3d x_axis = world_up.cross(z_axis).normalized();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

        Eigen::Matrix3d rotation;
        rotation.col(0) = x_axis;
        rotation.col(1) = y_axis;
        rotation.col(2) = z_axis;

        // 2. Construct Pose
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = source;
        pose.linear() = rotation;

        // 3. Delegate to Pose variant
        return checkBallBeamVisibility(pose, center, radius);
    }

    double checkBallBeamVisibility(const Eigen::Vector3d source, const Ball b) {

        return checkBallBeamVisibility(source, b.center, b.radius);
    }

    /**
     * @brief Checks if the segment between p1 and p2 is visible (not blocked).
     * Uses the method selected via setMethod() (Default: BF).
     */
    bool checkVisibility(const std::vector<double>& p1, const std::vector<double>& p2) {
        
        // Convert input vectors to Boost Segment
        Point3D pt1(p1[0], p1[1], p1[2]);
        Point3D pt2(p2[0], p2[1], p2[2]);
        Segment3D segment(pt1, pt2);

        switch (method_) {
            case VisibilityMethod::BF:
                return checkVisibilityBF(segment);
            case VisibilityMethod::GPUBF:
                return checkVisibilityGPUBF(segment);
            case VisibilityMethod::KDTree:
                return checkVisibilityKDTree(segment);
            default:
                throw std::runtime_error("Unknown visibility check method.");
        }
    }

    /**
     * @brief Eigen overload for checkVisibility
     */
    bool checkVisibility(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        std::vector<double> v1 = {p1.x(), p1.y(), p1.z()};
        std::vector<double> v2 = {p2.x(), p2.y(), p2.z()};
        return checkVisibility(v1, v2);
    }

    void clear() {
        obstacles_.clear();
    }

private:
    // ==============================================================================
    // Beam Logic (Private)
    // ==============================================================================

    // 2. Base function: Checks if point is in beam given the sensor pose
    bool isPointInBeam(const Eigen::Isometry3d& sensor_pose, const Eigen::Vector3d& target) {
        // Transform target to Sensor Frame
        Eigen::Vector3d local_target = sensor_pose.inverse() * target;

        double dist = local_target.norm();

        // 1. Check Range (Depth)
        // Assuming beam emanates along +Z axis
        if (local_target.z() < 0.0 || dist > tool_params_.beam_length) {
            return false;
        }

        // 2. Check Angle (Cone)
        // Vector from origin to point is just local_target.
        // The angle theta is between local_target and Z-axis (0,0,1).
        // cos(theta) = (v . z) / (|v| * |z|) = local_target.z() / local_target.norm()
        
        
        if (dist < 1e-6) return true; // Point is at the sensor origin

        double cos_theta = local_target.z() / dist;
        double cos_beam = std::cos(tool_params_.beam_angle);

        // Within cone if cos_theta >= cos_beam (since angle is smaller)
        // AND check for occlusion
        Eigen::Vector3d tool_position = sensor_pose.translation();
        return cos_theta >= cos_beam && checkVisibility(tool_position, target);
    }

    // Overload: Calculates FK then calls base function
    bool isPointInBeam(const std::vector<double>& joints, const Eigen::Vector3d& target) {
        if (!fk_solver_) {
            throw std::runtime_error("VisibilityOracle: FK Solver not set.");
        }
        Eigen::Isometry3d pose = fk_solver_(joints);
        return isPointInBeam(pose, target);
    }

    // ==============================================================================
    // 1. Brute Force Implementation
    // ==============================================================================
    bool checkVisibilityBF(const Segment3D& segment) {
        // Boost Geometry provides a robust 'intersects' function that handles
        // the Slab logic / separating axis theorem internally and efficiently.
        
        for (const auto& box : obstacles_) {
            if (bg::intersects(segment, box)) {
                // ROS_INFO("[VisOracle] Occlusion! Segment: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f) hit Box: min(%.2f, %.2f, %.2f) max(%.2f, %.2f, %.2f)",
                //     bg::get<0, 0>(segment), bg::get<0, 1>(segment), bg::get<0, 2>(segment),
                //     bg::get<1, 0>(segment), bg::get<1, 1>(segment), bg::get<1, 2>(segment),
                //     box.min_corner().get<0>(), box.min_corner().get<1>(), box.min_corner().get<2>(),
                //     box.max_corner().get<0>(), box.max_corner().get<1>(), box.max_corner().get<2>());
                return false; // Blocked
            }
        }
        return true; // Visible
    }

    // ==============================================================================
    // 2. Unimplemented Placeholders (As requested)
    // ==============================================================================
    bool checkVisibilityGPUBF(const Segment3D& segment) {
        throw std::runtime_error("GPUBF visibility check is not implemented.");
    }

    bool checkVisibilityKDTree(const Segment3D& segment) {
        throw std::runtime_error("KDTree visibility check is not implemented.");
    }
};

} // namespace visual_planner
