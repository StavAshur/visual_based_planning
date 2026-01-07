#pragma once

#include <vector>
#include <string>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <functional>
#include <random>

#include "../common/Types.h"
#include <visual_based_planning/data_structures/VisibilityOracleGPU.h>

// Eigen for 3D Math
#include <Eigen/Geometry>
#include <Eigen/Dense>

// CUDA Runtime for memory management functions in the header
#include <cuda_runtime.h> 

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

    enum class VisibilityMethod {
        BF,     // Brute Force (CPU)
        GPUBF,  // GPU Brute Force
        KDTree  // KD-Tree (Future)
    };

private:
    // Storage for Brute Force
    std::vector<Box3D> obstacles_;

    // --- GPU Members ---
    GPUBox* d_obstacles_ = nullptr; // Pointer to GPU memory
    bool gpu_dirty_ = false;        // Flag to indicate if CPU/GPU are out of sync
    size_t gpu_capacity_ = 0;       // Current number of allocated slots on GPU

    // Beam Visibility Members
    VisibilityToolParams tool_params_;
    VisibilityMethod method_; 
    int num_samples_ = 100;
    std::function<Eigen::Isometry3d(const std::vector<double>&)> fk_solver_;
    std::mt19937 rng_; 

public:
    VisibilityOracle() : rng_(std::random_device{}()), method_(VisibilityMethod::GPUBF) {
        tool_params_ = {M_PI / 12.0, 1.0};
    }

    ~VisibilityOracle() {
        freeGPUMemory();
    }

    void addObstacle(double min_x, double min_y, double min_z, double max_x, double max_y, double max_z) {
        Box3D b(Point3D(min_x, min_y, min_z), Point3D(max_x, max_y, max_z));
        obstacles_.push_back(b);
        gpu_dirty_ = true; // Mark that GPU needs an update
    }

    void clear() {
        obstacles_.clear();
        gpu_dirty_ = true;
    }

    // --- Configuration Setters ---

    void setVisibilityToolParams(VisibilityToolParams params) { tool_params_=params; }
    void setVisibilityToolParams(double beam_angle_rad, double beam_length_m) {
        tool_params_.beam_angle = beam_angle_rad;
        tool_params_.beam_length = beam_length_m;
    }
    void setNumSamples(int n) { num_samples_ = n; }
    void setFKSolver(std::function<Eigen::Isometry3d(const std::vector<double>&)> solver) {
        fk_solver_ = solver;
    }
    void setMethod(VisibilityMethod method) { method_ = method; }
    VisibilityMethod getMethod() const { return method_; }

    // --- Beam Visibility API ---

    bool checkPointBeamVisibility(const std::vector<double>& joints, const std::vector<double>& point) {
        Eigen::Vector3d target(point[0], point[1], point[2]);
        return isPointInBeam(joints, target);
    }
    
    bool checkPointBeamVisibility(const Eigen::Isometry3d& sensor_pose, const Eigen::Vector3d& point) {
        return isPointInBeam(sensor_pose, point);
    }

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
     * optimized for GPU batching.
     */
    double checkBallBeamVisibility(const Eigen::Isometry3d& sensor_pose, const Eigen::Vector3d center, double radius) {
        // 1. Generate all samples first
        std::vector<Eigen::Vector3d> samples;
        samples.reserve(num_samples_);
        
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        for (int i = 0; i < num_samples_; ++i) {
            Eigen::Vector3d offset;
            do {
                offset = Eigen::Vector3d(dist(rng_), dist(rng_), dist(rng_));
            } while (offset.squaredNorm() > 1.0);
            samples.push_back(center + (offset * radius));
        }

        int visible_count = 0;

        // 2. Branch based on method
        if (method_ == VisibilityMethod::GPUBF) {
            // --- GPU Batch Path ---
            std::vector<Eigen::Vector3d> ray_starts;
            std::vector<Eigen::Vector3d> ray_ends;
            Eigen::Vector3d origin = sensor_pose.translation();

            // Filter: Only raycast points that are actually inside the FOV
            for (const auto& s : samples) {
                if (isPointInFOV(sensor_pose, s)) {
                    ray_starts.push_back(origin);
                    ray_ends.push_back(s);
                }
            }

            // If no points are in FOV, visibility is 0
            if (ray_starts.empty()) return 0.0;

            // Execute Batch Collision Check
            std::vector<bool> results;
            checkBatchVisibility(ray_starts, ray_ends, results);

            // Tally results
            for (bool visible : results) {
                if (visible) visible_count++;
            }

        } else {
            // --- CPU / Standard Path ---
            for (const auto& s : samples) {
                // isPointInBeam handles FOV + Collision internally
                if (isPointInBeam(sensor_pose, s)) {
                    visible_count++;
                }
            }
        }

        return static_cast<double>(visible_count) / num_samples_;
    }


    double checkBallBeamVisibility(const std::vector<double>& joints, const Eigen::Vector3d center, double radius) {
        if (!fk_solver_) throw std::runtime_error("VisibilityOracle: FK Solver not set.");
        return checkBallBeamVisibility(fk_solver_(joints), center, radius);
    }

    double checkBallBeamVisibility(const Eigen::Vector3d source, const Eigen::Vector3d center, double radius) {
        Eigen::Vector3d dir = center - source;
        double dist = dir.norm();
        if (dist < 1e-4) return 1.0;

        Eigen::Vector3d z_axis = dir / dist;
        Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();
        if (std::abs(z_axis.dot(world_up)) > 0.99) world_up = Eigen::Vector3d::UnitX();

        Eigen::Vector3d x_axis = world_up.cross(z_axis).normalized();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

        Eigen::Matrix3d rotation;
        rotation.col(0) = x_axis; rotation.col(1) = y_axis; rotation.col(2) = z_axis;

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = source;
        pose.linear() = rotation;

        return checkBallBeamVisibility(pose, center, radius);
    }

    double checkBallBeamVisibility(const Eigen::Vector3d source, const Ball b) {
        return checkBallBeamVisibility(source, b.center, b.radius);
    }

    bool checkVisibility(const std::vector<double>& p1, const std::vector<double>& p2) {
        Point3D pt1(p1[0], p1[1], p1[2]);
        Point3D pt2(p2[0], p2[1], p2[2]);
        Segment3D segment(pt1, pt2);

        switch (method_) {
            case VisibilityMethod::BF: return checkVisibilityBF(segment);
            case VisibilityMethod::GPUBF: return checkVisibilityGPUBF(segment);
            case VisibilityMethod::KDTree: return checkVisibilityKDTree(segment);
            default: throw std::runtime_error("Unknown visibility check method.");
        }
    }

    bool checkVisibility(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        std::vector<double> v1 = {p1.x(), p1.y(), p1.z()};
        std::vector<double> v2 = {p2.x(), p2.y(), p2.z()};
        return checkVisibility(v1, v2);
    }




    /**
     * @brief Batch computes visibility between all pairs of points on the GPU.
     * * @param points Input workspace samples
     * @param results Output flattened adjacency matrix (size N*N). 
     * results[i*N + j] == true if visible.
     */
    void computeBatchVisibility(const std::vector<Eigen::Vector3d>& points, std::vector<bool>& results) {
        if (points.empty()) return;
        
        // 1. Ensure Obstacles are on GPU
        updateGPUObstacles(); 

        size_t num_points = points.size();
        size_t matrix_size = num_points * num_points;
        results.assign(matrix_size, false);

        // 2. Allocate GPU Memory for Points
        GPUPoint* d_points;
        cudaMalloc((void**)&d_points, num_points * sizeof(GPUPoint));

        // Convert Eigen to GPUPoint
        std::vector<GPUPoint> host_points(num_points);
        for(size_t i=0; i<num_points; ++i) {
            host_points[i] = {points[i].x(), points[i].y(), points[i].z()};
        }
        cudaMemcpy(d_points, host_points.data(), num_points * sizeof(GPUPoint), cudaMemcpyHostToDevice);

        // 3. Allocate GPU Memory for Results
        bool* d_results;
        cudaMalloc((void**)&d_results, matrix_size * sizeof(bool));
        // Initialize with false (optional, kernel overwrites)
        cudaMemset(d_results, 0, matrix_size * sizeof(bool)); 

        // 4. Launch Kernel
        // d_obstacles_ is a member variable from the previous VisibilityOracle implementation
        launchAllPairsVisibilityKernel(d_obstacles_, obstacles_.size(), d_points, (int)num_points, d_results);

        // 5. Copy Results Back
        // We use a temporary bool array because std::vector<bool> is a bitset specialisation 
        // and its .data() pointer is not a simple bool* array.
        bool* host_results_raw = new bool[matrix_size];
        cudaMemcpy(host_results_raw, d_results, matrix_size * sizeof(bool), cudaMemcpyDeviceToHost);

        // Fill std::vector<bool>
        for(size_t i=0; i<matrix_size; ++i) {
            results[i] = host_results_raw[i];
        }

        // 6. Cleanup
        delete[] host_results_raw;
        cudaFree(d_points);
        cudaFree(d_results);
    }


/**
     * @brief Checks visibility for a list of specific segments on the GPU.
     * Used for efficient edge validation in graphs.
     * @param p1_list Start points of segments
     * @param p2_list End points of segments
     * @param results Output boolean vector (true = visible)
     */
    void checkBatchVisibility(const std::vector<Eigen::Vector3d>& p1_list, 
                              const std::vector<Eigen::Vector3d>& p2_list, 
                              std::vector<bool>& results) {
        if (p1_list.size() != p2_list.size() || p1_list.empty()) return;

        // 1. Sync Obstacles
        updateGPUObstacles();

        size_t num_segments = p1_list.size();
        results.resize(num_segments);

        // 2. Allocate GPU Memory
        GPUPoint* d_p1;
        GPUPoint* d_p2;
        bool* d_results;

        cudaMalloc((void**)&d_p1, num_segments * sizeof(GPUPoint));
        cudaMalloc((void**)&d_p2, num_segments * sizeof(GPUPoint));
        cudaMalloc((void**)&d_results, num_segments * sizeof(bool));

        // 3. Copy Points
        std::vector<GPUPoint> host_p1(num_segments);
        std::vector<GPUPoint> host_p2(num_segments);
        
        for(size_t i=0; i<num_segments; ++i) {
            host_p1[i] = {p1_list[i].x(), p1_list[i].y(), p1_list[i].z()};
            host_p2[i] = {p2_list[i].x(), p2_list[i].y(), p2_list[i].z()};
        }

        cudaMemcpy(d_p1, host_p1.data(), num_segments * sizeof(GPUPoint), cudaMemcpyHostToDevice);
        cudaMemcpy(d_p2, host_p2.data(), num_segments * sizeof(GPUPoint), cudaMemcpyHostToDevice);

        // 4. Launch Kernel (Reuse the generic visibility kernel loop we defined earlier)
        // You might need to expose a generic kernel wrapper that takes arrays of p1/p2
        // If not available, use a loop of launchVisibilityKernel or write a specific 'batchSegmentKernel'
        launchBatchSegmentKernel(d_obstacles_, obstacles_.size(), d_p1, d_p2, (int)num_segments, d_results);

        // 5. Retrieve Results
        bool* host_results_raw = new bool[num_segments];
        cudaMemcpy(host_results_raw, d_results, num_segments * sizeof(bool), cudaMemcpyDeviceToHost);

        for(size_t i=0; i<num_segments; ++i) results[i] = host_results_raw[i];

        delete[] host_results_raw;
        cudaFree(d_p1);
        cudaFree(d_p2);
        cudaFree(d_results);
    }


private:


    bool isPointInBeam(const std::vector<double>& joints, const Eigen::Vector3d& target) {
        if (!fk_solver_) throw std::runtime_error("VisibilityOracle: FK Solver not set.");
        return isPointInBeam(fk_solver_(joints), target);
    }

    bool isPointInBeam(const Eigen::Isometry3d& sensor_pose, const Eigen::Vector3d& target) {
        if (!isPointInFOV(sensor_pose, target)) return false;

        Eigen::Vector3d tool_position = sensor_pose.translation();
        return checkVisibility(tool_position, target);
    }

    bool isPointInFOV(const Eigen::Isometry3d& sensor_pose, const Eigen::Vector3d& target) {
        Eigen::Vector3d local_target = sensor_pose.inverse() * target;
        double dist = local_target.norm();

        if (local_target.z() < 0.0 || dist > tool_params_.beam_length) return false;
        if (dist < 1e-6) return true;

        double cos_theta = local_target.z() / dist;
        double cos_beam = std::cos(tool_params_.beam_angle);

        return cos_theta >= cos_beam;
    }

    bool checkVisibilityBF(const Segment3D& segment) {
        for (const auto& box : obstacles_) {
            if (bg::intersects(segment, box)) return false;
        }
        return true; 
    }

    // ==============================================================================
    // 2. GPU Implementation
    // ==============================================================================
    
    void freeGPUMemory() {
        if (d_obstacles_) {
            cudaFree(d_obstacles_);
            d_obstacles_ = nullptr;
        }
        gpu_capacity_ = 0;
    }

    void updateGPUObstacles() {
        if (!gpu_dirty_) return;

        // Convert Boost Boxes to GPU Structs
        std::vector<GPUBox> host_gpu_boxes;
        host_gpu_boxes.reserve(obstacles_.size());

        for (const auto& b : obstacles_) {
            GPUBox gb;
            gb.min_x = b.min_corner().get<0>();
            gb.min_y = b.min_corner().get<1>();
            gb.min_z = b.min_corner().get<2>();
            gb.max_x = b.max_corner().get<0>();
            gb.max_y = b.max_corner().get<1>();
            gb.max_z = b.max_corner().get<2>();
            host_gpu_boxes.push_back(gb);
        }

        size_t required_bytes = host_gpu_boxes.size() * sizeof(GPUBox);

        // Reallocate if capacity insufficient
        if (host_gpu_boxes.size() > gpu_capacity_) {
            freeGPUMemory();
            cudaMalloc((void**)&d_obstacles_, required_bytes);
            gpu_capacity_ = host_gpu_boxes.size();
        }

        // Copy Data
        if (required_bytes > 0) {
            cudaMemcpy(d_obstacles_, host_gpu_boxes.data(), required_bytes, cudaMemcpyHostToDevice);
        }

        gpu_dirty_ = false;
    }

    bool checkVisibilityGPUBF(const Segment3D& segment) {
        if (obstacles_.empty()) return true;

        // 1. Sync GPU Memory if obstacles changed
        updateGPUObstacles();

        // 2. Prepare Points
        GPUPoint p1 = { bg::get<0,0>(segment), bg::get<0,1>(segment), bg::get<0,2>(segment) };
        GPUPoint p2 = { bg::get<1,0>(segment), bg::get<1,1>(segment), bg::get<1,2>(segment) };

        // 3. Call Kernel Wrapper
        return launchVisibilityKernel(d_obstacles_, obstacles_.size(), p1, p2);
    }

    bool checkVisibilityKDTree(const Segment3D& segment) {
        throw std::runtime_error("KDTree visibility check is not implemented.");
    }
};

} // namespace visual_planner