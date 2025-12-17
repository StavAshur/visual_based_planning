#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "nanoflann.hpp"   // The header you just downloaded
#include "Graph.h"

using namespace nanoflann;

// 1. The Data Container
// This holds your points and allows nanoflann to access them via index
struct PointCloud {
    struct Point {
        std::vector<double> config;
        VertexDesc id;
    };
    std::vector<Point> points;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return points.size(); }

    // Returns the dim'th component of the idx'th point in the class
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return points[idx].config[dim];
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

// 2. The Custom Distance Metric (Robot State Aware)
// This tells nanoflann how to calculate distance between points using MoveIt logic
template<class DataSource>
struct RobotStateDistance {
    typedef double ElementType;
    typedef double DistanceType;

    const DataSource &data_source;
    planning_scene::PlanningScenePtr planning_scene;
    std::string group_name;

    RobotStateDistance(const DataSource &_data_source, 
                       planning_scene::PlanningScenePtr _scene, 
                       const std::string& _group) 
        : data_source(_data_source), planning_scene(_scene), group_name(_group) {}

    // Calculate distance using JointModel logic
    inline DistanceType evalMetric(const ElementType *a, const size_t b_idx, size_t size) const {
        if (!planning_scene) return 0.0;

        const moveit::core::RobotState& state = planning_scene->getCurrentState();
        const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
        if (!jmg) return 0.0;

        const auto& active_joints = jmg->getActiveJointModels();
        double dist_sq = 0.0;

        // Get pointer to point b's data
        const ElementType* b = data_source.points[b_idx].config.data();

        // Iterate over joints
        for (size_t i = 0; i < size && i < active_joints.size(); ++i) {
            const moveit::core::JointModel* joint = active_joints[i];
            
            // distance() calculates the correct difference based on joint type (revolute, cyclic, etc.)
            // Note: This assumes 1 variable per joint in the vector, which is standard for simple arms.
            double d = joint->distance(a + i, b + i);
            dist_sq += d * d;
        }
        return std::sqrt(dist_sq);
    }
    
    // Required by nanoflann interface for optimizations
    // Since our distance is complex (state dependent), accumulation is just the full distance logic per dimension?
    // Nanoflann uses this for early exit. For complex joints, simpler Euclidean might be a safe lower bound, 
    // but to be correct we should use the joint distance. 
    inline DistanceType accum_dist(const ElementType a, const ElementType b, int i) const {
        // Warning: This simplistic accum_dist might not work perfectly if access to JointModel requires the index 'i'.
        // To properly implement this, we'd need access to the specific JointModel for dimension 'i'.
        // For safety/correctness with minimal changes, we can perform the specific joint calc.
        
        if (!planning_scene) return (a - b) * (a - b);
        const moveit::core::RobotState& state = planning_scene->getCurrentState();
        const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name);
        if (!jmg) return (a - b) * (a - b);
        const auto& active_joints = jmg->getActiveJointModels();
        
        if (i < active_joints.size()) {
            double d = active_joints[i]->distance(&a, &b);
            return d * d;
        }
        return (a - b) * (a - b);
    }
};

// 3. The Wrapper Class
class NearestNeighbor {
    // Define the KDTree type using our Custom Metric
    typedef KDTreeSingleIndexAdaptor<
        RobotStateDistance<PointCloud>,
        PointCloud,
        -1 /* Dimension determined at runtime */
        > RobotKDTree;

    PointCloud cloud_;
    RobotKDTree* index_;
    int dim_;
    
    // MoveIt Context
    planning_scene::PlanningScenePtr planning_scene_;
    std::string group_name_;

public:
    NearestNeighbor() : index_(nullptr), dim_(0), group_name_("manipulator") {}

    ~NearestNeighbor() {
        if (index_) delete index_;
    }

    void setPlanningScene(planning_scene::PlanningScenePtr scene, const std::string& group = "manipulator") {
        planning_scene_ = scene;
        group_name_ = group;
    }

    void addPoint(const std::vector<double>& q, VertexDesc v_id) {
        if (dim_ == 0) dim_ = q.size();

        // Add to our internal storage
        PointCloud::Point p;
        p.config = q;
        p.id = v_id;
        cloud_.points.push_back(p);

        // Rebuild the index
        if (index_) delete index_;
        

        index_ = new RobotKDTree(dim_, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10),
                                 planning_scene_, group_name_);
        index_->buildIndex();
    }

    std::vector<VertexDesc> kNearest(const std::vector<double>& query, size_t k) {
        if (cloud_.points.empty()) return {};

        size_t search_k = std::min(k, cloud_.points.size());
        
        // Results containers
        std::vector<size_t> ret_indexes(search_k);
        std::vector<double> out_dists_sq(search_k);

        // Run search
        nanoflann::KNNResultSet<double> resultSet(search_k);
        resultSet.init(&ret_indexes[0], &out_dists_sq[0]);
        
        // Pass the query data pointer
        index_->findNeighbors(resultSet, &query[0], nanoflann::SearchParameters(10));

        // Convert indices back to VertexDesc
        std::vector<VertexDesc> results;
        results.reserve(search_k);
        for (size_t i = 0; i < search_k; ++i) {
            results.push_back(cloud_.points[ret_indexes[i]].id);
        }
        
        return results;
    }
    
    // Brute Force K-NN implementation
    std::vector<VertexDesc> bruteForceKNN(const std::vector<double>& query, size_t k) {
        if (cloud_.points.empty() || !planning_scene_) return {};

        size_t search_k = std::min(k, cloud_.points.size());
        
        // Create a temporary metric instance to calculate distances
        RobotStateDistance<PointCloud> metric(cloud_, planning_scene_, group_name_);
        
        // Store pair <distance, index>
        typedef std::pair<double, size_t> DistIndexPair;
        std::vector<DistIndexPair> distances;
        distances.reserve(cloud_.points.size());

        for (size_t i = 0; i < cloud_.points.size(); ++i) {
            double d = metric.evalMetric(query.data(), i, query.size());
            distances.push_back({d, i});
        }

        // Partial sort to get top k smallest distances
        std::partial_sort(distances.begin(), distances.begin() + search_k, distances.end(),
            [](const DistIndexPair& a, const DistIndexPair& b) {
                return a.first < b.first;
            });

        // Extract IDs
        std::vector<VertexDesc> results;
        results.reserve(search_k);
        for (size_t i = 0; i < search_k; ++i) {
            results.push_back(cloud_.points[distances[i].second].id);
        }

        return results;
    }

    size_t size() const {
        return cloud_.points.size();
    }

    void clear() {
        // Clear the data container
        cloud_.points.clear();
        
        // Delete the index (it must be rebuilt when new points are added)
        if (index_) {
            delete index_;
            index_ = nullptr;
        }
        
        // Reset dimension so next addPoint triggers a rebuild logic
        dim_ = 0; 
    }
};