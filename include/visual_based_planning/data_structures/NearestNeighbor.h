#pragma once

#include <vector>
#include <cmath>
#include <angles/angles.h> // ROS package for cyclic math
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

// 2. The Custom Distance Metric (Cyclic)
// This tells nanoflann how to calculate distance between points
template<class DataSource>
struct CyclicDistance {
    typedef double ElementType;
    typedef double DistanceType;

    const DataSource &data_source;

    CyclicDistance(const DataSource &_data_source) : data_source(_data_source) {}

    // FIX: Renamed from operator() to evalMetric
    inline DistanceType evalMetric(const ElementType *a, const size_t b_idx, size_t size) const {
        double dist_sq = 0.0;
        for (size_t i = 0; i < size; ++i) {
            // Get the value from the tree
            double val_b = data_source.kdtree_get_pt(b_idx, i);
            double val_a = a[i];

            // Use ROS angles::shortest_angular_distance for cyclic joints
            double diff = angles::shortest_angular_distance(val_a, val_b);
            dist_sq += diff * diff;
        }
        return dist_sq;
    }
    
    // Required by nanoflann interface for optimizations
    inline DistanceType accum_dist(const ElementType a, const ElementType b, int) const {
        double diff = angles::shortest_angular_distance(a, b);
        return diff * diff;
    }
};

// 3. The Wrapper Class
class NearestNeighbor {
    // Define the KDTree type using our Custom Metric
    typedef KDTreeSingleIndexAdaptor<
        CyclicDistance<PointCloud>,
        PointCloud,
        -1 /* Dimension determined at runtime */
        > CyclicKDTree;

    PointCloud cloud_;
    CyclicKDTree* index_;
    int dim_;

public:
    NearestNeighbor() : index_(nullptr), dim_(0) {}

    ~NearestNeighbor() {
        if (index_) delete index_;
    }

    void addPoint(const std::vector<double>& q, VertexDesc v_id) {
        if (dim_ == 0) dim_ = q.size();

        // Add to our internal storage
        PointCloud::Point p;
        p.config = q;
        p.id = v_id;
        cloud_.points.push_back(p);

        // Rebuild the index
        // NOTE: nanoflann is optimized for static building. 
        // For dynamic RRT usage, we have to rebuild or use the dynamic add add-on.
        // For "Simple/Minimal", rebuilding is acceptable for small N (<2000), 
        // but for RRT it's better to just append and lazy-build.
        
        if (index_) delete index_;
        
        // Use the custom metric
        index_ = new CyclicKDTree(dim_, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
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