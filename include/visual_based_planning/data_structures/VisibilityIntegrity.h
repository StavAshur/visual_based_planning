#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <memory>
#include <iostream>
#include <unordered_set>
#include <random>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Dependencies
#include "VisibilityOracle.h"
#include "NearestNeighbor.h"
#include "Graph.h"
#include "../common/Types.h"
#include "../components/Sampler.h"

namespace visual_planner {

class VisibilityIntegrity {
public:
    struct Cluster {
        int id;
        Eigen::Vector3d center; // Geometric centroid of the cluster
        double radius;          // Radius of the cluster
        int size_;              // Number of points in the cluster
        std::vector<int> member_indices; // Indices of workspace samples belonging to this cluster
        
        // REFACTORED: These now store indices of *workspace samples* visible from the cluster
        std::vector<int> visible_samples_intersection; // Cached Intersection of Vis(p)
        std::vector<int> visible_samples_union;        // Cached Union of Vis(p)
    };

VisibilityIntegrityParams params_;


public:
    VisibilityIntegrity() 
        : params_(), rng_(std::random_device{}()) {}

    void setVisibilityOracle(std::shared_ptr<VisibilityOracle> oracle) {
        vis_oracle_ = oracle;
    }
    
    void setSampler(std::shared_ptr<Sampler> sampler) {
        sampler_ = sampler;
    }

    void setParams(const VisibilityIntegrityParams& params) {
        params_ = params;
    }

    /**
     * @brief Main initialization function.
     */
    void build() {
        if (!vis_oracle_ || !sampler_) {
            ROS_ERROR("[VisibilityIntegrity] Error: Oracle or Sampler not set.");
            return;
        }
        
        // 1. Sample Points
        std::cout << "[VisibilityIntegrity] Sampling " << params_.num_samples << " valid points..." << std::endl;
        workspace_samples_.clear();
        sampler_->sampleValidPoints(params_.num_samples, workspace_samples_, params_.face_samples);
        
        // 2. Precompute Visibility between Samples (New Definition of Vis(p))
        std::cout << "[VisibilityIntegrity] Computing sample-to-sample visibility..." << std::endl;
        computeSampleVisibility();

        // 3. Cluster based on VI
        std::cout << "[VisibilityIntegrity] Clustering..." << std::endl;
        performClustering();

        // 4. Build Cluster Visibility Graph
        std::cout << "[VisibilityIntegrity] Building Cluster Visibility Graph for " << clusters_.size() << " clusters..." << std::endl;
        buildClusterVisibilityGraph();
        
        // Build NN structure for points (All Samples)
        point_nn_.clear();
        for (size_t i = 0; i < workspace_samples_.size(); ++i) {
            std::vector<double> pt_vec = {workspace_samples_[i].x(), workspace_samples_[i].y(), workspace_samples_[i].z()};
            point_nn_.addPoint(pt_vec, i); // Store index as ID
        }
    }



/**
     * @brief Samples a point from the visibility region of the cluster containing 'point'.
     * 1. Classify 'point' to find its cluster C.
     * 2. Get neighbor clusters N_VG(C) from cluster visibility graph.
     * 3. Select one cluster from N_VG(C) via weighted sampling (size-based).
     * 4. Sample a valid point from that cluster's sphere.
     * 5. Verify sampled point belongs to the cluster and sees input point.
     */
    bool SampleFromVisibilityRegion(const Eigen::Vector3d& point, Eigen::Vector3d& sampled_point) {
        // ROS_WARN("Attempting sampling from visibility region..."); // Optional logging
        // 1. Find cluster for the input point
        int start_cluster_id = query(point);
        if (start_cluster_id == -1) return false;

        // 2. Get neighbor clusters 
        const std::vector<std::pair<int, double>>& candidate_clusters = cluster_vis_graph_[start_cluster_id];
        
        if (candidate_clusters.empty()) return false;

        // 3. Sample Loop with rejection sampling
        int chosen_cluster_id = -1;
    
        int max_attempts = 100;
        std::uniform_real_distribution<double> sample_dist(-1.0, 1.0);
        std::uniform_real_distribution<> reject_dist(0.0, 1.0);
        std::uniform_int_distribution<> dist_idx(0, candidate_clusters.size() - 1);


        for (int i = 0; i < max_attempts; ++i) {
            // Pick a cluster
            int candidate_idx = dist_idx(rng_);
            const auto& candidate_pair = candidate_clusters[candidate_idx];
            int candidate_id = candidate_pair.first;
            double weight = candidate_pair.second;

            if (reject_dist(rng_) < weight)
                chosen_cluster_id = candidate_id;
            else
                continue;

            const Cluster& chosen_cluster = clusters_[chosen_cluster_id];

            // Sample from sphere
            Eigen::Vector3d offset;
            do {
                offset = Eigen::Vector3d(sample_dist(rng_), sample_dist(rng_), sample_dist(rng_));
            } while (offset.squaredNorm() > 1.0);

            Eigen::Vector3d candidate = chosen_cluster.center + offset * chosen_cluster.radius;

            // --- MODIFICATION: Cluster Membership Check ---
            // Ensure the geometric candidate actually belongs to the topological cluster
            int candidate_actual_cluster = query(candidate);
            if (candidate_actual_cluster != chosen_cluster_id) {
                continue; // Reject if it falls into a different cluster's Voronoi region
            }

            // 4. Verify Visibility: Candidate must see Input Point
            if (vis_oracle_->checkVisibility(candidate, point)) {
                sampled_point = candidate;
                return true;
            }
        }

        return false;
    }

    /**
     * @brief Samples a point from the visibility region of the cluster containing 'ball'.
     * ... (Logic same as above, updated with membership check)
     */
    bool SampleFromVisibilityRegion(const Ball& ball, Eigen::Vector3d& sampled_point, double visibility_threshold) {

        // 1. Find cluster for the input ball using majority vote
        std::map<int, int> cluster_votes;
        std::uniform_real_distribution<double> dist_ball(-1.0, 1.0);
        int ball_samples = 10;

        for (int i = 0; i < ball_samples; ++i) {
            Eigen::Vector3d offset;
            do {
                offset = Eigen::Vector3d(dist_ball(rng_), dist_ball(rng_), dist_ball(rng_));
            } while (offset.squaredNorm() > 1.0);
            
            Eigen::Vector3d sample_in_ball = ball.center + offset * ball.radius;
            int cid = query(sample_in_ball);
            if (cid != -1) {
                cluster_votes[cid]++;
            }
        }

        if (cluster_votes.empty()) return false;

        int start_cluster_id = -1;
        int max_votes = -1;
        for (const auto& pair : cluster_votes) {
            if (pair.second > max_votes) {
                max_votes = pair.second;
                start_cluster_id = pair.first;
            }
        }

        if (start_cluster_id == -1) return false;

        // 2. Get neighbor clusters
        const std::vector<std::pair<int, double>>& candidate_clusters = cluster_vis_graph_[start_cluster_id];
        
        if (candidate_clusters.empty()) return false;

        // 3. Sample Loop with rejection sampling
        int chosen_cluster_id = -1;
    
        int max_attempts = 100;
        std::uniform_int_distribution<> dist_idx(0, candidate_clusters.size() - 1);
        std::uniform_real_distribution<double> sample_dist(-1.0, 1.0);
        std::uniform_real_distribution<> reject_dist(0.0, 1.0);

        for (int i = 0; i < max_attempts; ++i) {
            // Pick a cluster
            int candidate_idx = dist_idx(rng_);
            const auto& candidate_pair = candidate_clusters[candidate_idx];
            int candidate_id = candidate_pair.first;
            double weight = candidate_pair.second;

            if (reject_dist(rng_) < weight)
                chosen_cluster_id = candidate_id;
            else
                continue;

            const Cluster& chosen_cluster = clusters_[chosen_cluster_id];

            // Sample from sphere
            Eigen::Vector3d offset;
            do {
                offset = Eigen::Vector3d(sample_dist(rng_), sample_dist(rng_), sample_dist(rng_));
            } while (offset.squaredNorm() > 1.0);

            Eigen::Vector3d candidate = chosen_cluster.center + offset * chosen_cluster.radius;
            
            // --- MODIFICATION: Cluster Membership Check ---
            int candidate_actual_cluster = query(candidate);
            if (candidate_actual_cluster != chosen_cluster_id) {
                continue; 
            }

            // 4. Verify Visibility: Candidate must see Input Ball Center
            if (vis_oracle_->checkBallBeamVisibility(candidate, ball) > visibility_threshold) {
                sampled_point = candidate;
                return true;
            }
        }

        return false;
    }

    

    /**
     * @brief Query function using K-Nearest Neighbors on workspace samples.
     * Returns the Cluster ID (label) assigned to the query point.
     * @return int Cluster ID or -1 if invalid.
     */
    int query(const Eigen::Vector3d& query_point) {
        if (workspace_samples_.empty() || point_to_cluster_map_.empty()) return -1;

        std::vector<double> q_vec = {query_point.x(), query_point.y(), query_point.z()};
        
        // Get K nearest points
        std::vector<VertexDesc> neighbors = point_nn_.kNearest(q_vec, params_.k_neighbors);
        if (neighbors.empty()) return -1;

        // Vote for cluster
        std::map<int, int> votes;
        for (auto idx : neighbors) {
            if (idx < point_to_cluster_map_.size()) {
                int cid = point_to_cluster_map_[idx];
                if (cid != -1) votes[cid]++;
            }
        }

        if (votes.empty()) return -1;

        // Find winner
        int best_cid = -1;
        int max_votes = -1;
        for (const auto& pair : votes) {
            if (pair.second > max_votes) {
                max_votes = pair.second;
                best_cid = pair.first;
            }
        }
        return best_cid;
    }

    /**
     * @brief Checks if the cluster containing 'query_point' has a direct edge 
     * to the cluster containing 'target_point' in the Cluster Visibility Graph.
     */
    bool isConnectedToTarget(const Eigen::Vector3d& query_point, const Eigen::Vector3d& target_point) {
        // 1. Identify Clusters
        int query_cluster_id = query(query_point);
        int target_cluster_id = query(target_point);

        // Safety checks
        if (query_cluster_id == -1 || target_cluster_id == -1) {
            return false;
        }

        // If they are in the same cluster, we assume they are connected/visible locally
        if (query_cluster_id == target_cluster_id) {
            return true;
        }

        // 2. Check Adjacency in the Cluster Graph
        // cluster_vis_graph_[i] contains pairs {neighbor_id, weight} representing edges i -> neighbor
        const auto& neighbors = cluster_vis_graph_[target_cluster_id];
        
        for (const auto& edge : neighbors) {
            if (edge.first == query_cluster_id) {
                return true; 
            }
        }

        return false;
    }

private:


    std::shared_ptr<VisibilityOracle> vis_oracle_;
    
    // The sampled workspace points W
    std::vector<Eigen::Vector3d> workspace_samples_;
    
    // Precomputed visibility: vis_cache_[i] = list of indices of OTHER workspace samples visible from workspace_samples_[i]
    // Used for calculating VI during clustering
    std::vector<std::vector<int>> vis_cache_;

    // The Clusters
    std::vector<Cluster> clusters_;

    // Data Structures for Querying
    NearestNeighbor point_nn_;   // Maps a query point to closest workspace samples (for k-NN)
    
    // Cluster Visibility Graph: Nodes are Cluster IDs. Edge exists if C_i sees C_j.
    // Stored as adjacency list: cluster_vis_graph_[i] = { {j, weight}, {k, weight}, ... }
    std::vector<std::vector<std::pair<int, double>>> cluster_vis_graph_;

    // Map: Point Index -> Cluster ID
    std::vector<int> point_to_cluster_map_;

    // int params_.num_samples = 1000;
    // double params_.vi_threshold;
    // double params_.limit_diameter_factor; 
    // int params_.k_neighbors = 5; // Default k for query
    // int params_.face_samples = -1;

    std::shared_ptr<Sampler> sampler_; 
    std::mt19937 rng_;


/**
     * @brief Computes visibility using the GPU Oracle if available.
     * Replaces the CPU-based loop in build().
     */
    void computeSampleVisibility() {
        // If Oracle is set to GPU mode, use the batch accelerator
        if (vis_oracle_->getMethod() == VisibilityOracle::VisibilityMethod::GPUBF) {
            computeSampleVisibilityGPU();
        } else {
            // Fallback to original CPU implementation
            computeSampleVisibilityCPU();
        }
    }

    // Renamed your original function to this:
    void computeSampleVisibilityCPU() {
        vis_cache_.clear();
        vis_cache_.resize(workspace_samples_.size());
        
        for (size_t i = 0; i < workspace_samples_.size(); ++i) {
            vis_cache_[i].push_back(i); 
            for (size_t j = i + 1; j < workspace_samples_.size(); ++j) {
                if (vis_oracle_->checkVisibility(workspace_samples_[i], workspace_samples_[j])) {
                    vis_cache_[i].push_back(j);
                    vis_cache_[j].push_back(i);
                }
            }
            std::sort(vis_cache_[i].begin(), vis_cache_[i].end());
        }
    }

    // The new GPU logic
    void computeSampleVisibilityGPU() {
        std::cout << "[VisibilityIntegrity] GPU Acceleration: Computing " 
                  << workspace_samples_.size() * workspace_samples_.size() 
                  << " visibility pairs..." << std::endl;

        vis_cache_.clear();
        vis_cache_.resize(workspace_samples_.size());

        // 1. Get flat matrix from Oracle
        std::vector<bool> flat_results;
        vis_oracle_->computeBatchVisibility(workspace_samples_, flat_results);

        int N = workspace_samples_.size();

        // 2. Parse flat matrix into Adjacency List (vis_cache_)
        for (int i = 0; i < N; ++i) {
            vis_cache_[i].push_back(i); // Always visible to self

            for (int j = i + 1; j < N; ++j) {
                // Index mapping: Row-major (i * N + j)
                // Note: The kernel only computes for i < j. 
                // We trust those results and mirror them.
                if (flat_results[i * N + j]) {
                    vis_cache_[i].push_back(j);
                    vis_cache_[j].push_back(i);
                }
            }
            // Ensure sorted order for set intersections later
            std::sort(vis_cache_[i].begin(), vis_cache_[i].end());
        }
    }

    // --- Step 2: Clustering Logic (Greedy) ---
    void performClustering() {
        clusters_.clear();
        point_to_cluster_map_.assign(workspace_samples_.size(), -1);
        
        // List of remaining points (indices)
        std::vector<int> remaining_points;
        remaining_points.reserve(workspace_samples_.size());
        for (size_t i = 0; i < workspace_samples_.size(); ++i) {
            remaining_points.push_back(i);
        }

        while (!remaining_points.empty()) {
            // 1. Pick random first point
            std::uniform_int_distribution<> dist(0, remaining_points.size() - 1);
            int rand_idx = dist(rng_);
            int first_point_idx = remaining_points[rand_idx];

            Cluster c;
            c.id = clusters_.size();
            c.member_indices.push_back(first_point_idx);
            point_to_cluster_map_[first_point_idx] = c.id; // Map
            
            c.visible_samples_intersection = vis_cache_[first_point_idx];
            c.visible_samples_union = vis_cache_[first_point_idx];

            remaining_points[rand_idx] = remaining_points.back();
            remaining_points.pop_back();

            Eigen::Vector3d p_first = workspace_samples_[first_point_idx];
            
            std::sort(remaining_points.begin(), remaining_points.end(),
                [&](int a, int b) {
                    return (workspace_samples_[a] - p_first).squaredNorm() < (workspace_samples_[b] - p_first).squaredNorm();
                }
            );

            double max_dist_to_first = 0.0;
            
            for (size_t i = 0; i < remaining_points.size(); ) {
                int curr_idx = remaining_points[i];
                double dist = (workspace_samples_[curr_idx] - p_first).norm();

                if (c.member_indices.size() >= 10 && max_dist_to_first > 1e-6) {
                    if (dist > params_.limit_diameter_factor * max_dist_to_first) {
                        break; 
                    }
                }

                double score = calculateVI_Incremental(c, vis_cache_[curr_idx]);

                if (score >= params_.vi_threshold) {
                    c.member_indices.push_back(curr_idx);
                    point_to_cluster_map_[curr_idx] = c.id; // Map
                    
                    updateSets(c.visible_samples_intersection, c.visible_samples_union, vis_cache_[curr_idx]);
                    
                    if (dist > max_dist_to_first) {
                        max_dist_to_first = dist;
                    }

                    remaining_points.erase(remaining_points.begin() + i);
                } else {
                    ++i;
                }
            }

            // Compute Stats
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            for (int idx : c.member_indices) {
                sum += workspace_samples_[idx];
            }
            c.center = sum / c.member_indices.size();
            c.size_ = c.member_indices.size(); // Set size

            double max_r_sq = 0.0;
            for (int idx : c.member_indices) {
                double r_sq = (workspace_samples_[idx] - c.center).squaredNorm();
                if (r_sq > max_r_sq) max_r_sq = r_sq;
            }
            c.radius = std::sqrt(max_r_sq);

            clusters_.push_back(c);
        }
    }



    // --- Step 3: Cluster Visibility Graph Building ---
    void buildClusterVisibilityGraph() {
        cluster_vis_graph_.clear();
        cluster_vis_graph_.resize(clusters_.size());

        // --- Define Goal Region ---
        Eigen::Vector3d goal_center(0.33, 0.0, 0.33);
        double goal_radius = 1.2;
        
        std::vector<bool> cluster_intersects_goal(clusters_.size(), false);
        for (size_t i = 0; i < clusters_.size(); ++i) {
            double dist = (clusters_[i].center - goal_center).norm();
            if (dist < (clusters_[i].radius + goal_radius)) {
                cluster_intersects_goal[i] = true;
            }
        }

        int checks_per_pair = 100; 
        
        // --- GPU OPTIMIZATION START ---
        if (vis_oracle_->getMethod() == VisibilityOracle::VisibilityMethod::GPUBF) {
            
            // 1. Prepare Batch Data Structures
            std::vector<Eigen::Vector3d> batch_p1;
            std::vector<Eigen::Vector3d> batch_p2;
            // Store mapping to know which result belongs to which cluster pair
            struct PairMap { size_t i; size_t j; int start_idx; };
            std::vector<PairMap> pair_mappings;

            // 2. Collect all segments to check
            for (size_t i = 0; i < clusters_.size(); ++i) {
                for (size_t j = i + 1; j < clusters_.size(); ++j) {
                    // Skip if neither targets the goal
                    if (!cluster_intersects_goal[i] && !cluster_intersects_goal[j]) continue;

                    PairMap pm;
                    pm.i = i; 
                    pm.j = j;
                    pm.start_idx = batch_p1.size();
                    pair_mappings.push_back(pm);

                    for (int k = 0; k < checks_per_pair; ++k) {
                        int idx_i = clusters_[i].member_indices[std::uniform_int_distribution<>(0, clusters_[i].size_ - 1)(rng_)];
                        int idx_j = clusters_[j].member_indices[std::uniform_int_distribution<>(0, clusters_[j].size_ - 1)(rng_)];
                        
                        batch_p1.push_back(workspace_samples_[idx_i]);
                        batch_p2.push_back(workspace_samples_[idx_j]);
                    }
                }
            }

            // 3. Batch Process on GPU
            std::vector<bool> batch_results;
            if (!batch_p1.empty()) {
                vis_oracle_->checkBatchVisibility(batch_p1, batch_p2, batch_results);
            }

            // 4. Reconstruct Graph
            for (const auto& pm : pair_mappings) {
                int visible_count = 0;
                for (int k = 0; k < checks_per_pair; ++k) {
                    if (batch_results[pm.start_idx + k]) {
                        visible_count++;
                    }
                }

                if (visible_count > 0) {
                    double weight = static_cast<double>(visible_count) / checks_per_pair;
                    
                    if (cluster_intersects_goal[pm.j]) 
                        cluster_vis_graph_[pm.i].push_back({static_cast<int>(pm.j), weight});
                    
                    if (cluster_intersects_goal[pm.i]) 
                        cluster_vis_graph_[pm.j].push_back({static_cast<int>(pm.i), weight});
                }
            }

        } else {
            // --- EXISTING CPU LOGIC ---
            for (size_t i = 0; i < clusters_.size(); ++i) {
                for (size_t j = i + 1; j < clusters_.size(); ++j) {
                    
                    if (!cluster_intersects_goal[i] && !cluster_intersects_goal[j]) continue;

                    int visible_count = 0;
                    for (int k = 0; k < checks_per_pair; ++k) {
                        int idx_i = clusters_[i].member_indices[std::uniform_int_distribution<>(0, clusters_[i].size_ - 1)(rng_)];
                        int idx_j = clusters_[j].member_indices[std::uniform_int_distribution<>(0, clusters_[j].size_ - 1)(rng_)];
                        
                        if (vis_oracle_->checkVisibility(workspace_samples_[idx_i], workspace_samples_[idx_j])) {
                            visible_count++;
                        }
                    }

                    if (visible_count > 0) {
                        double weight = static_cast<double>(visible_count) / checks_per_pair;
                        if (cluster_intersects_goal[j]) 
                            cluster_vis_graph_[i].push_back({static_cast<int>(j), weight});
                        if (cluster_intersects_goal[i]) 
                            cluster_vis_graph_[j].push_back({static_cast<int>(i), weight});
                    }
                }
            }
        }
    }




    // --- Helpers ---

    double calculateVI_Incremental(const Cluster& c, const std::vector<int>& candidate_vis) {
        std::vector<int> temp_inter;
        std::set_intersection(c.visible_samples_intersection.begin(), c.visible_samples_intersection.end(),
                              candidate_vis.begin(), candidate_vis.end(),
                              std::back_inserter(temp_inter));

        if (temp_inter.empty()) return 0.0; 

        std::vector<int> temp_union;
        std::set_union(c.visible_samples_union.begin(), c.visible_samples_union.end(),
                       candidate_vis.begin(), candidate_vis.end(),
                       std::back_inserter(temp_union));

        return static_cast<double>(temp_inter.size()) / static_cast<double>(temp_union.size());
    }

    void updateSets(std::vector<int>& intersect_set, std::vector<int>& union_set, const std::vector<int>& new_set) {
        std::vector<int> next_inter;
        std::set_intersection(intersect_set.begin(), intersect_set.end(),
                              new_set.begin(), new_set.end(),
                              std::back_inserter(next_inter));
        intersect_set = next_inter;

        std::vector<int> next_union;
        std::set_union(union_set.begin(), union_set.end(),
                       new_set.begin(), new_set.end(),
                       std::back_inserter(next_union));
        union_set = next_union;
    }
};

} // namespace visual_planner