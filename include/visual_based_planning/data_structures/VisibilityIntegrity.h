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
#include <queue>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Dependencies
#include "VisibilityOracle.h"
#include "NearestNeighbor.h"
#include "Graph.h"
#include "../common/Types.h" 
#include "../components/Sampler.h"

namespace visual_planner {

struct VINode {
    int id;                 // Unique ID
    BoundingBox box;        // The bounding box
    std::string path_idx;   // "0", "01", etc. (for debugging)
    
    std::unique_ptr<VINode> left = nullptr;
    std::unique_ptr<VINode> right = nullptr;
    
    bool is_leaf = false;
    int index = -1;    // If leaf, index in the graph

    int height = 0; // 0 for leaves, 1+ for internal
    std::vector<std::pair<int, double>> visible_from_nodes; // IDs of nodes this node sees + visibility score
};

class VisibilityIntegrity {
public:
    VisibilityIntegrityParams params_;

    VisibilityIntegrity() 
        : params_(), rng_(std::random_device{}()) 
    {
        // Default bounds
        workspace_bounds_ = {-2.0, 2.0, -2.0, 2.0, 0.0, 2.0};
    }

    void setVisibilityOracle(std::shared_ptr<VisibilityOracle> oracle) {
        vis_oracle_ = oracle;
    }
    
    void setSampler(std::shared_ptr<Sampler> sampler) {
        sampler_ = sampler;
    }

    void setParams(const VisibilityIntegrityParams& params) {
        params_ = params;
    }

    void setWorkspaceBounds(const BoundingBox& bounds) {
        workspace_bounds_ = bounds;
    }

    const BoundingBox& getWorkspaceBounds() const {
        return workspace_bounds_;
    }

    /**
     * @brief Main initialization function.
     */
    void build() {
        if (!vis_oracle_ || !sampler_) {
            std::cerr << "[VisibilityIntegrity] Error: Oracle or Sampler not set." << std::endl;
            return;
        }

        std::cout << "[VisibilityIntegrity] Building Visibility Integrity Tree..." << std::endl;
        
        leaves_.clear();
        int node_counter = 0;
        int leaf_counter = 0;

        // Initial Convexity Check
        // Use Sampler to get obstacle-free points in the whole workspace
        std::vector<Eigen::Vector3d> S;
        sampler_->sampleInBox(workspace_bounds_, params_.num_samples, S);
        
        root_ = std::make_unique<VINode>();
        root_->box = workspace_bounds_;
        root_->path_idx = "";
        root_->id = node_counter++;

        buildTreeRecursive(root_.get(), workspace_bounds_, "0", node_counter, leaf_counter);
        
        std::cout << "[VisibilityIntegrity] Tree built. Leaves: " << leaves_.size() << std::endl;

    }


/**
     * @brief Samples a point that can see the target 'ball' with high probability.
     * Uses the "visible_from_nodes" list (Inverse Visibility) to find promising candidates.
     */
    bool SampleFromVisibilityRegion(const Ball& ball, Eigen::Vector3d& sampled_point, double visibility_threshold) {
        // 1. Find the leaf that contains the target Ball's center
        int target_leaf_idx = query(ball.center);
        if (target_leaf_idx == -1) return false;

        VINode* target_node = leaves_[target_leaf_idx];

        // 2. Get candidates: Nodes that are known to see this target node
        const auto& seers = target_node->visible_from_nodes;
        if (seers.empty()) return false;

        // 3. Rejection Sampling Loop
        int max_attempts = 100;
        std::uniform_int_distribution<> dist_idx(0, seers.size() - 1);
        std::uniform_real_distribution<double> dist_prob(0.0, 1.0);

        for (int i = 0; i < max_attempts; ++i) {
            // A. Pick a random candidate node from the list
            int rand_idx = dist_idx(rng_);
            const auto& candidate_pair = seers[rand_idx];
            
            int seer_id = candidate_pair.first;
            double vis_score = candidate_pair.second;

            // B. Rejection Sampling: Reject with probability (1 - score)
            // (i.e., Accept if random value < score)
            if (dist_prob(rng_) > vis_score) {
                continue; 
            }

            // C. Retrieve the actual Node object
            // Assumes all_nodes_map_ is populated as per previous steps
            if (all_nodes_map_.find(seer_id) == all_nodes_map_.end()) continue;
            VINode* seer_node = all_nodes_map_[seer_id];

            // D. Sample a point from the seer's bounding box
            std::vector<Eigen::Vector3d> samples;
            // sampleInBox ensures the point is within bounds and obstacle-free
            if (!sampler_->sampleInBox(seer_node->box, 1, samples) || samples.empty()) {
                continue; 
            }
            Eigen::Vector3d candidate_point = samples[0];

            // E. Verify actual visibility against the Ball
            // Returns fraction of ball visible from candidate_point
            double visible_fraction = vis_oracle_->checkBallBeamVisibility(candidate_point, ball);

            if (visible_fraction > visibility_threshold) {
                sampled_point = candidate_point;
                return true;
            }
        }

        return false;
    }

    int query(const Eigen::Vector3d& query_point) {
        if (!root_) return -1;

        // Helper lambda: Check if point is inside a box
        auto contains = [](const BoundingBox& b, const Eigen::Vector3d& p) {
            return (p.x() >= b.x_min && p.x() <= b.x_max &&
                    p.y() >= b.y_min && p.y() <= b.y_max &&
                    p.z() >= b.z_min && p.z() <= b.z_max);
        };

        // 1. Fast fail if outside the global workspace
        if (!contains(root_->box, query_point)) {
            return -1;
        }

        // 2. Traverse down the tree
        VINode* current = root_.get();
        
        while (!current->is_leaf) {
            // Check Left Child
            if (current->left && contains(current->left->box, query_point)) {
                current = current->left.get();
            } 
            // Check Right Child
            else if (current->right && contains(current->right->box, query_point)) {
                current = current->right.get();
            } 
            else {
                // Point is in parent but seemingly not in either child.
                // This can happen due to floating point precision at the split boundary.
                // Fallback: Check which one is closer to center or simply return -1.
                return -1; 
            }
        }

        return current->index;
    }


    bool isConnectedToTarget(const Eigen::Vector3d& query_point, const Eigen::Vector3d& target_point) {
        return true;
    }


private:
    std::shared_ptr<VisibilityOracle> vis_oracle_;
    std::shared_ptr<Sampler> sampler_;
    std::mt19937 rng_;
    BoundingBox workspace_bounds_; 
    size_t num_nodes_ = 0;
    std::unique_ptr<VINode> root_;
    std::vector<VINode*> leaves_; 

    // --- Bounding Box Helpers ---

    Eigen::Vector3d getSize(const BoundingBox& b) {
        return Eigen::Vector3d(
            b.x_max - b.x_min,
            b.y_max - b.y_min,
            b.z_max - b.z_min
        );
    }

    /**
     * @brief Recursive Tree Construction (Algorithm 2)
     */
    void buildTreeRecursive(VINode* node, const BoundingBox& env_bounds, const std::string& idx, int& node_counter, int& leaf_counter) {
        num_nodes_++;
        
        // 1. Sample S_in (Valid points inside current box)
        std::vector<Eigen::Vector3d> S_in;
        sampler_->sampleInBox(node->box, params_.num_samples, S_in);

        // 2. Sample S_out (Valid points in Env \ box)
        std::vector<Eigen::Vector3d> S_out;
        sampler_->sampleOutside(env_bounds, node->box, params_.num_samples, S_out);

        if (S_in.empty() || S_out.empty()) {
            makeLeaf(node, leaf_counter);
            return;
        }

        // 3. Compute Score (Intersection of visibility) - GPU OPTIMIZED
        
        // Combine sets to create a single context for the NxN matrix
        std::vector<Eigen::Vector3d> combined = S_in;
        combined.insert(combined.end(), S_out.begin(), S_out.end());

        // A. Upload and compute NxN matrix on GPU (No large transfer back to CPU)
        vis_oracle_->precomputeGpuMatrix(combined);

        // B. Define "Guards" (S_in)
        // We want to know: Which points in Combined are seen by ALL points in S_in?
        std::vector<int> guard_indices(S_in.size());
        std::iota(guard_indices.begin(), guard_indices.end(), 0); // Fill 0, 1, 2...

        // C. Run Reduction Kernel on GPU
        // Result is size of 'combined'. True if point[i] is seen by all guards.
        std::vector<bool> seen_by_all = vis_oracle_->checkSeenByAllGPU(guard_indices);

        // D. Count intersections
        // We only care about the S_out portion (indices starting at S_in.size())
        int intersection_count = 0;
        int n_out = S_out.size();
        
        for (size_t k = S_in.size(); k < combined.size(); ++k) {
            if (seen_by_all[k]) {
                intersection_count++;
            }
        }

        double score = static_cast<double>(intersection_count) / static_cast<double>(n_out);

        // Termination Criteria
        Eigen::Vector3d dims = getSize(node->box);
        double min_dim = 0.1; 
        bool too_small = (dims.maxCoeff() < min_dim);

        if (score > params_.vi_threshold || too_small) {
            makeLeaf(node, leaf_counter);
        } else {
            // Split (Alg 2 Lines 12-16)
            int d = 0; // 0=x, 1=y, 2=z
            if (dims.y() > dims.x() && dims.y() > dims.z()) d = 1;
            if (dims.z() > dims.x() && dims.z() > dims.y()) d = 2;

            BoundingBox left_box = node->box;
            BoundingBox right_box = node->box;

            if (d == 0) {
                double mid = (node->box.x_min + node->box.x_max) / 2.0;
                left_box.x_max = mid;
                right_box.x_min = mid;
            } else if (d == 1) {
                double mid = (node->box.y_min + node->box.y_max) / 2.0;
                left_box.y_max = mid;
                right_box.y_min = mid;
            } else {
                double mid = (node->box.z_min + node->box.z_max) / 2.0;
                left_box.z_max = mid;
                right_box.z_min = mid;
            }

            node->left = std::make_unique<VINode>();
            node->left->box = left_box;
            node->left->path_idx = idx + "0";
            node->left->id = node_counter++;

            node->right = std::make_unique<VINode>();
            node->right->box = right_box;
            node->right->path_idx = idx + "1";
            node->right->id = node_counter++;

            buildTreeRecursive(node->left.get(), env_bounds, idx + "0", node_counter, leaf_counter);
            buildTreeRecursive(node->right.get(), env_bounds, idx + "1", node_counter, leaf_counter);

            node->height = std::max(node->left->height, node->right->height) + 1;
        }
    }

    void makeLeaf(VINode* node, int& leaf_counter) {
        node->is_leaf = true;
        node->index = leaf_counter++;
        node->height = 0;
        leaves_.push_back(node);
    }


    /**
     * @brief Iterates over tree layers to build visibility connections.
     * Replaces the old graph building method.
     */
    void buildVisibilityLayers() {
        std::cout << "[VisibilityIntegrity] Building Visibility Layers..." << std::endl;

        // 1. Organize all nodes by height
        // We need to traverse the tree to collect pointers to all nodes
        std::map<int, std::vector<VINode*>> layers;
        collectNodesByLayer(root_.get(), layers);

        // 2. Process Layer 0 (Leaves) - The Base Case
        // We use the GPU to compute leaf-to-leaf visibility
        if (layers.count(0)) {
            computeLeafLayerVisibility(layers[0]);
        }

        // 3. Process Upper Layers (1 to Max Height)
        for (auto it = layers.begin(); it != layers.end(); ++it) {
            int h = it->first;
            if (h == 0) continue; // Already done

            std::vector<VINode*>& current_layer = it->second;
            std::cout << "Processing Layer " << h << " (" << current_layer.size() << " nodes)..." << std::endl;

            for (VINode* node : current_layer) {
                if (!node->left || !node->right) continue;

                // A. Compute "Seen By This Node" (Outgoing)
                // V sees U iff Left sees U AND Right sees U.
                // Intersection of sorted vectors.
                std::vector<std::pair<int, double>>& left_vis = node->left->visible_from_nodes;
                std::vector<std::pair<int, double>>& right_vis = node->right->visible_from_nodes;
                
                // (Vectors must be sorted for set_intersection)
                // They are guaranteed sorted by our compute functions.
                
                // A. Compute "Seen By This Node" (Outgoing) - Custom Intersection
                // Both vectors must be sorted by ID (first element) for this to work.
                
                auto it_left = left_vis.begin();
                auto it_right = right_vis.begin();

                while (it_left != left_vis.end() && it_right != right_vis.end()) {
                    if (it_left->first < it_right->first) {
                        ++it_left;
                    } else if (it_right->first < it_left->first) {
                        ++it_right;
                    } else {
                        // Match found! (IDs are equal)
                        double avg_weight = (it_left->second + it_right->second) / 2.0;

                        if (avg_weight > params_.vi_threshold) {
                            node->visible_from_nodes.push_back({it_left->first, avg_weight});
                        }

                        // Move both forward
                        ++it_left;
                        ++it_right;
                    }
                }
            }
        }
    }

    void computeLeafLayerVisibility(std::vector<VINode*>& leaves) {
        int n_samples = params_.num_samples;

        // We compare every leaf against every other leaf (O(L^2)) on GPU
        for (size_t i = 0; i < leaves.size(); ++i) {
            VINode* l1 = leaves[i];
            std::vector<Eigen::Vector3d> S1, S2;
            sampler_->sampleInBox(l1->box, n_samples, S1);
            for (size_t j = i + 1; j < leaves.size(); ++j) {
                VINode* l2 = leaves[j];
                
                sampler_->sampleInBox(l2->box, n_samples, S2);
                
                if (S1.empty() || S2.empty()) continue;

                // (Optimized: Combine S1+S2, Precompute, check sub-blocks)
                // For brevity, using the logic from previous prompts:
                std::pair<double,double> m1m2 = checkBiDirectionalVisibility(S1, S2); // Wrapper helper

                if (m1m2.first > params_.vi_threshold)
                    if (intersectsTargetSphere(l2->box))
                        l1->visible_from_nodes.push_back(std::make_pair(l2->id,m1m2.first));
                if (m1m2.second > params_.vi_threshold)
                    if (intersectsTargetSphere(l1->box))
                        l2->visible_from_nodes.push_back(std::make_pair(l1->id,m1m2.second));
            }
            // Ensure sorted for intersection later
            std::sort(leaves[i]->visible_from_nodes.begin(), leaves[i]->visible_from_nodes.end(), [](const auto& a, const auto& b) {
                return a.first < b.first;
            });
        }
    }

    bool intersectsTargetSphere(BoundingBox& b){
        double cx = 0.33, cy = 0.0, cz = 0.33;
        double r_sq = 1.2 * 1.2;
        double dist_sq = 0.0;

        // Calculate squared distance from Center to the Box
        if (cx < b.x_min) dist_sq += (cx - b.x_min) * (cx - b.x_min);
        else if (cx > b.x_max) dist_sq += (cx - b.x_max) * (cx - b.x_max);

        if (cy < b.y_min) dist_sq += (cy - b.y_min) * (cy - b.y_min);
        else if (cy > b.y_max) dist_sq += (cy - b.y_max) * (cy - b.y_max);

        if (cz < b.z_min) dist_sq += (cz - b.z_min) * (cz - b.z_min);
        else if (cz > b.z_max) dist_sq += (cz - b.z_max) * (cz - b.z_max);

        return dist_sq <= r_sq;
    };


    void collectNodesByLayer(VINode* node, std::map<int, std::vector<VINode*>>& layers) {
        if (!node) return;
        layers[node->height].push_back(node);
        all_nodes_map_[node->id] = node; // Helper for ID lookups
        collectNodesByLayer(node->left.get(), layers);
        collectNodesByLayer(node->right.get(), layers);
    }

    // Helper map for ID lookup
    std::unordered_map<int, VINode*> all_nodes_map_;


    std::pair<double,double> checkBiDirectionalVisibility(const std::vector<Eigen::Vector3d>& S1, 
                                                   const std::vector<Eigen::Vector3d>& S2) 
    {
        if (S1.empty() || S2.empty()) {
            ROS_ERROR("[VisibilityIntegrity] either S1 or S2 is empty");
            return std::make_pair(-1.0, -1.0);
        }
        

        // 1. Combine samples into a single context [S1 ... S2]
        std::vector<Eigen::Vector3d> combined = S1;
        combined.insert(combined.end(), S2.begin(), S2.end());

        // 2. Upload and Compute NxN Matrix on GPU
        // This is the heavy lifting, done once for this pair.
        vis_oracle_->precomputeGpuMatrix(combined);


        // --- Direction 1: Does S2 see S1? ---
        // Guards = All points in S2 (indices S1.size() to end)
        std::vector<int> guards_s2(S2.size());
        std::iota(guards_s2.begin(), guards_s2.end(), (int)S1.size());

        std::vector<bool> res_s2 = vis_oracle_->checkSeenByAllGPU(guards_s2);

        double m1 = 0.0;
        // We check the success count in the S1 portion (indices 0 to S1.size()-1)
        for (size_t k = 0; k < S1.size(); ++k) {
            if (res_s2[k]) m1 = m1 + 1.0;
        }

        // --- Direction 2: Does S1 see S2? ---
        // Guards = All points in S1 (indices 0 to S1.size()-1)
        std::vector<int> guards_s1(S1.size());
        std::iota(guards_s1.begin(), guards_s1.end(), 0);
        
        // Result is boolean for every point in 'combined'
        std::vector<bool> res_s1 = vis_oracle_->checkSeenByAllGPU(guards_s1);

        double m2 = 0.0;
        // We check the success count in the S2 portion (indices starting at S1.size())
        for (size_t k = S1.size(); k < combined.size(); ++k) {
            if (res_s1[k]) m2 = m2 + 1.0;
        }

        // Return true only if mutual visibility exists
        return std::make_pair(m1/S2.size(),m2/S1.size());
    }
};

} // namespace visual_planner