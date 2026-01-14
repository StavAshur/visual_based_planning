#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>
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
    VINode* parent = nullptr;
    
    
    bool is_leaf = false;
    int index = -1;    // If leaf, index in the graph

    int height = 0; // 0 for leaves, 1+ for internal
    std::vector<std::pair<VINode*, double>> visible_from_nodes; // IDs of nodes this node sees + visibility score
    double convexity_score = 0.0;
    bool visibility_computed = false;
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
        ComputeTreeVisibility(root_.get());
        std::cout << "[VisibilityIntegrity] Tree built. Leaves: " << leaves_.size() << std::endl;

    }


    /**
     * @brief Samples a point that can see the target 'ball' with high probability.
     * Uses the "visible_from_nodes" list (Inverse Visibility) to find promising candidates.
     */
    bool SampleFromVisibilityRegion(const Ball& ball, Eigen::Vector3d& sampled_point, double visibility_threshold) {
        // 1. Find the leaf that contains the target Ball's center
        std::vector<std::pair<VINode*, double>> seers;
        int target_leaf_idx = query(ball.center, seers);
        if (target_leaf_idx == -1) return false;

        VINode* target_node = leaves_[target_leaf_idx];

        if (seers.empty()) {
            ROS_WARN("[SampleFromVisibilityRegion] no one sees this point????");
            return false;
        }
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

    /**
     * @brief Finds the leaf containing the query point and populates the input vector
     * with 'visible_from_nodes' from all ancestors along the path.
     * * @param query_point The point to search for.
     * @param out_seeing_nodes Output vector to populate with (node_id, score) pairs.
     * @return int The leaf_index of the found leaf, or -1 if not found.
     */
    VINode* query(const Eigen::Vector3d& query_point, std::vector<std::pair<VINode*, double>>& out_seeing_nodes) {
        // Clear previous results to ensure the vector contains only relevant nodes
        out_seeing_nodes.clear();

        if (!root_) return nullptr;

        // Helper lambda: Check if point is inside a box
        auto contains = [](const BoundingBox& b, const Eigen::Vector3d& p) {
            return (p.x() >= b.x_min && p.x() <= b.x_max &&
                    p.y() >= b.y_min && p.y() <= b.y_max &&
                    p.z() >= b.z_min && p.z() <= b.z_max);
        };

        // 1. Fast fail if outside the global workspace
        if (!contains(root_->box, query_point)) {
            return nullptr;
        }

        VINode* current = root_.get();

        // 2. Traverse down to the leaf
        while (current) {
            // Accumulate visibility from the current node on the path
            out_seeing_nodes.insert(
                out_seeing_nodes.end(),
                current->visible_from_nodes.begin(),
                current->visible_from_nodes.end()
            );

            if (current->is_leaf) {
                return current;
            }

            // Determine which child to traverse
            if (current->left && contains(current->left->box, query_point)) {
                current = current->left.get();
            } 
            else if (current->right && contains(current->right->box, query_point)) {
                current = current->right.get();
            } 
            else {
                // Point is within parent box but not strictly in children (gap/precision).
                return nullptr; 
            }
        }

        return nullptr;
    }

    /**
     * @brief Ball Query: Returns nodes contained in or intersecting the ball (leaves only for intersection).
     * Populates 'out_seeing_nodes' with visibility lists from ALL visited nodes (ancestors/contained/intersecting).
     */
    void std::vector<VINode*> query(const Ball& ball, std::vector<std::pair<VINode*, double>>& out_seeing_nodes) {
        out_seeing_nodes.clear();

        if (!root_) return;

        // Recursive Lambda for Traversal
        // We use std::function to allow recursion
        std::function<void(VINode*)> traverse = [&](VINode* node) {
            if (!node) return;

            // 1. Check Intersection
            // If the distance from the ball center to the box is > radius, they are disjoint.
            double d2 = distSqPointBox(ball.center, node->box);
            if (d2 > ball.radius * ball.radius) {
                return; // Prune: No intersection
            }

            // 2. Accumulate Visibility (From this node, which intersects the ball)
            out_seeing_nodes.insert(
                out_seeing_nodes.end(),
                node->visible_from_nodes.begin(),
                node->visible_from_nodes.end()
            );

            // 3. Check Containment
            // If fully contained, we add this node and STOP recursing (as per instructions).
            if (isBoxFullyInBall(node->box, ball)) {
                result_nodes.push_back(node->id);
                return; 
            }

            // 4. Handle Partial Intersection
            if (node->is_leaf) {
                // If leaf intersects (but not fully contained), we still return it.
                result_nodes.push_back(node);
            } else {
                // Internal node intersects but is not contained: Recurse
                traverse(node->left.get());
                traverse(node->right.get());
            }
        };

        traverse(root_.get());
        return result_nodes;
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


    bool buildTreeRecursive(VINode* node, const BoundingBox& env_bounds, const std::string& idx, int& node_counter, int& leaf_counter) {
        num_nodes_++;
        
        // Calculate size early for logging
        Eigen::Vector3d box_size = getSize(node->box);

        ROS_INFO("[Node %s] Processing ID: %d | Bounds: [%.2f, %.2f, %.2f]", 
                idx.c_str(), node_counter, box_size.x(), box_size.y(), box_size.z());

        // 1. Sample S_in (Valid points inside current box)
        std::vector<Eigen::Vector3d> S_in;
        sampler_->sampleInBox(node->box, params_.num_samples, S_in);

        // 2. Sample S_out (Valid points in Env \ box)
        std::vector<Eigen::Vector3d> S_out;
        if (node_counter == 1) {
            // Root node logic
            sampler_->sampleInBox(node->box, params_.num_samples, S_out);
        } else {
            sampler_->sampleOutside(env_bounds, node->box, params_.num_samples, S_out);
        }

        // DEBUG: Check sample sizes (casting size_t to unsigned long for %lu)
        // ROS_INFO("Samples: S_in=%lu, S_out=%lu", S_in.size(), S_out.size());

        if (S_in.empty() || S_out.empty()) {
            // ROS_WARN("[Node %s] EMPTY SAMPLES! Forcing LEAF. (S_in=%lu, S_out=%lu)", 
            //         idx.c_str(), S_in.size(), S_out.size());
            return false;
        }

        // 3. Compute Score (Intersection of visibility) - GPU OPTIMIZED
        
        // Combine sets to create a single context for the NxN matrix
        std::vector<Eigen::Vector3d> combined = S_in;
        combined.insert(combined.end(), S_out.begin(), S_out.end());

        // A. Upload and compute NxN matrix on GPU
        vis_oracle_->precomputeGpuMatrix(combined);

        // B. Define "Guards" (S_in)
        std::vector<int> guard_indices(S_in.size());
        std::iota(guard_indices.begin(), guard_indices.end(), 0); 

        // C. Run Reduction Kernel on GPU
        std::vector<bool> seen_by_all = vis_oracle_->checkSeenByAllGPU(guard_indices);
        std::vector<bool> seen_by_any = vis_oracle_->checkSeenByAnyGPU(guard_indices);

        // D. Count
        int numerator_count = 0;
        int denominator_count = 0;
        
        // Iterate only over the S_out portion of the combined vector
        for (size_t k = S_in.size(); k < combined.size(); ++k) {
            if (seen_by_all[k]) numerator_count++;
            if (seen_by_any[k]) denominator_count++;
        }

        double score = (denominator_count > 0) 
                    ? static_cast<double>(numerator_count) / static_cast<double>(denominator_count)
                    : 0.0;


        // E. Compute Convexity Score
        // We reuse the 'seen_by_all' vector. Indices 0 to S_in.size()-1 correspond 
        // to points in S_in checking visibility against all other S_in points.
        int convex_count = 0;
        size_t n_in = S_in.size();

        for (size_t k = 0; k < n_in; ++k) {
            if (seen_by_all[k]) {
                convex_count++;
            }
        }

        double convexity_score = (n_in > 0) 
                               ? static_cast<double>(convex_count) / static_cast<double>(n_in) 
                               : 0.0;

        node->convexity_score = convexity_score;


        // DEBUG: Print Score Details
        // ROS_INFO("Score Calc: %d/%d | Score=%.4f (Thresh: %.4f)", 
        //         numerator_count, denominator_count, score, params_.vi_threshold);

        // Termination Criteria
        // Note: We used box_size calculated at the top
        double min_dim = 0.1; 
        bool too_small = (box_size.maxCoeff() < min_dim);

        // if (too_small) {
        //     ROS_WARN("Box too small! MaxDim=%.4f < %.4f", 
        //             box_size.maxCoeff(), min_dim);
        // }

        if (score > params_.vi_threshold || too_small) {
            // ROS_INFO("[Node %s] DECISION: LEAF (Final ID: %d)",idx.c_str(), node->id);
            makeLeaf(node);
        } else {
            // Split (Alg 2 Lines 12-16)
            int d = 0; // 0=x, 1=y, 2=z
            // Using box_size variable created at top
            if (box_size.y() >= box_size.x() && box_size.y() >= box_size.z()) d = 1;
            if (box_size.z() >= box_size.x() && box_size.z() >= box_size.y()) d = 2;

            // std::string axis_name = (d==0 ? "X" : (d==1 ? "Y" : "Z"));
            // ROS_INFO("[Node %s] DECISION: SPLIT on %s-axis", idx.c_str(), axis_name.c_str());

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
            node->left->parent = node;


            node->right = std::make_unique<VINode>();
            node->right->box = right_box;
            node->right->path_idx = idx + "1";
            node->right->id = node_counter++;
            node->right->parent = node;

            // Recursive calls
            bool left_success = buildTreeRecursive(node->left.get(), env_bounds, idx + "0", node_counter, leaf_counter);
            bool right_success = buildTreeRecursive(node->right.get(), env_bounds, idx + "1", node_counter, leaf_counter);

            node->height = std::max(node->left->height, node->right->height) + 1;

            if (!left_success)
                node->left = nullptr;
            if (!right_success)
                node->right = nullptr;
            if ((!left_success) && (!right_success))
                makeLeaf(node);            
            
            ROS_INFO("[Node %s] Finished processing children. Height set to %d", idx.c_str(), node->height);
        }
        return true;
    }

    void makeLeaf(VINode* node, int& leaf_counter) {
        leaf_counter++;
        node->is_leaf = true;
        node->height = 0;
        leaves_.push_back(node);
    }


    /**
     * @brief Recursively checks convexity and marks visibility computed.
     * Call with root_.get() to start the traversal.
     */
    void ComputeTreeVisibility(VINode* node) {
        if (!node) return;

        // 1. Process current node if it meets convexity criteria
        if (node->convexity_score > 0.99) {
            ComputeVisibilityInSiblings(node);
            node->visibility_computed = true;
            std::ofstream log_file("tree_visibility_log.txt", std::ios::app);
            if (log_file.is_open()) {
                log_file << "Node ID: " << node->id << "\n";
                log_file << "  Path Index: " << node->path_idx << "\n";
                log_file << "  Height: " << node->height << "\n";
                log_file << "  Is Leaf: " << (node->is_leaf ? "true" : "false") << "\n";
                log_file << "  Convexity Score: " << node->convexity_score << "\n";
                log_file << "  Visibility Computed: " << (node->visibility_computed ? "true" : "false") << "\n";
                
                // Bounding Box
                log_file << "  Bounding Box: " 
                        << "[" << node->box.x_min << ", " << node->box.x_max << "] x "
                        << "[" << node->box.y_min << ", " << node->box.y_max << "] x "
                        << "[" << node->box.z_min << ", " << node->box.z_max << "]\n";

                // Relationships
                log_file << "  Parent ID: " << (node->parent ? std::to_string(node->parent->id) : "null") << "\n";
                log_file << "  Left Child ID: " << (node->left ? std::to_string(node->left->id) : "null") << "\n";
                log_file << "  Right Child ID: " << (node->right ? std::to_string(node->right->id) : "null") << "\n";

                //  Visibility
                log_file << "  Visible Nodes (Outgoing): [";
                for (size_t i = 0; i < node->visible_from_nodes.size(); ++i) {
                    log_file << "(" << node->visible_from_nodes[i].first << ", " 
                            << node->visible_from_nodes[i].second << ")";
                    if (i < node->visible_from_nodes.size() - 1) log_file << ", ";
                }

                log_file << "]\n";

                log_file << "--------------------------------------------------\n";
                log_file.close();
            }
        }

        // 2. Recursively traverse children
        if (node->left) {
            ComputeTreeVisibility(node->left.get());
        }
        if (node->right) {
            ComputeTreeVisibility(node->right.get());
        }
    }

    /**
     * @brief Traverses the subtree rooted at 'root' to find nodes visible from 'v'.
     * Uses BFS traversal and prunes branches based on visibility score.
     */
    void ComputeNodeVisibilityInSubtree(VINode* v, VINode* root) {
        if (!v || !root) return;

        std::queue<VINode*> q;
        q.push(root);

        int n_samples = params_.num_samples;

        while (!q.empty()) {
            VINode* u = q.front();
            q.pop();

            // 1. Initial Checks: Convexity and Sphere Intersection
            bool u_in_sphere = intersectsWorkspaceSphere(u->box);
            bool v_in_sphere = intersectsWorkspaceSphere(v->box);
            
            // Proceed only if u is convex enough AND at least one node is in the sphere
            if (u->convexity_score > 0.99 && (u_in_sphere || v_in_sphere)) {
                
                // 2. Sample Points
                std::vector<Eigen::Vector3d> S_u, S_v;
                sampler_->sampleInBox(u->box, n_samples, S_u);
                sampler_->sampleInBox(v->box, n_samples, S_v);

                if (S_u.empty() || S_v.empty()) continue;

                // 3. Compute Bi-Directional "Seen By All" Score using GPU
                
                // Combine samples: [S_u ... S_v]
                std::vector<Eigen::Vector3d> combined = S_u;
                combined.insert(combined.end(), S_v.begin(), S_v.end());

                // Upload Matrix
                vis_oracle_->precomputeGpuMatrix(combined);

                // A. Count points in u that see all points in v
                // Guards = S_v (indices from S_u.size() to end)
                std::vector<int> guards_v(S_v.size());
                std::iota(guards_v.begin(), guards_v.end(), (int)S_u.size());
                
                std::vector<bool> u_sees_all_v = vis_oracle_->checkSeenByAllGPU(guards_v);
                
                int count_u_seeing_all_v = 0;
                for (size_t i = 0; i < S_u.size(); ++i) {
                    if (u_sees_all_v[i]) count_u_seeing_all_v++;
                }

                // B. Count points in v that see all points in u
                // Guards = S_u (indices from 0 to S_u.size()-1)
                std::vector<int> guards_u(S_u.size());
                std::iota(guards_u.begin(), guards_u.end(), 0);

                std::vector<bool> v_sees_all_u = vis_oracle_->checkSeenByAllGPU(guards_u);

                int count_v_seeing_all_u = 0;
                for (size_t i = S_u.size(); i < combined.size(); ++i) {
                    if (v_sees_all_u[i]) count_v_seeing_all_u++;
                }

                // 4. Calculate Score (Min of the two ratios)
                double ratio_u = static_cast<double>(count_u_seeing_all_v) / S_u.size();
                double ratio_v = static_cast<double>(count_v_seeing_all_u) / S_v.size();
                double score = std::min(ratio_u, ratio_v);

                // 5. Update and Recurse
                if (score > 0.99) {
                    
                    // Update u's list if v is relevant (intersects sphere)
                    if (v_in_sphere) {
                        u->visible_from_nodes.push_back({v, score});
                    }

                    // Update v's list if u is relevant (intersects sphere)
                    if (u_in_sphere) {
                        v->visible_from_nodes.push_back({u, score});
                    }


                }
                else {
                    // "Recursively" call on children (BFS Enqueue)
                    if (u->left) q.push(u->left.get());
                    if (u->right) q.push(u->right.get());
                }  
            }
        }
    }


    /**
     * @brief Traverses ancestors of v. For each ancestor u, computes visibility between v
     * and u's "other" child (the sibling of the path to v).
     */
    void ComputeVisibilityInSiblings(VINode* v) {
        if (!v) return;

        ROS_WARN("[ComputeTreeVisibility] computing visibility of node with id %d", v->id);

        // 'ancestor_child' tracks the node on the path to v that is a child of u
        VINode* ancestor_child = v;
        VINode* u = v->parent;

        while (u != nullptr) {
            // Determine the sibling: The child of u that is NOT ancestor_child
            VINode* sibling = nullptr;
            if (u->left.get() == ancestor_child) {
                sibling = u->right.get();
            } else {
                sibling = u->left.get();
            }

            // Process only if sibling exists and its visibility hasn't been finalized
            if (sibling && !sibling->visibility_computed) {
                ComputeNodeVisibilityInSubtree(v, sibling);
            }

            // Traverse up towards the root
            ancestor_child = u;
            u = u->parent;
        }
    }




    bool intersectsWorkspaceSphere(BoundingBox& b){
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


    // --- Geometric Helpers for Ball Query ---

    // Squared distance from a point to the closest point on a box
    double distSqPointBox(const Eigen::Vector3d& p, const BoundingBox& b) {
        double dist_sq = 0.0;
        if (p.x() < b.x_min) dist_sq += (p.x() - b.x_min) * (p.x() - b.x_min);
        else if (p.x() > b.x_max) dist_sq += (p.x() - b.x_max) * (p.x() - b.x_max);

        if (p.y() < b.y_min) dist_sq += (p.y() - b.y_min) * (p.y() - b.y_min);
        else if (p.y() > b.y_max) dist_sq += (p.y() - b.y_max) * (p.y() - b.y_max);

        if (p.z() < b.z_min) dist_sq += (p.z() - b.z_min) * (p.z() - b.z_min);
        else if (p.z() > b.z_max) dist_sq += (p.z() - b.z_max) * (p.z() - b.z_max);
        
        return dist_sq;
    }

    // Checks if the Box is FULLY contained within the Ball
    bool isBoxFullyInBall(const BoundingBox& b, const Ball& ball) {
        double r_sq = ball.radius * ball.radius;
        // Check all 8 corners. If any corner is outside, the box is not fully contained.
        // (Optimization: We could check only the furthest corner, but checking 8 is robust/simpler).
        double corners[8][3] = {
            {b.x_min, b.y_min, b.z_min}, {b.x_min, b.y_min, b.z_max},
            {b.x_min, b.y_max, b.z_min}, {b.x_min, b.y_max, b.z_max},
            {b.x_max, b.y_min, b.z_min}, {b.x_max, b.y_min, b.z_max},
            {b.x_max, b.y_max, b.z_min}, {b.x_max, b.y_max, b.z_max}
        };

        for (int i = 0; i < 8; ++i) {
            double dx = corners[i][0] - ball.center.x();
            double dy = corners[i][1] - ball.center.y();
            double dz = corners[i][2] - ball.center.z();
            if (dx*dx + dy*dy + dz*dz > r_sq) return false;
        }
        return true;
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

        // // 3. Process Upper Layers (1 to Max Height)
        // for (auto it = layers.begin(); it != layers.end(); ++it) {
        //     int h = it->first;
        //     if (h == 0) continue; // Already done

        //     std::vector<VINode*>& current_layer = it->second;
        //     std::cout << "Processing Layer " << h << " (" << current_layer.size() << " nodes)..." << std::endl;

        //     for (VINode* node : current_layer) {
        //         if (!node->left || !node->right) continue;

        //         // A. Compute "Seen By This Node" (Outgoing)
        //         // V sees U iff Left sees U AND Right sees U.
        //         // Intersection of sorted vectors.
        //         std::vector<std::pair<int, double>>& left_vis = node->left->visible_from_nodes;
        //         std::vector<std::pair<int, double>>& right_vis = node->right->visible_from_nodes;
                
        //         // (Vectors must be sorted for set_intersection)
        //         // They are guaranteed sorted by our compute functions.
                
        //         // A. Compute "Seen By This Node" (Outgoing) - Custom Intersection
        //         // Both vectors must be sorted by ID (first element) for this to work.
                
        //         auto it_left = left_vis.begin();
        //         auto it_right = right_vis.begin();

        //         while (it_left != left_vis.end() && it_right != right_vis.end()) {
        //             if (it_left->first < it_right->first) {
        //                 ++it_left;
        //             } else if (it_right->first < it_left->first) {
        //                 ++it_right;
        //             } else {
        //                 // Match found! (IDs are equal)
        //                 double avg_weight = (it_left->second + it_right->second) / 2.0;

        //                 if (avg_weight > params_.vi_threshold) {
        //                     node->visible_from_nodes.push_back({it_left->first, avg_weight});
        //                 }

        //                 // Move both forward
        //                 ++it_left;
        //                 ++it_right;
        //             }
        //         }
        //     }
        // }
    }

    void computeLeafLayerVisibility(std::vector<VINode*>& leaves) {
        size_t num_leaves = leaves.size();
        int n_samples = params_.num_samples;

        ROS_INFO("[LeafVis] Starting O(L^2) visibility check for %lu leaves.", num_leaves);

        // We compare every leaf against every other leaf (O(L^2)) on GPU
        for (size_t i = 0; i < num_leaves; ++i) {
            VINode* l1 = leaves[i];
            
            bool l1_intersect_ws = intersectsWorkspaceSphere(l1->box);

            // Periodic logging for outer loop progress
            if (i % 5 == 0 || i == num_leaves - 1) {
                ROS_INFO("[LeafVis] Processing Outer Leaf %d (%lu/%lu)", l1->id, i + 1, num_leaves);
            }


            std::vector<Eigen::Vector3d> S1;
            bool s1_ok = sampler_->sampleInBox(l1->box, n_samples, S1);

            if (!s1_ok || S1.empty()) {
                ROS_WARN("[LeafVis] Leaf %d: Failed to sample S1 (Empty or invalid)", l1->id);
                continue;
            }

            for (size_t j = i + 1; j < num_leaves; ++j) {
                VINode* l2 = leaves[j];

                bool l2_intersect_ws = intersectsWorkspaceSphere(l2->box);

                if (!l1_intersect_ws && !l2_intersect_ws)
                    continue;

                std::vector<Eigen::Vector3d> S2;
                bool s2_ok = sampler_->sampleInBox(l2->box, n_samples, S2);
                
                if (!s2_ok || S2.empty()) {
                    // Log only occasionally to avoid spam if a specific node is problematic
                    // ROS_WARN("[LeafVis] Leaf %d vs %d: Failed to sample S2", l1->id, l2->id);
                    continue;
                }

                // (Optimized: Combine S1+S2, Precompute, check sub-blocks)
                std::pair<double,double> m1m2 = checkBiDirectionalVisibility(S1, S2); // Wrapper helper

                // Check Forward Visibility (l1 -> l2)
                if (m1m2.first > params_.vi_threshold) {
                    if (l2_intersect_ws) {
                        ROS_INFO("[LeafVis] Link ADDED: Leaf %d sees Leaf %d (Score: %.2f)", 
                                l1->id, l2->id, m1m2.first);
                        l1->visible_from_nodes.push_back(std::make_pair(l2, m1m2.first));
                    } else {
                        ROS_INFO("[LeafVis] Link REJECTED: Leaf %d sees Leaf %d (Score: %.2f) but l2 not in TargetSphere", 
                                l1->id, l2->id, m1m2.first);
                    }
                }

                // Check Reverse Visibility (l2 -> l1)
                if (m1m2.second > params_.vi_threshold) {
                    if (l1_intersect_ws) {
                        ROS_INFO("[LeafVis] Link ADDED: Leaf %d sees Leaf %d (Score: %.2f)", 
                                l2->id, l1->id, m1m2.second);
                        l2->visible_from_nodes.push_back(std::make_pair(l1, m1m2.second));
                    } else {
                        ROS_INFO("[LeafVis] Link REJECTED: Leaf %d sees Leaf %d (Score: %.2f) but l1 not in TargetSphere", 
                                l2->id, l1->id, m1m2.second);
                    }
                }
            }

            // Ensure sorted for intersection later
            std::sort(leaves[i]->visible_from_nodes.begin(), leaves[i]->visible_from_nodes.end(), [](const auto& a, const auto& b) {
                return a.first < b.first;
            });
            
            // Log final connection count for this node
            ROS_INFO("[LeafVis] Leaf %d finished. Connected to %lu nodes.", 
                    l1->id, leaves[i]->visible_from_nodes.size());
        }

        ROS_INFO("[LeafVis] Finished computing visibility graph.");
    }



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