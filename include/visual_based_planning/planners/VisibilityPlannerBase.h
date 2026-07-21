#pragma once

#include <vector>
#include <memory>
#include <random>
#include <map>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "PlanningContext.h"
#include "../data_structures/Graph.h"
#include "../data_structures/NearestNeighbor.h"
#include "../common/Types.h"

namespace visual_planner {

/**
 * @brief Common base for the visibility-based planners.
 *
 * Holds the state of one planning run -- the graph and its nearest-neighbour index,
 * the start configuration, the target, and the resulting path -- plus the primitives
 * every planner is written in terms of: edge validation, extension, graph insertion,
 * visibility-guided goal sampling and path finalization.
 *
 * The world itself lives in a shared PlanningContext, so constructing a planner is
 * cheap and several planners can run against one already-built VI-tree. Derived
 * classes implement plan().
 *
 * Both RRTParams and PRMParams live here rather than on the individual planners
 * because the strategies mix: VisRRG extends like an RRT and connects to k nearest
 * neighbours like a PRM, so it needs both.
 */
class VisibilityPlannerBase {
protected:
    std::shared_ptr<PlanningContext> ctx_;

    // --- Per-run configuration ---
    bool shortcutting_;
    bool use_visibility_integrity_;   ///< Use visibility-guided goal sampling.
    int time_cap_;
    RRTParams rrt_params_;
    PRMParams prm_params_;

    // --- Per-run state ---
    Ball target_mes_;
    std::vector<double> start_joint_values_;
    std::vector<std::vector<double>> result_path_;
    GraphManager graph_;
    NearestNeighbor nn_;
    VertexDesc root_id_;
    std::map<VertexDesc,bool> checked_vertices_;
    std::mt19937 rng_;

public:
    explicit VisibilityPlannerBase(std::shared_ptr<PlanningContext> ctx)
        : ctx_(std::move(ctx)),
          shortcutting_(true),
          use_visibility_integrity_(true),
          time_cap_(120),
          rng_(std::random_device{}()),
          root_id_(-1),
          checked_vertices_()
    {
        // Default the start to wherever the robot currently is; callers normally
        // override this with setStartJoints() before planning.
        start_joint_values_ = ctx_->getCurrentJoints();
    }

    virtual ~VisibilityPlannerBase() = default;

    /// Runs the planner. Result is retrieved with getResultPath().
    virtual bool plan() = 0;

    std::shared_ptr<PlanningContext> getContext() const { return ctx_; }

    // ========================================================================
    // Configuration owned by the planner
    // ========================================================================

    void setShortcutting(bool enable) { shortcutting_ = enable; }
    void setRRTParams(const RRTParams& params) { rrt_params_ = params; }
    void setPRMParams(const PRMParams& params) { prm_params_ = params; }
    void setTimeCap(int time_cap) { time_cap_ = time_cap; }

    void setStartJoints(const std::vector<double>& start) { start_joint_values_ = start; }
    const std::vector<double>& getStartJoints() const { return start_joint_values_; }

    const std::vector<std::vector<double>>& getResultPath() const { return result_path_; }

    /**
     * @brief Enables visibility-guided goal sampling, and ensures the context has a
     *        structure to sample from.
     *
     * The two are driven together so that turning guidance on cannot leave the
     * planner querying a structure that was never built.
     */
    void setUseVisibilityIntegrity(bool enable) {
        use_visibility_integrity_ = enable;
        ctx_->setUseVisibilityStructure(enable);
    }

    // ========================================================================
    // Configuration forwarded to the shared context
    // ========================================================================

    /**
     * @brief Gives the nearest-neighbour index a scene, without touching the context.
     *
     * Required before planning: NearestNeighbor builds its distance metric from the
     * scene's joint model group, and returns no neighbours at all while it has none.
     * Kept separate from setPlanningScene() so a planner constructed against an
     * already-built context can be made ready without rebuilding the VI-tree.
     */
    void attachScene(const planning_scene::PlanningScenePtr& scene) {
        nn_.setPlanningScene(scene, ctx_->group_name_);
    }

    /// Rebuilds the shared context for a new scene, then attaches this planner to it.
    void setPlanningScene(const planning_scene::PlanningScenePtr& scene) {
        ctx_->setPlanningScene(scene);
        attachScene(scene);
    }

    void setWorkspaceBounds(const BoundingBox& b)               { ctx_->setWorkspaceBounds(b); }
    void setVisibilityToolParams(const VisibilityToolParams& p) { ctx_->setVisibilityToolParams(p); }
    void setVisibilityThreshold(double t)                       { ctx_->setVisibilityThreshold(t); }
    void setVisibilityIntegrityParams(const VisibilityIntegrityParams& p) { ctx_->setVisibilityIntegrityParams(p); }
    void setUseVisibilityRoadmap(bool enable)                   { ctx_->setUseVisibilityRoadmap(enable); }
    void setResolution(double res)                              { ctx_->setResolution(res); }
    void setGroupName(const std::string& group)                 { ctx_->setGroupName(group); }
    void setEELinkName(const std::string& ee_link)              { ctx_->setEELinkName(ee_link); }

    std::string getGroupName() const  { return ctx_->getGroupName(); }
    std::string getEELinkName() const { return ctx_->getEELinkName(); }
    bool isInitialized() const        { return ctx_->isInitialized(); }

    Eigen::Isometry3d solveFK(const std::vector<double>& joints) { return ctx_->solveFK(joints); }

    // --- Accessors ---
    GraphManager& getGraph()               { return graph_; }
    NearestNeighbor& getNN()               { return nn_; }
    VisualIK& getVisualIK()                { return ctx_->getVisualIK(); }
    PathSmoother& getSmoother()            { return ctx_->getSmoother(); }
    ValidityChecker& getValidityChecker()  { return ctx_->getValidityChecker(); }
    Sampler& getSampler()                  { return ctx_->getSampler(); }
    Ball getTargetMES() const              { return target_mes_; }

    // ========================================================================
    // Primitives
    // ========================================================================

    std::vector<double> interpolate(const std::vector<double>& start, const std::vector<double>& end, double t) {
        return ctx_->validity_checker_->interpolate(start, end, t);
    }

    double distance(const std::vector<double>& a, const std::vector<double>& b) {
        return ctx_->validity_checker_->distance(a, b);
    }

    bool validateEdge(const std::vector<double>& start, const std::vector<double>& end, EdgeCheckMode mode = EdgeCheckMode::BINARY_SEARCH) {
        return ctx_->validity_checker_->validateEdge(start, end, mode);
    }

    std::vector<double> extend(const std::vector<double>& start, const std::vector<double>& goal, double max_step = -1.0) {
        double step = (max_step < 0) ? rrt_params_.max_extension : max_step;
        return ctx_->validity_checker_->extend(start, goal, step);
    }

    // ========================================================================
    // 2. Planner Operations
    // ========================================================================

    void reset() {
        ROS_WARN("Clearing roadmap graph and NN tree.");
        nn_.clear();
        graph_.clear(); 
    }

    void computeTargetMES(const std::vector<geometry_msgs::Point>& targets) {
        if (targets.empty()) {
            target_mes_ = {Eigen::Vector3d::Zero(), 0.0};
        } else {
            Eigen::Vector3d centroid(0, 0, 0);
            for (const auto& p : targets) {
                centroid += Eigen::Vector3d(p.x, p.y, p.z);
            }
            centroid /= targets.size();

            double max_dist_sq = 0.0;
            for (const auto& p : targets) {
                Eigen::Vector3d pt(p.x, p.y, p.z);
                double d_sq = (pt - centroid).squaredNorm();
                if (d_sq > max_dist_sq) {
                    max_dist_sq = d_sq;
                }
            }
            target_mes_.center = centroid;
            target_mes_.radius = std::sqrt(max_dist_sq);
        }
        ROS_INFO("Target MES has center = (%f,%f,%f) and radius = %f",
                target_mes_.center.x(), target_mes_.center.y(), target_mes_.center.z(), target_mes_.radius);
    }

    VertexDesc addState(const std::vector<double>& q, bool compute_ee_pose = true) {
        Eigen::Isometry3d ee_pose;
        if (compute_ee_pose){
            ee_pose = solveFK(q);
        }
        else
            ee_pose = Eigen::Isometry3d::Identity();
        VertexDesc v = graph_.addVertex(q, ee_pose);
        nn_.addPoint(q, v);
        return v;
    }
    VertexDesc connectToGraph(std::vector<double>& q_connect, int k ,VertexDesc v_id = -1) {
        bool connected = false;
        VertexDesc res = v_id; // Default res to v_id
         if (v_id == root_id_){
            // ROS_WARN("VisPRM: Trying to connect start %zu vertex to graph", root_id_);
            k = k + checked_vertices_.size(); // Increase k to account for already checked vertices
	    } 

        std::vector<VertexDesc> neighbors = nn_.kNearest(q_connect, k);
        
        for (auto n_id : neighbors) {
            if (v_id == root_id_) {
                // ROS_WARN("VisPRM: Trying to connect start vertex to vertex %zu", n_id);
                if (checked_vertices_[n_id]) {
                    continue;
                }
                checked_vertices_[n_id] = true;
            }
            if (n_id == res || graph_.isEdge(n_id, res)) continue; 

            std::vector<double> q_neighbor = graph_.getVertexConfig(n_id); 

            if (validateEdge(q_connect, q_neighbor, prm_params_.edge_validation_method)) {
                // If the state hasn't been added to the graph yet, add it ONCE
                // if (v_id == root_id_)
                    // ROS_WARN("VisPRM: Connected start vertex to vertex %zu", n_id);
                if (res == -1) {
                    res = addState(q_connect);
                }
                
                // Now safely draw the edge
                graph_.addEdge(res, n_id, distance(q_connect, q_neighbor));
                connected = true;
            }
        }
        return connected ? res : -1;
    }

    /**
     * @brief Samples a configuration whose end-effector sees the target.
     *
     * Asks the context's active visibility structure for a workspace point that sees
     * the target, orients the tool toward it, and solves IK. Retries until a pose is
     * both sampled and reachable.
     */
    bool sampleVisibilityGoal(std::vector<double>& res_sample, int attempts = 100) {
        Eigen::Vector3d sample_pos;

        for (int i = 0; i < attempts; i++) {
            if (!ctx_->sampleVisibilityRegion(target_mes_, sample_pos)) continue;

            Eigen::Matrix3d sample_ori = ctx_->sampler_->computeLookAtRotation(sample_pos, target_mes_.center);

            Eigen::Isometry3d sample_pose;
            sample_pose.translation() = sample_pos;
            sample_pose.linear() = sample_ori;

            std::vector<double> ik_seed = ctx_->sampler_->sampleUniform();

            if (ctx_->vis_ik_->solveIK(sample_pose, ik_seed, res_sample)) {
                return true;
            }
        }
        return false;
    }

    // Helper to extract and smooth path
    void finalizePath(VertexDesc start, VertexDesc goal) {
        std::vector<VertexDesc> path_idx = graph_.shortestPath(start, goal);
        std::vector<std::vector<double>> raw_path;
        raw_path.reserve(path_idx.size());
        
        for (auto v : path_idx) {
            raw_path.push_back(graph_.getVertexConfig(v));
        }

        if (shortcutting_) {
            ROS_INFO("Smoothing path with shortcuts...");
            result_path_ = ctx_->path_smoother_->smoothPath(raw_path);
        } else {
            result_path_ = raw_path;
        }

        if (raw_path.size() >= 2) {
            for (size_t i = 0; i < raw_path.size() - 1; ++i) {
                if (!ctx_->validity_checker_->validateEdge(raw_path[i], raw_path[i+1])) {
                    ROS_ERROR("VisibilityPlannerBase: Final path validation FAILED at edge %lu -> %lu. The path contains collision!", i, i+1);
                    // Depending on policy, we might want to clear result_path_ or just warn.
                    // For now, we warn but proceed to smoothing if enabled (which might fix or fail).
                }
            }
        }

    }
};

} // namespace visual_planner
