#pragma once

#include <Eigen/Core>

namespace visual_planner {

// Defined here so it is accessible to VisualIK, Sampler, and Planner
struct Ball {
    Eigen::Vector3d center;
    double radius;
};

enum class EdgeCheckMode {
    LINEAR,
    BINARY_SEARCH
};
} // namespace visual_planner




