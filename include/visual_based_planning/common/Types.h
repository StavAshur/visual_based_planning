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

    struct BoundingBox {
        double x_min, x_max;
        double y_min, y_max;
        double z_min, z_max;
    };

} // namespace visual_planner



