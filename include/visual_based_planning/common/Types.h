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

    struct RRTParams {
        double goal_bias = std::pow(0.5, 7);         
        double max_extension = 0.1;      
        int max_iterations = 5000;       
    };

    struct PRMParams {
        int num_neighbors = 10;          
        int num_samples = 1000;  
        int max_goals = 10;
        EdgeCheckMode edge_validation_method = EdgeCheckMode::BINARY_SEARCH;
        int max_size = 10000;
    };

    struct VisibilityIntegrityParams {
        int num_samples = 1000;
        double vi_threshold = 0.7;
        int k_neighbors = 5;
        double limit_diameter_factor = 2.0;
        int face_samples = -1;
    };
    

    struct VisibilityToolParams {
        double beam_angle = M_PI / 12.0;  // Half-angle in radians
        double beam_length = 1.0; 
    };
} // namespace visual_planner



