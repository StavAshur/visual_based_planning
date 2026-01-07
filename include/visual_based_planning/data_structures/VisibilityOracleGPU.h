#pragma once

#include <vector>

namespace visual_planner {

// Simple struct for GPU data transfer to avoid Boost dependencies in CUDA code
struct GPUBox {
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
};

struct GPUPoint {
    double x, y, z;
};

/**
 * @brief Launcher function implemented in the .cu file.
 * * @param d_obstacles Pointer to obstacles already on GPU memory
 * @param num_obstacles Number of obstacles
 * @param p1 Start of segment
 * @param p2 End of segment
 * @return true If visible (no intersection)
 * @return false If occluded (intersection found)
 */
bool launchVisibilityKernel(const GPUBox* d_obstacles, int num_obstacles, 
                            GPUPoint p1, GPUPoint p2);

/**
 * @brief Launches a kernel to check visibility between ALL pairs of points.
 * Optimizes for N*N checks against M obstacles.
 * * @param d_obstacles Pointer to obstacles on GPU
 * @param num_obstacles Number of obstacles
 * @param d_points Pointer to sample points on GPU
 * @param num_points Number of points
 * @param d_results Pointer to output array (size = num_points * num_points). 
 * Will contain 'true' if visible, 'false' if occluded.
 */
void launchAllPairsVisibilityKernel(const GPUBox* d_obstacles, int num_obstacles, 
                                    const GPUPoint* d_points, int num_points, 
                                    bool* d_results);

/**
 * @brief Launches a kernel to check visibility for a specific list of segments.
 * Useful for graph edge validation where we have many P1->P2 pairs.
 * @param d_obstacles Pointer to obstacles on GPU
 * @param num_obstacles Number of obstacles
 * @param d_p1 Pointer to start points of segments (size M)
 * @param d_p2 Pointer to end points of segments (size M)
 * @param num_segments Number of segments (M)
 * @param d_results Output boolean array (size M)
 */
void launchBatchSegmentKernel(const GPUBox* d_obstacles, int num_obstacles, 
                              const GPUPoint* d_p1, const GPUPoint* d_p2, 
                              int num_segments, bool* d_results);
} // namespace visual_planner