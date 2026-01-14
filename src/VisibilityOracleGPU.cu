// In: src/visual_based_planning/src/VisibilityOracleGPU.cu

#include <visual_based_planning/data_structures/VisibilityOracleGPU.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cstdio>

namespace visual_planner {

// --- Helper: Slab Method (Device Only) ---
__device__ bool checkSlabIntersection(const GPUBox& box, const GPUPoint& p1, const GPUPoint& p2) {
    double dir_x = p2.x - p1.x;
    double dir_y = p2.y - p1.y;
    double dir_z = p2.z - p1.z;

    double t_min = 0.0;
    double t_max = 1.0;

    // X Axis
    if (abs(dir_x) < 1e-9) { 
        if (p1.x < box.min_x || p1.x > box.max_x) return false;
    } else {
        double inv_dir = 1.0 / dir_x;
        double t1 = (box.min_x - p1.x) * inv_dir;
        double t2 = (box.max_x - p1.x) * inv_dir;
        if (t1 > t2) { double temp = t1; t1 = t2; t2 = temp; }
        t_min = max(t_min, t1);
        t_max = min(t_max, t2);
        if (t_min > t_max) return false;
    }

    // Y Axis
    if (abs(dir_y) < 1e-9) {
        if (p1.y < box.min_y || p1.y > box.max_y) return false;
    } else {
        double inv_dir = 1.0 / dir_y;
        double t1 = (box.min_y - p1.y) * inv_dir;
        double t2 = (box.max_y - p1.y) * inv_dir;
        if (t1 > t2) { double temp = t1; t1 = t2; t2 = temp; }
        t_min = max(t_min, t1);
        t_max = min(t_max, t2);
        if (t_min > t_max) return false;
    }

    // Z Axis
    if (abs(dir_z) < 1e-9) {
        if (p1.z < box.min_z || p1.z > box.max_z) return false;
    } else {
        double inv_dir = 1.0 / dir_z;
        double t1 = (box.min_z - p1.z) * inv_dir;
        double t2 = (box.max_z - p1.z) * inv_dir;
        if (t1 > t2) { double temp = t1; t1 = t2; t2 = temp; }
        t_min = max(t_min, t1);
        t_max = min(t_max, t2);
        if (t_min > t_max) return false;
    }

    return true; // Intersection found
}

// --- Kernel 1: Single Segment (Existing) ---
__global__ void checkIntersectionKernel(const GPUBox* obstacles, int num_obstacles, 
                                        GPUPoint p1, GPUPoint p2, bool* d_collision_found) {
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx >= num_obstacles || *d_collision_found) return;

    if (checkSlabIntersection(obstacles[idx], p1, p2)) {
        *d_collision_found = true;
    }
}

// --- Kernel 2: All Pairs  ---
__global__ void allPairsVisibilityKernel(const GPUBox* obstacles, int num_obstacles, 
                                         const GPUPoint* points, int num_points, 
                                         bool* results) {
    // Map thread to a specific pair (i, j)
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    int total_pairs = num_points * num_points;

    if (idx >= total_pairs) return;

    int i = idx / num_points;
    int j = idx % num_points;

    // 1. Trivial cases
    if (i == j) {
        results[idx] = true;
        return;
    }

    // 2. Symmetric Optimization: Only compute for i < j
    // The CPU will mirror the results later.
    if (i > j) return;

    GPUPoint p1 = points[i];
    GPUPoint p2 = points[j];

    // 3. Check against all obstacles
    // Note: For very large obstacle sets, this inner loop might be slow.
    // However, for N=2000 points and M=100 obstacles, this is perfectly fine.
    for (int k = 0; k < num_obstacles; ++k) {
        if (checkSlabIntersection(obstacles[k], p1, p2)) {
            results[idx] = false; // Occluded
            return;
        }
    }

    results[idx] = true; // Visible
}


// --- Kernel 3: Batch Segment List ---
__global__ void batchSegmentKernel(const GPUBox* obstacles, int num_obstacles, 
                                   const GPUPoint* p1_list, const GPUPoint* p2_list, 
                                   int num_segments, bool* results) {
    // Each thread handles ONE segment from the list
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    
    if (idx >= num_segments) return;

    GPUPoint p1 = p1_list[idx];
    GPUPoint p2 = p2_list[idx];

    // Default to visible
    bool visible = true;

    // Check this single segment against all obstacles
    for (int k = 0; k < num_obstacles; ++k) {
        // Uses the helper we defined earlier
        if (checkSlabIntersection(obstacles[k], p1, p2)) {
            visible = false;
            break; // Early exit on collision
        }
    }
    
    results[idx] = visible;
}



// --- Kernel 4: Seen By All Reduction ---
__global__ void seenByAllKernel(const bool* matrix, int num_points,
                                const int* query_indices, int num_query_indices,
                                bool* results) {
    // Thread ID corresponds to the 'candidate' point we are checking
    int candidate_idx = blockDim.x * blockIdx.x + threadIdx.x;

    if (candidate_idx >= num_points) return;

    bool seen_by_all = true;

    // Check visibility against every Query Index (Guard)
    for (int k = 0; k < num_query_indices; ++k) {
        int guard_idx = query_indices[k];

        // Trivial case: A point always sees itself
        if (guard_idx == candidate_idx) continue;

        // SMART INDEXING:
        // Because allPairsVisibilityKernel only fills the upper triangle (i <= j),
        // we must ensure we always read from the smaller index to the larger index.
        int r = min(guard_idx, candidate_idx);
        int c = max(guard_idx, candidate_idx);
        
        // Matrix is row-major: index = row * N + col
        if (!matrix[r * num_points + c]) {
            seen_by_all = false;
            break; // Failed "seen by all" condition, stop checking
        }
    }

    results[candidate_idx] = seen_by_all;
}


// --- Kernel 5: Seen By Any (OR Reduction) ---
__global__ void seenByAnyKernel(const bool* matrix, int num_points,
                                const int* query_indices, int num_query_indices,
                                bool* results) {
    int candidate_idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (candidate_idx >= num_points) return;

    bool seen_by_any = false;

    for (int k = 0; k < num_query_indices; ++k) {
        int guard_idx = query_indices[k];
        
        // A point always sees itself
        if (guard_idx == candidate_idx) { 
            seen_by_any = true; 
            break; 
        }

        int r = min(guard_idx, candidate_idx);
        int c = max(guard_idx, candidate_idx);
        
        if (matrix[r * num_points + c]) {
            seen_by_any = true;
            break; // Found one! Optimization: Stop checking.
        }
    }
    results[candidate_idx] = seen_by_any;
}



// --- Wrapper Implementation ---
void launchBatchSegmentKernel(const GPUBox* d_obstacles, int num_obstacles, 
                              const GPUPoint* d_p1, const GPUPoint* d_p2, 
                              int num_segments, bool* d_results) {
    
    int threadsPerBlock = 256;
    int blocksPerGrid = (num_segments + threadsPerBlock - 1) / threadsPerBlock;

    batchSegmentKernel<<<blocksPerGrid, threadsPerBlock>>>(d_obstacles, num_obstacles, d_p1, d_p2, num_segments, d_results);
    
    // Error checking
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA Error [BatchSegment]: %s\n", cudaGetErrorString(err));
    }
    cudaDeviceSynchronize();
}


void launchAllPairsVisibilityKernel(const GPUBox* d_obstacles, int num_obstacles, 
                                    const GPUPoint* d_points, int num_points, 
                                    bool* d_results) {
    int total_threads = num_points * num_points;
    int threadsPerBlock = 256;
    int blocksPerGrid = (total_threads + threadsPerBlock - 1) / threadsPerBlock;

    allPairsVisibilityKernel<<<blocksPerGrid, threadsPerBlock>>>(d_obstacles, num_obstacles, d_points, num_points, d_results);
    
    // Check for launch errors
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA Error: %s\n", cudaGetErrorString(err));
    }
    cudaDeviceSynchronize();
}

bool launchVisibilityKernel(const GPUBox* d_obstacles, int num_obstacles, 
                            GPUPoint p1, GPUPoint p2) {
    
    // 1. Allocate device flag for result
    bool* d_collision_found;
    bool h_collision_found = false;
    
    cudaMalloc((void**)&d_collision_found, sizeof(bool));
    cudaMemcpy(d_collision_found, &h_collision_found, sizeof(bool), cudaMemcpyHostToDevice);

    // 2. Launch Kernel
    int threadsPerBlock = 256;
    int blocksPerGrid = (num_obstacles + threadsPerBlock - 1) / threadsPerBlock;

    checkIntersectionKernel<<<blocksPerGrid, threadsPerBlock>>>(d_obstacles, num_obstacles, p1, p2, d_collision_found);

    // 3. Retrieve Result
    cudaMemcpy(&h_collision_found, d_collision_found, sizeof(bool), cudaMemcpyDeviceToHost);
    cudaFree(d_collision_found);

    // If collision found, visibility is FALSE
    return !h_collision_found;
}


// --- Wrapper for Seen By All ---
void launchComputeSeenByAll(const bool* d_visibility_matrix, int num_points,
                            const int* d_query_indices, int num_query_indices,
                            bool* d_results) {
    
    int threadsPerBlock = 256;
    int blocksPerGrid = (num_points + threadsPerBlock - 1) / threadsPerBlock;

    seenByAllKernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_visibility_matrix, 
        num_points, 
        d_query_indices, 
        num_query_indices, 
        d_results
    );

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA Error [SeenByAll]: %s\n", cudaGetErrorString(err));
    }
    cudaDeviceSynchronize();
}


// --- Wrapper for Seen By Any ---
void launchComputeSeenByAny(const bool* d_visibility_matrix, int num_points,
                            const int* d_query_indices, int num_query_indices,
                            bool* d_results) {
    int threadsPerBlock = 256;
    int blocksPerGrid = (num_points + threadsPerBlock - 1) / threadsPerBlock;

    seenByAnyKernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_visibility_matrix, num_points, d_query_indices, num_query_indices, d_results
    );
    cudaDeviceSynchronize();
}

} // namespace visual_planner