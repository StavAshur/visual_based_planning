#pragma once

#include <vector>
#include <memory>
#include <limits>
#include <algorithm>
#include "ValidityChecker.h"

class PathSmoother {
private:
    std::shared_ptr<ValidityChecker> validator_;

public:
    PathSmoother(std::shared_ptr<ValidityChecker> validator) : validator_(validator) {}

    /**
     * @brief Smooths the path by finding the shortest combination of valid shortcuts.
     * This essentially runs Dijkstra's algorithm on the graph formed by the waypoints,
     * where an edge exists between index i and j if the straight line path is valid.
     */
    std::vector<std::vector<double>> smoothPath(const std::vector<std::vector<double>>& input_path) {
        if (input_path.size() < 3) {
            return input_path; // Can't smooth a line or a point
        }

        size_t n = input_path.size();
        
        // dp[i] stores the minimum cost (distance) to reach node i from node 0
        std::vector<double> cost(n, std::numeric_limits<double>::infinity());
        
        // parent[i] stores the index of the predecessor of i in the optimal path
        std::vector<int> parent(n, -1);

        cost[0] = 0.0;

        for (int i = 0; i < n; ++i) {
            // If this node is unreachable, we can't extend from it
            if (cost[i] == std::numeric_limits<double>::infinity()) continue;

            // Look ahead to all future nodes
            for (int j = i + 1; j < n; ++j) {
                // Calculate distance
                double segment_dist = validator_->distance(input_path[i], input_path[j]);
                double new_cost = cost[i] + segment_dist;

                // Optimization: If this path isn't shorter than what we already found for j, skip checking collision
                if (new_cost >= cost[j]) continue;

                // Check if the shortcut is valid
                if (validator_->validateEdge(input_path[i], input_path[j])) {
                    // We found a better path to j
                    ROS_INFO("Found a short cut between nodes %d and %d in the path!", i, j);
                    cost[j] = new_cost;
                    parent[j] = i;
                }
            }
        }

        // Reconstruct the path backwards
        std::vector<std::vector<double>> smoothed_path;
        int curr = n - 1;
        
        // If end is unreachable (shouldn't happen if input path was valid), return input
        if (cost[curr] == std::numeric_limits<double>::infinity()) {
             return input_path;
        }

        while (curr != -1) {
            smoothed_path.push_back(input_path[curr]);
            curr = parent[curr];
        }

        std::reverse(smoothed_path.begin(), smoothed_path.end());
        return smoothed_path;
    }
};