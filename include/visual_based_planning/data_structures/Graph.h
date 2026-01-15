#pragma once
#include <vector>
#include <algorithm> // for std::reverse
#include <limits>    // for infinity
#include <iostream>  // for logging

#include <Eigen/Dense>
#include <Eigen/Geometry> // Required for Eigen::Isometry3d

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>

// Vertex Property
struct GraphVertex {
    std::vector<double> joint_config;
    Eigen::Isometry3d ee_pose;
};

// Edge Property
struct GraphEdge {
    double weight;
};

// Define the Boost Graph with vecS (std::vector) for storage
// This allows O(1) access by index
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, GraphVertex, GraphEdge> PlannerGraph;
typedef boost::graph_traits<PlannerGraph>::vertex_descriptor VertexDesc;
typedef boost::graph_traits<PlannerGraph>::edge_descriptor EdgeDesc;

class GraphManager {
public:
    PlannerGraph G_;
    
    // --- Modifiers ---

    /**
     * @brief Adds a vertex with a configuration and an end-effector pose.
     * @param q The joint configuration.
     * @param pose The Isometry3d pose associated with this vertex (e.g., end-effector pose).
     */
    VertexDesc addVertex(const std::vector<double>& q, const Eigen::Isometry3d& pose) {
        VertexDesc v = boost::add_vertex(G_);
        G_[v].joint_config = q;
        G_[v].ee_pose = pose; // Store the pose
        return v;
    }

    void addEdge(VertexDesc u, VertexDesc v, double weight) {
        GraphEdge e; e.weight = weight;
        boost::add_edge(u, v, e, G_);
        boost::add_edge(v, u, e, G_);
    }

    // --- Getters ---

    // Get the Joint Configuration of a specific node
    std::vector<double> getVertexConfig(VertexDesc v) const {
        return G_[v].joint_config;
    }

    // Get the End-Effector Pose of a specific node
    Eigen::Isometry3d getVertexPose(VertexDesc v) const {
        return G_[v].ee_pose;
    }

    // Get the number of vertices
    size_t getNumVertices() const {
        return boost::num_vertices(G_);
    }

    // Get number of edges
    size_t getNumEdges() const {
        return boost::num_edges(G_);
    }

    /**
     * @brief Checks if an edge exists between two vertices.
     * Uses Boost Graph's native edge lookup.
     */
    bool isEdge(VertexDesc u, VertexDesc v) {
        // boost::edge returns a pair<edge_descriptor, bool>
        std::pair<EdgeDesc, bool> result = boost::edge(u, v, G_);
        return result.second;
    }

    /**
     * @brief Finds a vertex that matches the given joint configuration.
     * @param q The joint configuration to look for.
     * @param epsilon The tolerance for floating point comparison.
     * @return A pair containing the VertexDesc and a bool (true if found, false otherwise).
     * * NOTE: This performs a linear search O(N). For fast lookups, 
     * use the NearestNeighbor structure instead.
     */
    std::pair<VertexDesc, bool> getVertex(const std::vector<double>& q, double epsilon = 1e-5) {
        // Iterate over all vertices in the graph
        auto v_range = boost::vertices(G_);
        for (auto it = v_range.first; it != v_range.second; ++it) {
            VertexDesc v = *it;
            const std::vector<double>& stored_q = G_[v].joint_config;

            if (stored_q.size() != q.size()) continue;

            bool match = true;
            for (size_t i = 0; i < q.size(); ++i) {
                if (std::abs(stored_q[i] - q[i]) > epsilon) {
                    match = false;
                    break;
                }
            }

            if (match) {
                return std::make_pair(v, true);
            }
        }

        // Return a dummy descriptor and false if not found
        // In BGL vecS, descriptors are usually integers, but we shouldn't rely on magic numbers.
        return std::make_pair(VertexDesc(), false);
    }
    // --- Algorithms ---

    /**
     * @brief Finds the shortest path between start and goal using Dijkstra's algorithm.
     * @return A vector of Vertex descriptors representing the path. Empty if no path found.
     */
    std::vector<VertexDesc> shortestPath(VertexDesc start, VertexDesc goal) {
        size_t num_nodes = boost::num_vertices(G_);
        
        // 1. Prepare vectors to store results
        std::vector<VertexDesc> predecessors(num_nodes);
        std::vector<double> distances(num_nodes);

        // 2. Run Boost Dijkstra
        // We use weight_map to tell BGL to look at our GraphEdge::weight struct member
        boost::dijkstra_shortest_paths(G_, start,
            boost::predecessor_map(&predecessors[0])
            .distance_map(&distances[0])
            .weight_map(boost::get(&GraphEdge::weight, G_))
        );

        // 3. Check if path exists
        // If distance is max (infinity), the goal is unreachable
        if (distances[goal] >= std::numeric_limits<double>::max()) {
            return {}; // Return empty vector
        }

        // 4. Reconstruct Path (Backtracking from goal to start)
        std::vector<VertexDesc> path;
        VertexDesc current = goal;

        while (current != start) {
            path.push_back(current);
            current = predecessors[current];
            
            // Safety check against cycles or disconnected components not caught by distance check
            // In a valid Dijkstra run on undirected graph, this shouldn't happen if distance < max
            if (current == predecessors[current] && current != start) {
                // This means we hit a root that isn't start (unreachable)
                return {};
            }
        }
        path.push_back(start);

        // 5. Reverse to get Start -> Goal order
        std::reverse(path.begin(), path.end());

        return path;
    }

    void clear() {
        G_.clear();
    }

/**
     * @brief Computes the number of connected components and prints a representative for each.
     * @return The number of disjoint subgraphs.
     */
    int countConnectedComponents() {
        if (boost::num_vertices(G_) == 0) return 0;
        
        // Map vertex ID -> Component ID
        std::vector<int> component(boost::num_vertices(G_));
        int num_components = boost::connected_components(G_, &component[0]);
        
        std::cout << "[GraphManager] Total Connected Components: " << num_components << std::endl;

        // Track which component IDs we have already printed
        std::vector<bool> printed(num_components, false);
        int count_printed = 0;

        // Iterate through all vertices to find one representative per component
        for (size_t v = 0; v < component.size(); ++v) {
            int c_id = component[v];
            
            // If we haven't printed a representative for this component yet
            if (!printed[c_id]) {
                printed[c_id] = true;
                count_printed++;

                // Print the representative's configuration
                std::cout << "  - Component " << c_id << " Rep (Node " << v << "): [";
                const auto& q = G_[v].joint_config;
                for (size_t i = 0; i < q.size(); ++i) {
                    std::cout << q[i] << (i < q.size() - 1 ? ", " : "");
                }
                std::cout << "]" << std::endl;

                // Optimization: Stop if we've found all representatives
                if (count_printed == num_components) break;
            }
        }
        
        return num_components;
    }

};