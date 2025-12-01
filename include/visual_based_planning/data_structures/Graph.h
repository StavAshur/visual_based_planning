#pragma once
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

struct GraphVertex {
    std::vector<double> joint_config;
};

struct GraphEdge {
    double weight;
};

// Define the Boost Graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, GraphVertex, GraphEdge> PlannerGraph;
typedef boost::graph_traits<PlannerGraph>::vertex_descriptor VertexDesc;

class GraphManager {
public:
    PlannerGraph g;
    
    VertexDesc addVertex(const std::vector<double>& q) {
        VertexDesc v = boost::add_vertex(g);
        g[v].joint_config = q;
        return v;
    }

    void addEdge(VertexDesc u, VertexDesc v, double weight) {
        GraphEdge e; e.weight = weight;
        boost::add_edge(u, v, e, g);
    }

    std::vector<VertexDesc> shortestPath(VertexDesc start, VertexDesc goal) {
        // Placeholder for Dijkstra/A*
        return std::vector<VertexDesc>();
    }
};