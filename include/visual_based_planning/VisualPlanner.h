#pragma once
#include "data_structures/Graph.h"
#include "components/Sampler.h"
#include "components/ValidityChecker.h"

// Aggregates all components
class VisualPlanner {
    GraphManager graph_;
    // include other members...
public:
    VisualPlanner() {}
    
    // Core primitive functions
    void extendEdge(const std::vector<double>& src, const std::vector<double>& target, double resolution) {
        // Implementation
    }

    bool validateEdge(const std::vector<double>& src, const std::vector<double>& target, double resolution) {
        return true;
    }

    // Main Planning functions
    bool planVisRRT() { return true; }
    bool planVisPRM() { return true; }
};