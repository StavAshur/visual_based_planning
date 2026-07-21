#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>

#include "VisPRMPlanner.h"
#include "VisRRTPlanner.h"
#include "VisibilityPlannerBase.h"

namespace visual_planner {

/**
 * @brief Everything a planner needs configured that is not part of the shared context.
 *
 * Grouped into one struct so a caller can describe a planner in a single value and the
 * factory can apply the planner-specific pieces (VisRRT's VisualIK snap) only where
 * they mean something. Without this the caller would have to downcast the returned
 * base pointer to reach those setters.
 */
struct PlannerOptions {
    bool use_visual_ik = true;            ///< VisRRT only: attempt the VisualIK snap.
    bool use_visibility_integrity = true; ///< Draw goal samples from the visibility structure.
    bool shortcutting = true;
    int time_cap = 120;
    RRTParams rrt;
    PRMParams prm;
};

/**
 * @brief Builds the planner named by `mode` on the given shared context.
 *
 * Adding a planner means adding a branch here rather than touching the caller's
 * control flow. Returns nullptr for an unrecognised mode so the caller can decide
 * whether to fall back or fail.
 *
 * @param mode One of "VisRRT" or "VisPRM".
 * @param ctx  The shared world. Several planners may hold the same one, which is what
 *             lets a mode switch avoid rebuilding the VI-tree.
 */
inline std::unique_ptr<VisibilityPlannerBase> createPlanner(
        const std::string& mode,
        const std::shared_ptr<PlanningContext>& ctx,
        const PlannerOptions& opts = PlannerOptions())
{
    std::unique_ptr<VisibilityPlannerBase> planner;

    if (mode == "VisRRT") {
        planner.reset(new VisRRTPlanner(ctx, opts.use_visual_ik));
    } else if (mode == "VisPRM") {
        planner.reset(new VisPRMPlanner(ctx));
    } else {
        ROS_ERROR("[PlannerFactory] Unknown planner mode '%s'.", mode.c_str());
        return nullptr;
    }

    planner->setShortcutting(opts.shortcutting);
    planner->setTimeCap(opts.time_cap);
    planner->setRRTParams(opts.rrt);
    planner->setPRMParams(opts.prm);
    planner->setUseVisibilityIntegrity(opts.use_visibility_integrity);

    return planner;
}

} // namespace visual_planner
