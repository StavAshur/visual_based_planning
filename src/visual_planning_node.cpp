#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Include your headers
#include "visual_based_planning/VisualPlanner.h"
#include "visual_based_planning/PlanVisibilityPath.h"

// Service Callback
bool planCallback(visual_based_planning::PlanVisibilityPath::Request &req,
                  visual_based_planning::PlanVisibilityPath::Response &res) {
    ROS_INFO("Received planning request");
    
    // Instantiate your planner here or use a global one
    VisualPlanner planner;
    
    if (req.planner_type == "VisRRT") {
        res.success = planner.planVisRRT();
    } else {
        res.success = planner.planVisPRM();
    }
    
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_planning_node");
    ros::NodeHandle nh;

    // Load MoveIt Planning Scene Monitor
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    ros::ServiceServer service = nh.advertiseService("plan_visibility_path", planCallback);
    
    ROS_INFO("Visual Planning Service Ready");
    ros::spin();
    return 0;
}