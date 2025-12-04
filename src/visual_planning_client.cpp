#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visual_based_planning/PlanVisibilityPath.h>

// Use your existing PathExecuter component
#include "visual_based_planning/components/PathExecuter.h"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_planning_client");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~"); // Private node handle for params

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 1. Setup Service Client
  ros::ServiceClient client =
      nh.serviceClient<visual_based_planning::PlanVisibilityPath>("plan_visibility_path");

  ROS_INFO("Waiting for planning service...");
  client.waitForExistence();
  ROS_INFO("Service available.");

  // 2. Load Parameters
  std::string yaml_path;
  if (!pnh.getParam("targets_yaml", yaml_path)) {
    // Fallback or error
    if (!nh.getParam("targets_yaml", yaml_path)) {
        ROS_ERROR("Missing param 'targets_yaml'.");
        return 1;
    }
  }
  
  std::string planner_type;
  pnh.param<std::string>("planner_type", planner_type, "VisRRT"); // Default to RRT

  // 3. Load Targets from YAML
  YAML::Node config;
  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to load YAML file: %s", e.what());
    return 1;
  }

  if (!config["target_points"]) {
    ROS_ERROR("YAML missing 'target_points'");
    return 1;
  }

  // 4. Prepare Request
  visual_based_planning::PlanVisibilityPath srv;
  srv.request.planner_type = planner_type;

  YAML::Node pts = config["target_points"];
  for (std::size_t i = 0; i < pts.size(); ++i) {
    geometry_msgs::Point p;
    p.x = pts[i][0].as<double>();
    p.y = pts[i][1].as<double>();
    p.z = pts[i][2].as<double>();
    srv.request.task.target_points.push_back(p);
  }

  ROS_INFO("Requesting path for %lu targets using %s...", srv.request.task.target_points.size(), planner_type.c_str());

  // 5. Call Service
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call service plan_visibility_path");
    return 1;
  }

  if (!srv.response.success) {
    ROS_WARN("Planner failed to find a valid path.");
    return 0;
  }

  // 6. Execute Path
  ROS_INFO("Path received with %lu waypoints. Preparing execution...", srv.response.trajectory.points.size());

  // Convert Trajectory msg back to vector<vector<double>> for PathExecuter
  // (PathExecuter handles the Time Parameterization)
  std::vector<std::vector<double>> path_to_execute;
  for (const auto& point : srv.response.trajectory.points) {
      path_to_execute.push_back(point.positions);
  }

  // Instantiate Executer (assumes "manipulator" group, change if needed)
  PathExecuter executer("manipulator");
  
  if (executer.executePath(path_to_execute)) {
      ROS_INFO("Motion complete.");
  } else {
      ROS_ERROR("Execution failed.");
  }

  return 0;
}