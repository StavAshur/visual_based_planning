#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visual_based_planning/VisualIK.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_ik_client");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();


  ros::ServiceClient client =
      nh.serviceClient<visual_based_planning::VisualIK>("visual_ik_server");

  ROS_INFO("Waiting for visual_ik service...");
  client.waitForExistence();
  ROS_INFO("Service available.");

  //----------------------------------------------------------------------
  // 1. Load YAML containing target points
  //----------------------------------------------------------------------
  std::string yaml_path;
  if (!nh.getParam("targets_yaml", yaml_path))
  {
    ROS_ERROR("Missing param '~targets_yaml'. Set it in the launch file.");
    return 1;
  }

  YAML::Node config;

  try {
    config = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    ROS_ERROR("Failed to load YAML file: %s", e.what());
    return 1;
  }

  if (!config["target_points"])
  {
    ROS_ERROR("YAML missing 'target_points'");
    return 1;
  }

  visual_based_planning::VisualIK srv;
  YAML::Node pts = config["target_points"];

  for (std::size_t i = 0; i < pts.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = pts[i][0].as<double>();
    p.y = pts[i][1].as<double>();
    p.z = pts[i][2].as<double>();
    srv.request.target_points.push_back(p);
  }

  ROS_INFO("Loaded %lu target points from YAML.", srv.request.target_points.size());

  //----------------------------------------------------------------------
  // 2. Call the service
  //----------------------------------------------------------------------
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to call service visual_ik_server");
    return 1;
  }

  if (!srv.response.success)
  {
    ROS_WARN("Service returned failure: %s", srv.response.message.c_str());
    return 0;
  }

  geometry_msgs::Pose best_pose = srv.response.best_pose;
  ROS_INFO("Received valid pose to move to.");

  //----------------------------------------------------------------------
  // 3. Move the robot to the returned pose
  //----------------------------------------------------------------------
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  move_group.setPoseTarget(best_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (!success)
  {
    ROS_ERROR("MoveIt planning failed. Not executing.");
    return 1;
  }

  ROS_INFO("Executing motion...");
  move_group.execute(plan);

  ROS_INFO("Motion complete.");
  return 0;
}
