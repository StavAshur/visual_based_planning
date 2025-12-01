// visual_ik_server.cpp
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visual_based_planning/VisualIK.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include <Eigen/Geometry>
#include <vector>
#include <limits>
#include <math.h>

class VisualIKServer
{
public:
  VisualIKServer()
    : move_group("manipulator")
  {
    // Node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Parameters (can be set via YAML/rosparam)
    nh.param("num_iters", num_iters_, 20);        // default 20
    nh.param("step_size", step_size_, 0.05);      // default 5 cm per step

    ROS_INFO("Visual IK server params: num_iters=%d, step_size=%f", num_iters_, step_size_);

    // Robot model and state
    robot_model_ = move_group.getRobotModel();
    if (!robot_model_)
    {
      ROS_ERROR("Failed to get RobotModel from MoveGroupInterface");
      throw std::runtime_error("no robot model");
    }
    kinematic_state_.reset(new moveit::core::RobotState(robot_model_));
    kinematic_state_->setToDefaultValues();

    // Force the RobotState to match the real robot at startup
    robot_state::RobotStatePtr current_state = move_group.getCurrentState();
    if (current_state)
    {
        *kinematic_state_ = *current_state;
    }
    else
    {
        ROS_WARN("Initial robot state not available yet.");
    }

    // End effector link name (hardcoded per your request)
    ee_link_ = "ur_arm_tool0";

    // Advertise service
    service_ = nh.advertiseService("visual_ik_server", &VisualIKServer::serviceCallback, this);
    ROS_INFO("Visual IK service ready (ee_link='%s').", ee_link_.c_str());
  }

private:
  ros::ServiceServer service_;
  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::core::RobotModelConstPtr robot_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  std::string ee_link_;

  int num_iters_;
  double step_size_;

  bool serviceCallback(visual_based_planning::VisualIK::Request &req,
                       visual_based_planning::VisualIK::Response &res)
  {
    if (req.target_points.empty())
    {
      res.success = false;
      res.message = "Empty target_points list.";
      ROS_WARN("%s", res.message.c_str());
      return true;
    }

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    if (!current_state)
    {
        ROS_ERROR("Failed to fetch current robot state from MoveGroupInterface.");
        res.success = false;
        return true;
    }

    *kinematic_state_ = *current_state;  // sync

    // 1) compute center c (average of points) and radius r (furthest distance)
    Eigen::Vector3d c(0,0,0);
    std::vector<Eigen::Vector3d> points;
    points.reserve(req.target_points.size());
    for (const geometry_msgs::Point &gp : req.target_points)
    {
      Eigen::Vector3d p(gp.x, gp.y, gp.z);
      points.push_back(p);
      c += p;
    }
    c /= static_cast<double>(points.size());

    double r = 0.0;
    for (const Eigen::Vector3d &p : points)
    {
      double d = (p - c).norm();
      if (d > r) r = d;
    }

    ROS_INFO("Computed sphere center c=(%f, %f, %f), radius r=%f", c.x(), c.y(), c.z(), r);

    // 2) get current end-effector position p and orientation M (as quaternion)
    geometry_msgs::PoseStamped current_pose_stamped = move_group.getCurrentPose(ee_link_);
    geometry_msgs::Pose current_pose = current_pose_stamped.pose;

    Eigen::Vector3d p_current(current_pose.position.x,
                              current_pose.position.y,
                              current_pose.position.z);

    Eigen::Quaterniond q_current(current_pose.orientation.w,
                                 current_pose.orientation.x,
                                 current_pose.orientation.y,
                                 current_pose.orientation.z);

    // Ensure quaternion is normalized
    q_current.normalize();

    // Extract z-axis unit vector (3rd column of rotation matrix)
    Eigen::Matrix3d R = q_current.toRotationMatrix();
    Eigen::Vector3d z_axis = R.col(2);   // third column
    Eigen::Vector3d v = z_axis.normalized();

    ROS_INFO("Current EE pos p=(%f,%f,%f). EE z-axis v=(%f,%f,%f)",
             p_current.x(), p_current.y(), p_current.z(),
             v.x(), v.y(), v.z());


    // Temporary until I set the flashlight up
    //   get beam range h
    //   get beam angle theta
    double h = 1.0;
    double theta = M_PI/3;

    // Compute valid segment [a, b] along v
    Eigen::Vector3d a = c - (h - r) * v;
    Eigen::Vector3d b = c - (r / std::tan(theta / 2.0)) * v;
    double seg_length = (b - a).norm();

    // Eigen::Vector3d t = ik_pose.position;  // target point

    Eigen::Vector3d ab = b - a;
    double t_param = ab.dot(p_current - a) / ab.squaredNorm();

    // clamping
    t_param = (t_param < 0.0) ? 0.0 : t_param;
    t_param = (t_param > 1.0) ? 1.0 : t_param;

    Eigen::Vector3d p_prime = a + t_param * ab;

    Eigen::Vector3d dir = ab.normalized();
    int num_steps = 20;
    double step_size = seg_length / (2.0 * num_steps);

    geometry_msgs::Pose test_pose;
    test_pose.position.x = p_prime.x();
    test_pose.position.y = p_prime.y();
    test_pose.position.z = p_prime.z();
    test_pose.orientation = current_pose.orientation;


    for (int i = 1; i <= num_steps; i++) {

        // Step in +dir
        if (t_param + i * (step_size / seg_length) <= 1.0) {
            Eigen::Vector3d p_plus = p_prime + i * step_size * dir;
            test_pose.position.x = p_plus.x();
            test_pose.position.y = p_plus.y();
            test_pose.position.z = p_plus.z();
            if (checkIK(test_pose, res)) {
                return true;
            }
        }

        // Step in -dir
        if (t_param - i * (step_size / seg_length) >= 0.0) {
            Eigen::Vector3d p_minus = p_prime - i * step_size * dir;
            test_pose.position.x = p_minus.x();
            test_pose.position.y = p_minus.y();
            test_pose.position.z = p_minus.z();
            if (checkIK(test_pose, res)) {
                return true;
            }
        }
    }


    // Nothing found
    res.success = false;
    res.message = "No IK solution found along searched line/steps.";
    ROS_WARN("%s", res.message.c_str());
    return true;
  }

  // checkIK uses a temporary RobotState to avoid mutating the stored kinematic_state_
  bool checkIK(const geometry_msgs::Pose &pose, visual_based_planning::VisualIK::Response &res_out)
  {
    // Create a temporary copy of current robot state
    moveit::core::RobotState temp_state(*kinematic_state_);

    const moveit::core::JointModelGroup* jmg =
      temp_state.getJointModelGroup(move_group.getName());
    if (!jmg)
    {
      ROS_ERROR("Failed to get JointModelGroup for group '%s'", move_group.getName().c_str());
      res_out.success = false;
      res_out.message = "JointModelGroup lookup failed.";
      return false;
    }

    // Try IK. Use a small timeout (seconds) or zero for single-shot depending on setFromIK semantics.
    double ik_timeout = 0.01; // seconds - small quick attempt (adjust if needed)
    bool found = temp_state.setFromIK(jmg, pose, ik_timeout);

    if (found)
    {
      res_out.best_pose = pose;
      res_out.success = true;
      res_out.message = "Found valid IK solution.";
      ROS_INFO("IK solution found for pose (%f,%f,%f).", pose.position.x, pose.position.y, pose.position.z);
      return true;
    }

    // not found
    return false;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_ik_server");

  // IMPORTANT: MoveGroupInterface requires a spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  try
  {
    VisualIKServer server;
    ros::waitForShutdown();   // recommended instead of ros::spin()
  }
  catch (const std::exception &ex)
  {
    ROS_FATAL("Exception in VisualIKServer: %s", ex.what());
    return 1;
  }

  return 0;
}

