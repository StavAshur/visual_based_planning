=================================================================
===================== SSH access to robot =======================
=================================================================

Test communication using command
"ping administrator@192.168.131.1"

SSH using the command
"ssh administrator@192.168.131.1"
password: clearpath

Copying files from pc into the robot is done using "scp":
scp <file_to_copy> administrator@192.168.131.1:<location_to_copy_to>
Example:
scp ~/catkin_ws/src/tau01_husky/tau01_moveit_config/config/husky.srdf  administrator@192.168.131.1:~/catkin_ws/src/tau01_husky/tau01_moveit_config/config/husky.srdf

First time using a new computer add the line:
"192.168.131.1    cpr-a200-0738"
to the file:
/etc/hosts

If using distrobox might need to use the command"
" sudo sysctl -w net.ipv4.ping_group_range="0 2147483647" "
to allow distrobox VMs to access ethernet socket

=================================================================
==================== Setting up pc + robot ======================
=================================================================

1. Turn robot on: 
  1.1 Turn Husky on
  1.2. Wait for comm light to turn green, then press 1 in the emergency remote (remote with 4 round black buttons + red round emergency button) to unlock
  1.3. Turn UR on
  1.4. Wait for screen to allow initialization, go to initialization screen, turn on and start UR5 (same button in initialization screen)

2. Open terminal in PC, SSH into robot ("ssh administrator@192.168.131.1") and start bringup launch file:
  "roslaunch tau01_bringup tau01_bringup.launch"

3. In UR screen 
  3.1. Press "file" (top left)
  3.2. "load program"
  3.3. Choose "external control"
  3.4. Press play icon (bottom center)

  Command "rostopic echo /ur/ur_hardware_interface/robot_program_running" from PC terminal should return "true" if working properly.

4. Open terminal in PC, SSH into robot ("ssh administrator@192.168.131.1") and start moveit launch file:
  "roslaunch tau01_moveit_config tau01_moveit_planning_execution.launch"

5. Open how ever many terminals you need in PC:
  5.0 enter distrobox using "distrobox enter <vm_name>" (example: distrobox "enter ubuntu1804-husky")
  5.1 Got to catkin_ws and source ("source devel/setup.bash")
  5.2 If we want the commands to run from a non-ssh'd terminal - need to set the robot as ros master 
    "export ROS_MASTER_URI=http://192.168.131.1:11311"
    "export ROS_IP=192.168.131.X" where X depends on the computer you're using.

    The two addresses I've seen are 100 and 101:
    "export ROS_IP=192.168.131.100"
    "export ROS_IP=192.168.131.101"

  5.3 For visualization of the robot in RViz:
  "roslaunch husky_viz view_robot.launch"

  5.4 To run visual-based motion planning:
    5.4.1 Launch service: "roslaunch visual_based_planning visual_planner.launch". See next section for parameters.
    5.4.2 Set up scene: "roslaunch scene_builder load_scene.launch". Only AABBs for now.
      edit file catkin_ws/src/scene_builder/config/obstacles.yaml to change environment
    5.4.3 Launch client with visual planning task "rosrun visual_based_planning visual_planning_client"
      Targets are defined in catkin_ws/src/visual_based_planning/config/targets.yaml


  6. To use simulated robot instead of real robot replace all above robot instructions with:
    6.1 Open and source two terminals
    6.2 In first terminal "roscore"
    6.3 Launch simulation in second "roslaunch tau01_moveit_config demo.launch use_gui:=true"
      gui is used to move the robot away from first position if invalid


The order matters! The scene must be loaded after the service is launched.


=================================================================
=================== Planner parameterization ====================
=================================================================


1. Locate the Configuration File: catkin_ws/src/visual_based_planning/config/planner_config.yaml

2. Core Behavior Settings:
  2.1 mode: Selects the planning algorithm. Set to "VisRRT" (Tree-based, good for single queries)
    or "VisPRM" (Graph-based, good for multiple queries).
  2.2 use_visual_ik: Set to true to allow the planner to "snap" the tool orientation to face the target automatically.
    Set false for standard random sampling.
  2.3 use_visibility_integrity: Set to true to create the visibility integrity data structure before running RRT/PRM.
    Pre-processing time is long.
  2.4 enable_shortcutting: Set to true to post-process the path, removing unnecessary waypoints and creating smoother motion.
  2.5 workspace_bounds: Sets the environment bounding box for visibility integrity data structure and sampling 3D points. 

3. Robot & Constraints:
  3.1 group_name: The MoveIt planning group to control (e.g., "manipulator").
  3.2 ee_link_name: The link name of the flashlight/camera (e.g., "tool0").
  3.3 resolution: Step size for collision checking in radians (suggested 0.05). Smaller is safer but slower.
  3.4 visibility_threshold: Fraction of the target ball (0.0 to 1.0, suggested 0.8) that must be illuminated to consider the goal "reached".

4. Algorithm Tuning (RRT):
  4.1 rrt/goal_bias: Probability (0.0 - 1.0, suggested 0.01) of sampling the goal state directly. Higher values converge faster in open spaces.
  4.2 rrt/max_extension: Maximum distance (in joint space radians) the tree can grow in one step. Keep small (e.g., 0.1) for cluttered environments.
  4.3 rrt/max_iterations: Maximum number of samples to try before reporting failure (suggested 5000).

5. Visibility Integrity Parameters:
    5.1 visibility_integrity/num_samples: The number of valid points that will make up the data structure (1000 - fast, 5000 - takes time)
    5.2 visibility_integrity/vi_threshold: Integrity threshold for visibility clusters (range 0.0 - 1.0, suggested 0.8). Higher value will create more, but more cohesive cluster. 
    5.3 visibility_integrity/k_neighbors: Used to label query points and associate them with a cluster (suggested 5).
    5.4 visibility_integrity/limit_diameter_factor: Higher value will create more clusters but smaller and more convex (suggested 2.0).

=================================================================
========== Changes made in the Husky onboard computer ===========
=================================================================

I have changed the file /opt/ros/melodic/share/husky_description/urdf/tau01_husky_extras.urdf.xacro IN THE ROBOT COMPUTER:
Removed the gripper links and camera link because they were causing collisions in moveit for some reason,
and also not actually physically installed at the moment.

OMPL original lib files are stored in a dir. Different place in each pc I'm using, but main one is:
/opt/ros/melodic/lib/temp_ompl_lib_files

In order to attach stl files of luminating object - changed original_urdf_tau01.xacro,
and saved a backup of revious version as backup_original_urdf_tau01.xacro
Also added collision disables to /home/roblab22/catkin_ws/src/tau01_husky/tau01_moveit_config/config/husky.srdf:
<disable_collisions link1="ur_arm_wrist_3_link" link2="flashlight" reason="Adjacent" />
<disable_collisions link1="flashlight" link2="light_beam" reason="Adjacent" />

Some more collision disabling was added to the srdf file because the planning would not be performed otherwise:
  <disable_collisions link1="base_link" link2="front_bumper_extension_link" reason="Adjacent" />
  <disable_collisions link1="base_link" link2="rear_bumper_extension_link" reason="Adjacent" />
  <disable_collisions link1="front_bumper_extension_link" link2="front_bumper_link" reason="Adjacent" />
  <disable_collisions link1="front_bumper_extension_link" link2="top_chassis_link" reason="Adjacent" />
  <disable_collisions link1="rear_bumper_extension_link" link2="rear_bumper_link" reason="Adjacent" />
  <disable_collisions link1="rear_bumper_extension_link" link2="top_chassis_link" reason="Adjacent" />

Also added a "cable_link" which is supposed to protect the cables from manipulator movements

=================================================================
================ Using distrobox (Ubuntu 24.04) =================
=================================================================

To install vscode on an ubuntu 18.04 virutal machine:
sudo apt update
sudo apt install wget gpg
wget https://update.code.visualstudio.com/1.74.3/linux-deb-x64/stable -O code_1.74.3-1668896808_amd64.deb
sudo apt install ./code_1.74.3-1668896808_amd64.deb

This is a way of creating a virutal machine that should, in theory, be able to visualize. Getting libboost issues though.
distrobox create --image ubuntu:18.04 --name ubuntu-1804 --nvidia

To allow distrobox port access I use the command:
sudo sysctl -w net.ipv4.ping_group_range="0 2147483647"



=================================================================
============================= Misc. =============================
=================================================================


Joint states can be seen by
"rostopic echo /ur/joint_states"

home position is:
[0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

Origin is center of the Husky by default.
Any x,y,z positions given to the UR are w.r.t the center of the Husky.

To find current tool position I used:
"rosrun tf tf_echo base_link ur_arm_tool0"

Was able to move the arm using this message:
"
rostopic pub /ur/ur_arm_scaled_pos_joint_traj_controller/command trajectory_msgs/JointTrajectory "header:
  stamp: now
  frame_id: ''
joint_names: ['ur_arm_shoulder_pan_joint','ur_arm_shoulder_lift_joint','ur_arm_elbow_joint','ur_arm_wrist_1_joint','ur_arm_wrist_2_joint','ur_arm_wrist_3_joint']
points:
- positions: [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]
  time_from_start: {secs: 5, nsecs: 0}"
"

=================================================================
=========== Old irrelevant stuff I did when used OMPL ===========
=================================================================


In order to change OMPL implementations:
Install OMPL on pc
  1.Open some file:
  "sudo nano /etc/apt/sources.list.d/ros-latest.list"

  2. add this line (can actually write the distribution in the previous file for ROS instead of the parameter)
  "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main"

  3. important to update after step 2!
  "sudo apt update"

  4. get the OPML files by going to the directory where you want the OMPL dir and:
  "sudo apt source ros-${ROS_DISTRO}-ompl"

  5. Need to give permissions to do stuff in the OMPL dir:
  "sudo chown -R $USER:$USER ros-XXXXX-ompl-1.X.X"
  (replace X's so it would be the dir that was created by step 4)

  6. Go into the OMPL dir (ros-XXXXX-ompl-1.X.X or whatever)
  "mkdir build && cd build"
  "cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/ompl_custom/install"
  "make -j$(nproc)"
  "make install"

  The path to where the ompl_costum dir is not very important in my opinion.
  Also the $(nproc) thing can be replaced with the number of threads deemed enough. I use 4.

Change OMPL implemnentations in ros-XXXXX-ompl-1.X.X/src/ompl
Dont forget to go into build and compile:
"make -j$(nproc)" 

Replace original OMPL files with new ones 

In ros-XXXXX-ompl-1.X.X/build/lib you have the binaries
libompl.so  libompl.so.1.X.X  libompl.so.XX
replace the old ones (after the originals have been copied for backup) in /opt/ros/XXXXX/lib (X's are ROS distro):
"sudo cp libompl.so* /opt/ros/melodic/lib/"


OMPL original lib files are stored in a dir. Different place in each pc I'm using, but main one is:
/opt/ros/melodic/lib/temp_ompl_lib_files



The OMPL moveit plugin also needs something similar.
For this I have created the ROS workspace ompl_bridge_ws which contains the single package I need.
Compiling is done the normal way (sourceing ROS and calling catkin_make)
sudo cp ~/ompl_bridge_ws/devel/lib/*.so* /opt/ros/melodic/lib/
original files are in /opt/ros/melodic/lib/temp_ompl_moveit_plugin


Commands to copy stuff to husky:

ompl:
scp ~/ros-melodic-ompl-1.4.2/build/lib/libompl.so* administrator@192.168.131.1:~/
sudo cp ~/libompl.so* /opt/ros/melodic/lib/

ompl-moveit interface:
scp ~/ompl_bridge_ws/devel/lib/libmoveit_ompl* administrator@192.168.131.1:~/
sudo cp ~/libmoveit_ompl* /opt/ros/melodic/lib/

VisibilityOracle.h:
scp ~/ros-melodic-ompl-1.4.2/src/ompl/datastructures/VisibilityOracle.h administrator@192.168.131.1:~/
sudo cp ~/VisibilityOracle.h /opt/ros/melodic/include/ompl-1.4/ompl/datastructures


Maybe need to do the same with RRTConnect.h
sudo cp ~/ros-melodic-ompl-1.4.2/src/ompl/geometric/planners/rrt/RRTConnect.h /opt/ros/melodic/include/ompl-1.4/ompl/geometric/planners/rrt/


I added  /opt/ros/melodic/include/ompl-1.4/ompl/datastructures/VisibilityOracle.h to get the ompl_interface to compile.
"sudo cp ~/ros-melodic-ompl-1.4.2/src/ompl/datastructures/VisibilityOracle.h /opt/ros/melodic/include/ompl-1.4/ompl/datastructures"

I switched RRTConnect.h in /opt/ros/melodic/... for the same reason.
Backup file is RRTConnect.h_old
"sudo cp ~/ros-melodic-ompl-1.4.2/src/ompl/geometric/planners/rrt/RRTConnect.h /opt/ros/melodic/include/ompl-1.4/ompl/geometric/planners/rrt/"



I did the exact same thing with trac_ik_kinematics_plugin:
"sudo apt source ros-melodic-trac-ik-kinematics-plugin
sudo chown -R $USER:$USER ros-melodic-trac-ik-kinematics-plugin_1.X.X
cd ros-melodic-trac-ik-kinematics-plugin_1.X.X
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic \
  -DCMAKE_PREFIX_PATH=/opt/ros/melodic
make -j$(nproc)
sudo make install"

Only difference is that the lib file is now in:
~/trac_ik_custom/install/lib/

And also I built the file on a system (my pc) with libmoveit_kinematics_base.so.1.0.11,
but the robot has libmoveit_kinematics_base.so.1.0.10 installed.
I created a symbolic link in the robot's /opt/ros/melodic/lib that made it work:
sudo ln -s libmoveit_kinematics_base.so.1.0.10 libmoveit_kinematics_base.so.1.0.11
I can probably change it to 1.0.10 by checking out the right branch.

This is obviously very bad, but I hope it's good enough.




rostopic pub -1 /joint_states sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['ur_arm_shoulder_pan_joint', 'ur_arm_shoulder_lift_joint', 'ur_arm_elbow_joint', 'ur_arm_wrist_1_joint', 'ur_arm_wrist_2_joint', 'ur_arm_wrist_3_joint']
position: [4.2718, -2.31469, -0.886255, 4.22474, -0.114735, 2.50107]
velocity: []
effort: []"


  - Component 0 Rep (Node 0): [3.13908, -2.57108, 0.584965, 0.15331, 0.821841, -1.34711]
  - Component 1 Rep (Node 1): [4.2718, -2.31469, -0.886255, 4.22474, -0.114735, 2.50107]
  - Component 2 Rep (Node 4): [-2.0684, 3.50033, -1.52582, 5.66239, 5.03846, -1.67514]
  - Component 3 Rep (Node 65): [-3.59072, -6.09309, -2.20329, -3.81297, -2.1566, 0.625787]
  - Component 4 Rep (Node 73): [5.06953, 3.43167, -1.45059, -5.0706, -4.50585, -0.583155]
  - Component 5 Rep (Node 96): [-1.53265, -6.15796, 0.367529, 0.307783, -5.07474, 1.45166]
  - Component 6 Rep (Node 192): [3.98344, -6.15471, -1.65838, -3.7672, -1.82623, 2.44857]
  - Component 7 Rep (Node 193): [2.34977, -5.6969, -2.07005, 3.04985, 4.34032, 1.24507]
python -c 'import sys, rospy, moveit_commander; moveit_commander.roscpp_initialize(sys.argv); rospy.init_node("cmd_move", anonymous=True); group = moveit_commander.MoveGroupCommander("manipulator"); group.go([3.98344, -6.15471, -1.65838, -3.7672, -1.82623, 2.44857], wait=True)'

