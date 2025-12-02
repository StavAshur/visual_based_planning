# cmake_minimum_required(VERSION 3.6)
# project(visual_based_planning)


# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   geometry_msgs
#   message_generation
#   moveit_core
#   moveit_ros_planning
#   moveit_ros_planning_interface
# )

# find_package(yaml-cpp REQUIRED)

# add_service_files(
#   FILES
#   VisualIK.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )


# catkin_package(

# )


# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )


# add_executable(visual_ik_server src/visual_ik_server.cpp)
# add_dependencies(visual_ik_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(visual_ik_server ${catkin_LIBRARIES})


# add_executable(visual_ik_client src/visual_ik_client.cpp)
# target_link_libraries(visual_ik_client
#    ${catkin_LIBRARIES}
#    yaml-cpp
# )
# add_dependencies(visual_ik_client ${catkin_EXPORTED_TARGETS})

# install(FILES config/targets.yaml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

cmake_minimum_required(VERSION 3.6)
project(visual_based_planning)

# C++14 is standard for Melodic/MoveIt
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  eigen_conversions
  angles
)

find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  VisualIK.srv
  PlanVisibilityPath.srv
)

add_message_files(
  FILES
  VisibilityTask.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visual_planner_lib
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs moveit_core moveit_ros_planning_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 1. Create a library for your logic (Implementation files would go here)
add_library(visual_planner_lib
  src/visual_planner.cpp
)
# Since we are mostly header-only right now, this target helps CMake know about headers
target_link_libraries(visual_planner_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

# 2. The Main Service Node
add_executable(visual_planning_node src/visual_planning_node.cpp)
add_dependencies(visual_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_planning_node visual_planner_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)

# 3. Existing targets (kept for your reference)
add_executable(visual_ik_server src/visual_ik_server.cpp)
add_dependencies(visual_ik_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_ik_server ${catkin_LIBRARIES})

add_executable(visual_ik_client src/visual_ik_client.cpp)
target_link_libraries(visual_ik_client ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(visual_ik_client ${catkin_EXPORTED_TARGETS})

install(FILES config/targets.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})