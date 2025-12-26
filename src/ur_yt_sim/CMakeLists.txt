cmake_minimum_required(VERSION 3.8)
project(ur_yt_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Find dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(linkattacher_msgs REQUIRED)

# =====================
# Executables
# =====================

add_executable(test_ik src/test_ik.cpp)
ament_target_dependencies(
  test_ik
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
)

add_executable(test_ik_collision_obj src/test_ik_collision_obj.cpp)
ament_target_dependencies(
  test_ik_collision_obj
  rclcpp
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
)

add_executable(pick_place src/pick_place.cpp)
ament_target_dependencies(
  pick_place
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  linkattacher_msgs
)

# =====================
# Install
# =====================

install(
  TARGETS
    test_ik
    test_ik_collision_obj
    pick_place
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    urdf
    config
    meshes
    worlds
    rviz
    models
  DESTINATION share/${PROJECT_NAME}
)

# =====================
# Testing
# =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
