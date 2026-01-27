cmake_minimum_required(VERSION 3.8)
project(kinematics_module)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(pluginlib REQUIRED)

find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_kinematics_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
