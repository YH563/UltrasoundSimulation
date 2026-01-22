cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ur_description REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# 安装资源文件（URDF/launch/world）
install(DIRECTORY urdf launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()