################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(multibot_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(multibot_ros2_interface REQUIRED)
find_package(multibot_util REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(gazebo_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
################################################################################
# Build
################################################################################
include_directories(
  include
)

# robot
set(ROBOT_DEPENDENCIES
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "multibot_ros2_interface"
  "multibot_util"
  "Qt5"
)

set(PANEL_SOURCES "include/multibot_robot/robot_panel.hpp")

set(ROBOT_EXEC_NAME "robot")
file(GLOB ROBOT_SOURCES "src/*.cpp")
add_executable(${ROBOT_EXEC_NAME} ${ROBOT_SOURCES} ${PANEL_SOURCES})
# add_executable(${ROBOT_EXEC_NAME} ${ROBOT_SOURCES})

target_link_libraries(${ROBOT_EXEC_NAME} Qt5::Widgets)
ament_target_dependencies(${ROBOT_EXEC_NAME} ${ROBOT_DEPENDENCIES})

target_include_directories(${ROBOT_EXEC_NAME} PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
install(TARGETS
  ${ROBOT_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  ui
  robot
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multibot_robot.dsv.in")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(Qt5)

################################################################################
# Build test
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
ament_package()