################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(multibot2_robot)

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

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(multibot2_msgs REQUIRED)
find_package(multibot2_util REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ompl REQUIRED)
# find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(Boost COMPONENTS program_options system filesystem REQUIRED)

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES} ${CERES_LIBRARIES} ${OMPL_LIBRARIES} Qt6::Widgets)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${CERES_INCLUDES}
  ${EXTERNAL_INCLUDE_DIRS}
)

# robot
set(ROBOT_DEPENDENCIES
  rclcpp
  costmap_converter
  geometry_msgs
  tf2
  tf2_ros
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  multibot2_msgs
  multibot2_util
  Qt6
  nav2_core
  nav2_costmap_2d
  nav2_util
)

set(ROBOT_EXEC_NAME "robot")
set(library_name ${ROBOT_EXEC_NAME}_core)

file(GLOB ROBOT_SRC_FILES "src/*.cpp")
list(REMOVE_ITEM ROBOT_SRC_FILES ${PROJECT_SOURCE_DIR}/src/main.cpp)
file(GLOB TEB_SRC_FILES "src/teb_local_planner/*.cpp")
file(GLOB GLOBAL_PLANNER_SRC_FILES "src/global_planner/*.cpp")
set(PANEL_HEADER_FILE "include/multibot2_robot/robot_panel.h")

add_library(${library_name} SHARED
  ${ROBOT_SRC_FILES}
  ${TEB_SRC_FILES}
  ${GLOBAL_PLANNER_SRC_FILES}
  ${PANEL_HEADER_FILE}
)

target_include_directories(${library_name} PUBLIC
  ${Qt6Core_INCLUDE_DIRS}
  ${Qt6Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

ament_target_dependencies(${library_name}
  ${ROBOT_DEPENDENCIES}
)

target_compile_definitions(${library_name}
  PUBLIC "PLUGINLIB__ENABLE_BOOST_FUNCTIONS"
)

add_executable(${ROBOT_EXEC_NAME}
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  ${PANEL_HEADER_FILE}
)

target_link_libraries(${ROBOT_EXEC_NAME}
  ${library_name}
  ${EXTERNAL_LIBS}
)

set(CONTROLLER_EXEC_NAME "keyboard_controller")
set(CONTROLLER_library_name ${CONTROLLER_EXEC_NAME}_core)

file(GLOB CONTROLLER_SRC_FILES "src/controller/*.cpp")
list(REMOVE_ITEM CONTROLLER_SRC_FILES ${PROJECT_SOURCE_DIR}/src/controller/keyboard_controller.cpp)
set(CONTROLLER_PANEL_HEADER_FILE "include/multibot2_robot/controller/controller_panel.h")

add_library(${CONTROLLER_library_name} SHARED
  ${CONTROLLER_SRC_FILES}
  ${CONTROLLER_PANEL_HEADER_FILE}
)

target_include_directories(${CONTROLLER_library_name} PUBLIC
  ${Qt6Core_INCLUDE_DIRS}
  ${Qt6Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

ament_target_dependencies(${CONTROLLER_library_name}
  ${ROBOT_DEPENDENCIES}
)

target_compile_definitions(${CONTROLLER_library_name}
  PUBLIC "PLUGINLIB__ENABLE_BOOST_FUNCTIONS"
)

add_executable(${CONTROLLER_EXEC_NAME}
  ${PROJECT_SOURCE_DIR}/src/controller/keyboard_controller.cpp
  ${CONTROLLER_PANEL_HEADER_FILE}
)

target_link_libraries(${CONTROLLER_EXEC_NAME}
  ${CONTROLLER_library_name}
  Qt6::Widgets
)

################################################################################
# Install
################################################################################
install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${ROBOT_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${CONTROLLER_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${CONTROLLER_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  params
  robot
  rviz
  ui
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(Qt6)

ament_export_libraries(${library_name})

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