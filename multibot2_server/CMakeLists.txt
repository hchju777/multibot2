################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(multibot2_server)

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
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(multibot2_msgs REQUIRED)
find_package(multibot2_util REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(gazebo_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(CGAL REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

# server
set(SERVER_DEPENDENCIES
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  multibot2_msgs
  multibot2_util
  Qt5
)

set(SERVER_EXEC_NAME "server")
set(library_name ${SERVER_EXEC_NAME}_core)

file(GLOB SERVER_SRC_FILES "src/*.cpp")
list(REMOVE_ITEM SERVER_SRC_FILES ${PROJECT_SOURCE_DIR}/src/main.cpp)
set(PANEL_HEADER_FILE "include/multibot2_server/server_panel.h")
add_library(${library_name} SHARED
  ${SERVER_SRC_FILES}
  ${PANEL_HEADER_FILE}
)

set(EXTERNAL_LIBS ${CGAL_LIBRARIES} Qt5::Widgets)

target_include_directories(${library_name} PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

ament_target_dependencies(${library_name}
  ${SERVER_DEPENDENCIES}
)

target_compile_definitions(${library_name}
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

add_executable(${SERVER_EXEC_NAME}
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  ${PANEL_HEADER_FILE}
)

target_link_libraries(${SERVER_EXEC_NAME}
  ${library_name}
  ${EXTERNAL_LIBS}
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

install(TARGETS ${SERVER_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  maps
  models
  rviz
  ui
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multibot2_server.dsv.in")

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(Qt5)

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