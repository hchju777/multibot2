################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(multibot2_driver)

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

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(multibot2_msgs REQUIRED)

find_package(gazebo_ros REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

# Fake Driver
set(FAKE_DRIVER_DEPENDENCIES
  "rclcpp"
  "rclpy"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2_msgs"
)

set(FAKE_DRIVER_EXEC_NAME "fake_driver")
set(FAKE_DRIVER_LIB_NAME ${FAKE_DRIVER_EXEC_NAME}_core)

file(GLOB FAKE_DRIVER_SOURCES "src/fake_driver/*.cpp")
list(REMOVE_ITEM FAKE_DRIVER_SOURCES ${PROJECT_SOURCE_DIR}/src/fake_driver/fake_node.cpp)
add_library(${FAKE_DRIVER_LIB_NAME} SHARED
  ${FAKE_DRIVER_SOURCES}
)

ament_target_dependencies(${FAKE_DRIVER_LIB_NAME}
    ${FAKE_DRIVER_DEPENDENCIES}
)

target_compile_definitions(${FAKE_DRIVER_LIB_NAME}
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

add_executable(${FAKE_DRIVER_EXEC_NAME}
    ${PROJECT_SOURCE_DIR}/src/fake_driver/fake_node.cpp
)

target_link_libraries(${FAKE_DRIVER_EXEC_NAME}
  ${FAKE_DRIVER_LIB_NAME}
)

# ISR_M2_Node
set(ISR_M2_NODE_DEPENDENCIES
  "rclcpp"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "nav_msgs"
  "multibot2_msgs"
)

set(ISR_M2_NODE_EXEC_NAME "isr_m2_node")
set(ISR_M2_NODE_LIB_NAME ${ISR_M2_NODE_EXEC_NAME}_core)

file(GLOB DRIVER_SOURCES "src/driver/*.cpp")
list(REMOVE_ITEM DRIVER_SOURCES ${PROJECT_SOURCE_DIR}/src/driver/driver_node.cpp)
add_library(${ISR_M2_NODE_LIB_NAME} SHARED
  ${DRIVER_SOURCES}
)

ament_target_dependencies(${ISR_M2_NODE_LIB_NAME}
    ${ISR_M2_NODE_DEPENDENCIES}
)

target_compile_definitions(${ISR_M2_NODE_LIB_NAME}
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

add_executable(${ISR_M2_NODE_EXEC_NAME}
    ${PROJECT_SOURCE_DIR}/src/driver/driver_node.cpp
)

target_link_libraries(${ISR_M2_NODE_EXEC_NAME}
  ${ISR_M2_NODE_LIB_NAME}
)

################################################################################
# Install
################################################################################
# Fake Driver
install(TARGETS
  ${FAKE_DRIVER_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# ISR_M2_Node
install(TARGETS
  ${ISR_M2_NODE_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_ros)

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