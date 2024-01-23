################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(multibot2_util)

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

find_package(dwb_critics REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(multibot2_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(EXTERNAL_INCLUDE_DIRS ${Boost_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${Eigen3_LIBS})

################################################################################
# Build
################################################################################
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( ${Eigen3_INCLUDE_DIRS} )

include_directories(
  include
  ${EXTERNAL_INCLUDE_DIRS}
)

set(DEPENDENCIES
    dwb_critics
    tf2
    geometry_msgs
    multibot2_msgs
)

set(LIBRARY_NAME "multibot2_util")
file(GLOB UTIL_SOURCES "src/*.cpp")

add_library(${LIBRARY_NAME} SHARED
  ${UTIL_SOURCES}
)

target_link_libraries(${LIBRARY_NAME}
   ${EXTERNAL_LIBS}
)

target_include_directories(${LIBRARY_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${dwb_critics_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
  )

ament_target_dependencies(${LIBRARY_NAME}  ${DEPENDENCIES})
ament_export_dependencies(${DEPENDENCIES})
ament_export_targets(${LIBRARY_NAME} HAS_LIBRARY_TARGET)

################################################################################
# Install
################################################################################
install(DIRECTORY include/multibot2_util/
  DESTINATION include/multibot2_util/
)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})

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