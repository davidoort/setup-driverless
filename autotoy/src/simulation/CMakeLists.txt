cmake_minimum_required(VERSION 2.8.3)
project(simulation)

# ----- PACKAGE DEPENDENCIES -----
# Specify dependencies for building this package
find_package(
  # Add catkin
  catkin
  # Specify packages within catkin
  REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  track
)

# ----- EXPORT PACKAGE DEPENDENCIES -----
# Specify dependencies for when other packages depend on this package
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  roscpp
  track
)

# ----- BUILD -----
# Specifies additional locations of header files
# See https://cmake.org/cmake/help/v3.0/command/include_directories.html
include_directories(${catkin_INCLUDE_DIRS})

# Make main.cpp into an executable (which creates a node)
add_executable(simulationmain src/main.cpp)
target_link_libraries(simulationmain ${catkin_LIBRARIES})
