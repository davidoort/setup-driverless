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
  visualization_msgs
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

install(PROGRAMS scripts/key_teleop.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# ----- BUILD -----
# Specifies additional locations of header files
# See https://cmake.org/cmake/help/v3.0/command/include_directories.html
include_directories(${catkin_INCLUDE_DIRS})

# Make main.cpp into an executable (which creates a node)
add_executable(god src/god.cpp)
target_link_libraries(god ${catkin_LIBRARIES})

#add_executable(basic_shapes src/basic_shapes.cpp)
#target_link_libraries(basic_shapes ${catkin_LIBRARIES})

add_executable(cone_shapes src/cones.cpp)
target_link_libraries(cone_shapes ${catkin_LIBRARIES})

add_executable(visualize_track src/visualize_track.cpp)
target_link_libraries(visualize_track ${catkin_LIBRARIES})

add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines ${catkin_LIBRARIES})

#add_executable(conemap src/conemap.cpp)
#target_link_libraries(conemap ${catkin_LIBRARIES})
