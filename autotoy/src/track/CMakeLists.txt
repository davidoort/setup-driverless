cmake_minimum_required(VERSION 2.8.3)
project(track)

# ----- PACKAGE DEPENDENCIES -----
# Specify dependencies for building this package
find_package(
  # Add catkin
  catkin
  # Specify packages within catkin
  REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
)

# ----- SERVICE DEPENDENCIES -----
# Specify internal service dependencies
add_service_files(
  FILES
  ConePlacer.srv
  Generator.srv
)

# ----- MESSAGE DEPENDENCIES -----
# Specify internal message dependencies
add_message_files(
  FILES
  Cone.msg
  Cones.msg
  Line.msg
  Point.msg
  Track.msg
)

# Specify external message dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
)

# ----- EXPORT PACKAGE DEPENDENCIES -----
# Specify dependencies for when other packages depend on this package
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  roscpp
)

# ----- BUILD -----
# Specifies additional locations of header files
# See https://cmake.org/cmake/help/v3.0/command/include_directories.html
include_directories(${catkin_INCLUDE_DIRS})

add_executable(generator src/generator.cpp)
target_link_libraries(generator ${catkin_LIBRARIES})

add_executable(coneplacer src/coneplacer.cpp)
target_link_libraries(coneplacer ${catkin_LIBRARIES})
