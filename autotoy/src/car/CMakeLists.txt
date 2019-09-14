cmake_minimum_required(VERSION 2.8.3)
project(car)

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
  track
)
find_package(OpenCV 3 REQUIRED)

# ----- MESSAGE DEPENDENCIES -----
# Specify internal message dependencies
add_message_files(
  FILES
  Control.msg
  Location.msg
)

# Specify external message dependencies
generate_messages(
  DEPENDENCIES
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
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Make camerasimulator.cpp into an executable (which creates a node)
add_executable(camerasimulator src/camerasimulator.cpp)
target_link_libraries(camerasimulator ${catkin_LIBRARIES})

# Make controller.cpp into an executable (which creates a node)
add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

# Make trackfinder.cpp into an executable (which creates a node)
add_executable(trackfinder src/trackfinder.cpp)
target_link_libraries(trackfinder ${catkin_LIBRARIES})
target_link_libraries(trackfinder ${OpenCV_LIBS})