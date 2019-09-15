cmake_minimum_required(VERSION 2.8.3)
project(track)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Cones.msg
  Line.msg
  Point.msg
  Track.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ConePlacer.srv
  Generator.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES track
CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(generator src/generator.cpp)
target_link_libraries(generator ${catkin_LIBRARIES})

add_executable(coneplacer src/coneplacer.cpp)
target_link_libraries(coneplacer ${catkin_LIBRARIES})

add_executable(callBen src/callBen.cpp)
target_link_libraries(callBen ${catkin_LIBRARIES})

