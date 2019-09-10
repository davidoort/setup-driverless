cmake_minimum_required(VERSION 2.8.3)
project(car)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Location.msg
  Control.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  track
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
#  INCLUDE_DIRS include
#  LIBRARIES car
CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(camerasimulator src/camerasimulator.cpp)
target_link_libraries(camerasimulator ${catkin_LIBRARIES})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(trackfinder src/trackfinder.cpp)
target_link_libraries(trackfinder ${catkin_LIBRARIES})
