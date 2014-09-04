cmake_minimum_required(VERSION 2.8.3)
project(beliefstate_client)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS designator_integration designator_integration_msgs roscpp roslib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES beliefstate_client
  CATKIN_DEPENDS designator_integration designator_integration_msgs roscpp roslib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(beliefstate_client
  src/beliefstate_client/beliefstate_client.cpp
)

target_link_libraries(beliefstate_client
  ${catkin_LIBRARIES}
  designator_integration
)
