cmake_minimum_required(VERSION 2.8.3)
project(beliefstate_client)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(catkin REQUIRED COMPONENTS designator_integration designator_integration_msgs roscpp roslib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES beliefstate_client
  CATKIN_DEPENDS designator_integration designator_integration_msgs roscpp roslib)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/beliefstate_client/BeliefstateClient.cpp
  src/beliefstate_client/Object.cpp
  src/beliefstate_client/Context.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  designator_integration)

add_executable(ex1
  src/examples/ex1.cpp)

target_link_libraries(ex1
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})
