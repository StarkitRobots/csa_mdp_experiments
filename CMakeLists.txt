cmake_minimum_required(VERSION 2.8.3)
project(csa_mdp_experiments)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosban_csa_mdp
)

find_package(rosban_control QUIET)

# If rosban control is found, then compile files related to simulation
if (rosban_control_FOUND)
  message("rosban_control found, adding support for gazebo interface")
  list(APPEND catkin_LIBRARIES     ${rosban_control_LIBRARIES})
  list(APPEND catkin_INCLUDE_DIRS  ${rosban_control_INCLUDE_DIRS})
else()
  message("rosban_control was not found, no support for gazebo interface")
  add_definitions(-DNO_ROSBAN_CONTROL)
endif(rosban_control_FOUND)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csa_mdp_experiments
  CATKIN_DEPENDS rosban_csa_mdp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Directories containing sources files
set(DIRECTORIES
  src/kick_model
  src/learning_machine
  src/policies
  src/problems
  )

if(rosban_control_FOUND)
  set(DIRECTORIES ${DIRECTORIES} src/interface)
endif(rosban_control_FOUND)

# Build ALL_SOURCES
set (SOURCES)
set (ALL_SOURCES)
set (PREFIXED_SOURCES)
foreach (DIRECTORY ${DIRECTORIES})
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/${DIRECTORY}")
    include (${DIRECTORY}/sources.cmake)

    foreach (SOURCE ${SOURCES})
        set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
    endforeach (SOURCE)

    set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)

# Declare a C++ library
add_library(csa_mdp_experiments
  ${ALL_SOURCES}
  )
target_link_libraries(csa_mdp_experiments ${catkin_LIBRARIES})

## Common generic program

add_executable(learn_from_logs src/learn_from_logs.cpp)
target_link_libraries(learn_from_logs ${catkin_LIBRARIES} csa_mdp_experiments)

# Generic machine to execute experiment, parameters are given through xml file
add_executable(learning_machine src/learning_machine.cpp)
target_link_libraries(learning_machine ${catkin_LIBRARIES} csa_mdp_experiments)

# Work In progress
add_executable(black_box_learning src/black_box_learning.cpp)
target_link_libraries(black_box_learning ${catkin_LIBRARIES} csa_mdp_experiments)