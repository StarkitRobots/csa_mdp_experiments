cmake_minimum_required(VERSION 2.8.3)
project(csa_mdp_experiments)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosban_csa_mdp
)

find_package(rosban_control)

# If rosban control is found, then compile files related to simulation
if (rosban_control_FOUND)
  message("rosban_control found, adding support for regression_forests control")
  list(APPEND catkin_LIBRARIES     ${rosban_control_LIBRARIES})
  list(APPEND catkin_INCLUDE_DIRS  ${rosban_control_INCLUDE_DIRS})
else()
  message("rosban_control was not found, no support for regression_forests control")
endif(rosban_control_FOUND)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csa_mdp_experiments
#  CATKIN_DEPENDS rosban_control
#  DEPENDS system_lib
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
  src/learning_machine
#  src/mre_machine #Mostly replaced by learning machine but not entirely yet
  src/policies
  src/problems
  src/interface
  )

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

# Generic MRE experiments (evaluation / exploration)
# Mostly replaced by learning machine but not entirely yet
#add_executable(mre_experiment src/mre_experiment.cpp)
#target_link_libraries(mre_experiment ${catkin_LIBRARIES} csa_mdp_experiments)

# Temporary experiment
add_executable(learning_machine src/learning_machine.cpp)
target_link_libraries(learning_machine ${catkin_LIBRARIES} csa_mdp_experiments)