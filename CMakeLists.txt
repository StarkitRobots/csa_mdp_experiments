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
  src/mre_machine
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
add_executable(mre_experiment src/mre_experiment.cpp)
target_link_libraries(mre_experiment ${catkin_LIBRARIES} csa_mdp_experiments)

# Temporary experiment
add_executable(tmp_model_based src/tmp_model_based.cpp)
target_link_libraries(tmp_model_based ${catkin_LIBRARIES} csa_mdp_experiments)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS csa_mdp_experiments csa_mdp_experiments_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_csa_mdp_experiments.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
