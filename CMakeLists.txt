cmake_minimum_required(VERSION 2.8.3)
project(object_search)

find_package(roscpp REQUIRED COMPONENTS)

# find_package(octomap REQUIRED)
find_package(octomap 1.6 REQUIRED) 
#include_directories(${OCTOMAP_INCLUDE_DIRS}) 

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${OCTOMAP_LIBRARIES}
)

###########
## Build ##
###########

include_directories(${OCTOMAP_INCLUDE_DIRS})
# link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(supporting_plane_detector ${OCTOMAP_LIBRARIES})

add_executable(supporting_plane_detector src/supporting_plane_detector.cpp)

#############
## Install ##
#############

install(TARGETS supporting_plane_detector
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
