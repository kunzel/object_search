cmake_minimum_required(VERSION 2.8.3)
project(object_search)

find_package(roscpp REQUIRED)
find_package(nav_goals_msgs REQUIRED)
find_package(std_msgs REQUIRED)
  
 
# find_package(octomap REQUIRED)
find_package(octomap 1.6 REQUIRED) 


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

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${nav_goals_msgs_INCLUDE_DIRS}
)

include_directories(${OCTOMAP_INCLUDE_DIRS})
# link_directories(${OCTOMAP_LIBRARY_DIRS})

add_executable(supporting_plane_detector src/supporting_plane_detector.cpp)
target_link_libraries(supporting_plane_detector ${OCTOMAP_LIBRARIES})

add_executable(nav_goals_evaluation_server src/nav_goals_evaluation_server.cpp)
target_link_libraries(nav_goals_evaluation_server ${roscpp_LIBRARIES})
target_link_libraries(nav_goals_evaluation_server ${OCTOMAP_LIBRARIES})
add_dependencies(nav_goals_evaluation_server nav_goals_msgs_gencpp)

#############
## Install ##
#############

install(TARGETS supporting_plane_detector #nav_goals_evaluation_server
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



