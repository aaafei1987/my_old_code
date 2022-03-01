cmake_minimum_required(VERSION 3.10.2)
project(auto_parking)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  eigen_conversions 
  std_srvs
  pcl_ros
  pcl_conversions
)

add_message_files(
    FILES
    SetAutoPark.msg
    AutoParkStatus.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

find_package(OpenMP REQUIRED)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include_directories(include  ${catkin_INCLUDE_DIRS})
include(cmake/src_path_defination.cmake)
catkin_package()

link_directories(
	include
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(auto_parking_node src/auto_parking_node.cpp ${ALL_SRCS})
add_dependencies(auto_parking_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_parking_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(laser_obstacle_node src/laser_obstacle_node.cpp ${ALL_SRCS})
add_dependencies(laser_obstacle_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(laser_obstacle_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

install(TARGETS 
        laser_obstacle_node
        auto_parking_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

