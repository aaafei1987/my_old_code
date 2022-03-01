cmake_minimum_required(VERSION 3.10.2)
project(mag_recognition)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  message_generation
  hinson_640n
  tf
)

#add_message_files(
#  FILES
# Hinson640nData.msg
#)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")


include(cmake/YAML.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

add_executable(mag_recognition  src/mag_recognition_node.cpp ${ALL_SRCS})
add_dependencies(mag_recognition ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_recognition ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(mag_record  src/mag_record_node.cpp ${ALL_SRCS})
add_dependencies(mag_record ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_record ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(mag_test  src/test_node.cpp ${ALL_SRCS})
add_dependencies(mag_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_test ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

install(TARGETS 
    mag_recognition
    mag_record
    mag_test
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


