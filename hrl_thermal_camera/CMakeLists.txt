cmake_minimum_required(VERSION 2.8.3)
project(hrl_thermal_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  tf
  multisense_lib
  rosbag
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
  image_geometry
  message_generation
)

catkin_python_setup()

set(CMAKE_C_FLAGS "-O2")
set(CMAKE_CXX_FLAGS "-O2")



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DeviceInfo.msg
  RawCamConfig.msg
  RawImuData.msg
  RawLidarData.msg
  RawCamCal.msg
  RawCamData.msg
  RawLidarCal.msg
  Histogram.msg
  StampedPps.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(cfg/multisense.cfg)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp
                rospy
                std_msgs
                genmsg
                tf
                multisense_lib
                rosbag
                sensor_msgs
                angles
                cv_bridge
                dynamic_reconfigure
                image_geometry
                image_transport
                message_runtime
                message_generation
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/camera.cpp
  src/reconfigure.cpp
)

add_executable(hrl_thermal_camera_node src/hrl_thermal_camera_node.cpp)

#   src/${PROJECT_NAME}/hrl_thermal_camera.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(hrl_thermal_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(hrl_thermal_camera_node src/hrl_thermal_camera_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(hrl_thermal_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(hrl_thermal_camera_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS hrl_thermal_camera hrl_thermal_camera_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hrl_thermal_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
