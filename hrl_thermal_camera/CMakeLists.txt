cmake_minimum_required(VERSION 2.8.3)
project(hrl_thermal_camera)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Ebus REQUIRED)
find_package(OpenCV 2 REQUIRED)
#find_package(OpenCV 3 REQUIRED)

find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  sensor_msgs
  geometry_msgs
  image_geometry
  genmsg
  message_generation
  message_runtime
  tf
  image_transport
  dynamic_reconfigure
  cv_bridge
  rospy
  usb_cam
  image_proc
)

catkin_python_setup()

#set(CMAKE_C_FLAGS "-O2")
#set(CMAKE_CXX_FLAGS "-O2")

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS   roscpp
                  std_msgs
                  pcl_ros
                  sensor_msgs
                  geometry_msgs
                  image_geometry
                  genmsg
                  message_generation
                  message_runtime
                  tf
                  image_transport
                  dynamic_reconfigure
                  cv_bridge
                  rospy
                  usb_cam
                  image_proc
 DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/hrl_thermal_camera
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Ebus_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(${PROJECT_NAME}
#  src/hrl_thermal_camera/pcloud_painter.cpp
#  src/hrl_thermal_camera/painter.cpp
#)

#   src/${PROJECT_NAME}/hrl_thermal_camera.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(hrl_thermal_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(thermal_pcloud_painter src/hrl_thermal_camera/painter.cpp src/hrl_thermal_camera/pcloud_painter.cpp )
#add_executable(hrl_thermal_camera  src/hrl_thermal_camera/hrl_thermal_camera.cpp  src/hrl_thermal_camera/hrl_thermal_camera_node.cpp)
add_executable(hrl_thermal_camera src/hrl_thermal_camera/hrl_thermal_camera_node.cpp src/hrl_thermal_camera/hrl_thermal_camera.cpp)
#add_executable(cv_lib_test src/hrl_thermal_camera/cv_lib_test_node.cpp src/hrl_thermal_camera/cv_lib_test.cpp)
#add_executable(temp_to_bw_converter src/hrl_thermal_camera/black_and_white_conversion_main.cpp src/hrl_thermal_camera/black_and_white_conversion.cpp)
add_executable(test_converter src/hrl_thermal_camera/image_converter.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(hrl_thermal_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(thermal_pcloud_painter
  ${catkin_LIBRARIES}
)

target_link_libraries(hrl_thermal_camera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Ebus_LIBRARIES}
    )
#target_link_libraries(cv_lib_test
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${Ebus_LIBRARIES}
#    )

#target_link_libraries(temp_to_bw_converter
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    ${Ebus_LIBRARIES}
#    )

target_link_libraries(test_converter
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Ebus_LIBRARIES}
    )


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
#set(OpenCV_INCLUDE_DIRS /usr/local/)
#set(OpenCV_LIBS /usr/local/lib/libopencv_core.so)


