cmake_minimum_required(VERSION 2.8)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
get_rospack_flags(pr2_hardware_interface)
get_rospack_flags(pr2_controller_manager)
get_rospack_flags(pr2_gazebo_plugins)
get_rospack_flags(pr2_mechanism_model)

include_directories(${roscpp_INCLUDE_DIRS} ${pr2_hardware_interface_INCLUDE_DIRS} ${pr2_controller_manager_INCLUDE_DIRS} ${pr2_gazebo_plugins_INCLUDE_DIRS} ${pr2_mechanism_model_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS} ${pr2_hardware_interface_LIBRARY_DIRS} ${pr2_controller_manager_LIBRARY_DIRS} ${pr2_gazebo_plugins_LIBRARY_DIRS} ${pr2_mechanism_model_LIBRARY_DIRS})

set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")
add_library(ros_model_plugin SHARED ros_model_plugin.cc)

set_target_properties(ros_model_plugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}" COMPILE_FLAGS "${pr2_hardware_interface_CFLAGS_OTHER}" COMPILE_FLAGS "${pr2_controller_manager_CFLAGS_OTHER}" COMPILE_FLAGS "${pr2_gazebo_plugins_CFLAGS_OTHER}" COMPILE_FLAGS "${pr2_mechanism_model_CFLAGS_OTHER}")
set_target_properties(ros_model_plugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}" LINK_FLAGS "${pr2_controller_manager_LDFLAGS_OTHER}")
target_link_libraries(ros_model_plugin ${roscpp_LIBRARIES} ${pr2_hardware_interface_LIBRARIES} ${pr2_controller_manager_LIBRARIES} ${pr2_gazebo_plugins_LIBRARIES} ${pr2_mechanism_model_LIBRARIES} pr2_controller_manager)
install (TARGETS ros_model_plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)

