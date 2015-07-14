cmake_minimum_required(VERSION 2.8.3)
project(hrl_cody_arms)

find_package(catkin REQUIRED COMPONENTS visualization_msgs std_srvs std_msgs kdl hrl_lib equilibrium_point_control)

catkin_python_setup()

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS visualization_msgs std_srvs std_msgs kdl hrl_lib equilibrium_point_control
    INCLUDE_DIRS 
    LIBRARIES 
)