cmake_minimum_required(VERSION 3.0.2)
project(mpc_ros VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# ROS related
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    std_msgs
    dynamic_reconfigure
    geometry_msgs
    mav_interface_msgs
)

# Other dependencies
find_package(Boost REQUIRED)
find_package(YamlCpp REQUIRED)

# Dynamic reconfigure stuff
generate_dynamic_reconfigure_options(cfg/Commands.cfg)

# Declare a catkin package
catkin_package()

# Build the third-party libraries
add_subdirectory(third_party)
set(HPIPM_LIBS hpipm blasfeo)

set(ATP_SOURCE
    src/Autopilot.cpp
    src/MPC.cpp
)

# Controller node
add_executable(mpc_node src/node.cpp ${ATP_SOURCE})
add_dependencies(mpc_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_include_directories(mpc_node PRIVATE
    ${YAMLCPP_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
include ${catkin_INCLUDE_DIRS}
include ${Boost_INCLUDE_DIRS}
${hpipm_INCLUDE_DIRS}
${blasfeo_INCLUDE_DIRS}
)

target_link_libraries(mpc_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${YAMLCPP_LIBRARY}
    ${HPIPM_LIBS}
)

