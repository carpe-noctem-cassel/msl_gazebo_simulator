cmake_minimum_required(VERSION 2.8.3)
project(nubot_gazebo)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_ros  
  nubot_common
  dynamic_reconfigure
  msl_actuator_msgs
)

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(config/nubot_gazebo.cfg)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

message("gazebo include path: ${GAZEBO_INCLUDE_DIRS}")

catkin_package(
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros 
    roscpp
    dynamic_reconfigure
    msl_actuator_msgs
  DEPENDS 
    gazebo
)

add_library(nubot_gazebo plugins/nubot_gazebo.cc
                         plugins/nubot_PID.cc
                         plugins/parabolic_transition_planning.cc)
target_link_libraries(nubot_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
add_dependencies(nubot_gazebo ${PROJECT_NAME}_gencfg nubot_common_generate_messages_cpp)

add_executable(nubot_teleop_keyboard plugins/nubot_teleop_keyboard.cc)
target_link_libraries(nubot_teleop_keyboard ${catkin_LIBRARIES})

# include (FindPkgConfig)
# if (PKG_CONFIG_FOUND)
#	pkg_check_modules(GAZEBO gazebo)
# endif()
