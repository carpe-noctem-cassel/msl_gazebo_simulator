cmake_minimum_required(VERSION 2.8.3)
project(msl_gazebo_sim_link)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Qt5Core REQUIRED)
get_target_property(Qt5Core_location Qt5::Core LOCATION)
find_package(Qt5Network REQUIRED)
get_target_property(Qt5Network_location Qt5::Network LOCATION)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  msl_msgs
  system_config
)

find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_plugins gazebo_ros roscpp system_config
)

###########
## Build ##
###########
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
                     ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
                     ${Qt5Core_INCLUDE_DIRS} 
                     ${Qt5Network_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/GazeboSimLink.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Qt5Core_location} ${Qt5Network_location})
