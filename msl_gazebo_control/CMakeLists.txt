cmake_minimum_required(VERSION 2.8.3)
project(msl_gazebo_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp process_manager system_util alica_ros_proxy alica_client std_msgs message_generation)

find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
include(${QT_USE_FILE})

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
#find_package(Qt5Gui REQUIRED)
#find_package(Qt5Core REQUIRED)

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

add_message_files(
  FILES
  RobotCommand.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)


set(msl_gazebo_control_SRCS
  src/msl_gazebo_control/flowlayout.cpp
  src/msl_gazebo_control/RobotsControl.cpp
  src/msl_gazebo_control/Robot.cpp
)


set(msl_gazebo_control_HDRS
  include/msl_gazebo_control/flowlayout.h
  include/msl_gazebo_control/RobotsControl.h
  include/msl_gazebo_control/Robot.h
)

set(msl_gazebo_control_UIS
  ui/RobotsControl.ui
  ui/ControlledRobot.ui  
)

set(msl_gazebo_control_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  #${Qt5Gui_INCLUDE_DIRS}
  #${Qt5Core_INCLUDE_DIRS}
)

set(msl_gazebo_control_RCS
  resources/resources.qrc
)

catkin_package(
  INCLUDE_DIRS ${msl_gazebo_control_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)


qt4_wrap_cpp(msl_gazebo_control_MOCS ${msl_gazebo_control_HDRS})
qt4_add_resources(msl_gazebo_control_RESOURCES ${msl_gazebo_control_RCS}) 
qt4_wrap_ui(msl_gazebo_control_UIS_H ${msl_gazebo_control_UIS})

#qt5_wrap_cpp(msl_gazebo_control_MOCS ${msl_gazebo_control_HDRS})
#qt5_add_resources(msl_gazebo_control_RESOURCES ${msl_gazebo_control_RCS}) 
#qt5_wrap_ui(msl_gazebo_control_UIS_H ${msl_gazebo_control_UIS})

include_directories(${msl_gazebo_control_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${msl_gazebo_control_SRCS} ${msl_gazebo_control_MOCS} ${msl_gazebo_control_UIS_H} ${msl_gazebo_control_RESOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY})

add_dependencies(${PROJECT_NAME} ${catkin_LIBRARIES} msl_gazebo_control_generate_messages_cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/msl_gazebo_control
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/msl_gazebo_control
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)