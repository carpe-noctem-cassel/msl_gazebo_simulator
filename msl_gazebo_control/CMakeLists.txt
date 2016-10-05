cmake_minimum_required(VERSION 2.8.3)
project(msl_gazebo_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp system_config)

#find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Core REQUIRED)
#INCLUDE( ${QT_USE_FILE} ) # adds some useful macros and variables, that contains a path to CMake script
SET( QT_WRAP_CPP        TRUE )
SET( QT_USE_QTXML       TRUE )

## Use c++ 11x std
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
if (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")
	set(CMAKE_CXX_COMPILER_ARG1 "-std=c++11" CACHE STRING "C++ version for eclipse" FORCE)
	set(CMAKE_ECLIPSE_VERSION "4.5" CACHE STRING "Eclipse version" FORCE)
	add_definitions (-DCMAKE_ECLIPSE_GENERATE_SOURCE_PROJECT=TRUE)
endif (${CMAKE_EXTRA_GENERATOR} MATCHES "Eclipse CDT4")

catkin_python_setup()

set(msl_gazebo_control_SRCS
  src/msl_gazebo_control/GazeboControl.cpp
)

set(msl_gazebo_control_HDRS
  include/msl_gazebo_control/GazeboControl.h
)

set(msl_gazebo_control_UIS
  ui/GazeboControl.ui
)

set(msl_gazebo_control_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS include ${CMAKE_CURRENT_BINARY_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)


QT5_WRAP_UI(msl_gazebo_control_UIS_H ${msl_gazebo_control_UIS})
QT5_WRAP_CPP(msl_gazebo_control_MOCS ${msl_gazebo_control_HDRS})

include_directories(${msl_gazebo_control_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${msl_gazebo_control_SRCS} ${msl_gazebo_control_MOCS} ${msl_gazebo_control_UIS_H})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}  ${QT_LIBRARIES} ${Qt5Network_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES})

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
