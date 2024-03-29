cmake_minimum_required(VERSION 2.8.3)
project(osa_communication)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cereal_port
  roscpp
  std_msgs
  osa_msgs
  osa_common
  can_msgs
  message_generation
  nodelet
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SelectMotorController.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES osa_serial
 CATKIN_DEPENDS cereal_port roscpp std_msgs osa_msgs osa_common can_msgs nodelet
#  DEPENDS system_lib
 DEPENDS Eigen
)

###########
## Build ##
###########

add_compile_options(-std=c++11)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include include/osa_communication)
include_directories(
  ${catkin_INCLUDE_DIRS}
  osa_control/include
)

# Declare nodelet library
add_library(socketcan_reader_nodelet src/socketcan_reader_nodelet.cpp)

## Declare a cpp library
add_library(osa_communication
  src/can_layer.cpp
  src/epos_controller.cpp
)

add_library(osa_i2c
  src/i2c_master.cpp
  src/i2c_slave.cpp
)


## Declare a cpp executable
#add_executable(osa_master_board src/master_board.cpp)
add_executable(osa_can_layer src/main.cpp)
add_executable(osa_motor_controller_board src/motor_controller_board.cpp)
add_executable(osa_waldo_joy src/waldo_joy.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(osa_can_layer osa_communication_generate_messages_cpp osa_communication osa_common)
add_dependencies(osa_motor_controller_board osa_communication_generate_messages_cpp osa_i2c osa_common)
add_dependencies(socketcan_reader_nodelet ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(osa_master_board ${catkin_LIBRARIES})
target_link_libraries(socketcan_reader_nodelet ${catkin_LIBRARIES})
target_link_libraries(osa_communication ${catkin_LIBRARIES})
target_link_libraries(osa_can_layer ${catkin_LIBRARIES} osa_communication osa_common)
target_link_libraries(osa_motor_controller_board ${catkin_LIBRARIES} osa_i2c osa_common)
target_link_libraries(osa_waldo_joy ${catkin_LIBRARIES})

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
# install(TARGETS osa_serial osa_serial_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_osa_serial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
