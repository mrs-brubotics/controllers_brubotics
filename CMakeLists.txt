cmake_minimum_required(VERSION 3.5)
project(controllers_brubotics)

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  mrs_lib
  mrs_msgs
  mrs_uav_managers
  pluginlib
  roscpp
  rospy
  mrs_modules_msgs # added by brubotics
  sensor_msgs # added by brubotics
  std_msgs # added by brubotics
  nav_msgs # added by brubotics
  mrs_uav_controllers # added by brubotics
  tf # added by brubotics
  #rosbag # added by brubotics
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

generate_dynamic_reconfigure_options(
  cfg/se3_copy_controller.cfg
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wpedantic)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

set(LIBRARIES
  ControllersBrubotics_Se3CopyController
  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS Eigen
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

# Se3CopyController
add_library(ControllersBrubotics_Se3CopyController
  src/se3_copy_controller/se3_copy_controller.cpp
  src/common.cpp
  )

add_dependencies(ControllersBrubotics_Se3CopyController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(ControllersBrubotics_Se3CopyController
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# install(DIRECTORY scripts/
#   USE_SOURCE_PERMISSIONS
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )

# install(DIRECTORY scripts
#   USE_SOURCE_PERMISSIONS
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   )
