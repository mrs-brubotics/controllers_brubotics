cmake_minimum_required(VERSION 2.8.3)
project(controllers_brubotics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  cmake_modules
  mrs_msgs
  dynamic_reconfigure
  mrs_uav_managers
  mrs_uav_controllers # additionally added by brubotics wrt ctu
  mrs_lib
  tf
  #rosbag
  )

generate_dynamic_reconfigure_options(
  cfg/se3_controller_brubotics.cfg
  )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -Wall -Wextra")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES MpcControllerCvxgen
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs mrs_msgs mrs_uav_managers mrs_lib tf mrs_uav_controllers #rosbag# mrs_uav_controllers additionally added by brubotics wrt ctu
  DEPENDS Eigen
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )
#################################### this block (= copy of ctu) was initially not added by interns
# Mpc Solver Library

# Store in CMAKE_DEB_HOST_ARCH var the current build architecture
execute_process(COMMAND
  dpkg-architecture
  -qDEB_HOST_ARCH
  OUTPUT_VARIABLE
  CMAKE_DEB_HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

# deduce the library path based on the system architecture
if(${CMAKE_DEB_HOST_ARCH} MATCHES "armhf")
  MESSAGE(FATAL_ERROR "Mising MpcControllerSolver.so for armhf")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "i386")
  MESSAGE(FATAL_ERROR "Mising MpcControllerSolver.so for i386")
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "x64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcControllerSolver/x64/libMpcControllerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "amd64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcControllerSolver/x64/libMpcControllerSolver.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "arm64")
  set(MPC_CONTROLLER_SOLVER_BIN ${PROJECT_SOURCE_DIR}/lib/MpcControllerSolver/arm64/libMpcControllerSolver.so)
else()
  MESSAGE(FATAL_ERROR ${CMAKE_DEB_HOST_ARCH})
  MESSAGE(FATAL_ERROR "MpcControllerSolver.so has not been selected, check CMakeLists.txt.")
endif()


####################################


# Se3 controller

add_library(Se3ControllerBrubotics
  src/se3_controller_brubotics/se3_controller_brubotics.cpp
  )

add_dependencies(Se3ControllerBrubotics
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(Se3ControllerBrubotics
  ${catkin_LIBRARIES}
  )


  # Se3 controller with load

add_library(Se3ControllerBruboticsLoad
  src/se3_controller_brubotics_load/se3_controller_brubotics_load.cpp
  )

add_dependencies(Se3ControllerBruboticsLoad
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(Se3ControllerBruboticsLoad
  ${catkin_LIBRARIES}
  )


#############
## Install ##
#############

install(TARGETS
  Se3ControllerBrubotics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS
  Se3ControllerBruboticsLoad
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

install(FILES
  plugins.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
