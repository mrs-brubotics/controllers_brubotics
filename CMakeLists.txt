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
  cfg/se3_brubotics_controller.cfg 
  cfg/se3_copy_controller.cfg
  cfg/se3_brubotics_load_controller.cfg
  cfg/se3_brubotics_pm_controller.cfg
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -Wall -Wextra")

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES MpcControllerCvxgen
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

# Se3CopyController
add_library(Se3CopyController
  src/se3_copy_controller/se3_copy_controller.cpp
  )
add_dependencies(Se3CopyController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )
target_link_libraries(Se3CopyController
  ${catkin_LIBRARIES}
  )

# Se3BruboticsController
add_library(Se3BruboticsController
  src/se3_brubotics_controller/se3_brubotics_controller.cpp
  )
add_dependencies(Se3BruboticsController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )
target_link_libraries(Se3BruboticsController
  ${catkin_LIBRARIES}
  )


  # Se3BruboticsLoadController

add_library(Se3BruboticsLoadController
  src/se3_brubotics_load_controller/se3_brubotics_load_controller.cpp
  )

add_dependencies(Se3BruboticsLoadController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(Se3BruboticsLoadController
  ${catkin_LIBRARIES}
  )

  # Se3BruboticsPmController
#[[
add_library(Se3BruboticsPmController
  src/se3_brubotics_pm_controller/se3_brubotics_pm_controller.cpp
  )

add_dependencies(Se3BruboticsPmController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(Se3BruboticsPmController
  ${catkin_LIBRARIES}
  )
]]#

#############
## Install ##
#############

install(TARGETS
  Se3CopyController
  Se3BruboticsController
  Se3BruboticsLoadController
  #Se3BruboticsPmController
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
