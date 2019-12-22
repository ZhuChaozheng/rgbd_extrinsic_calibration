cmake_minimum_required(VERSION 2.8.3)
project(rgbd_extrinsic_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
)

# add PCL library
FIND_PACKAGE( PCL REQUIRED)
message(${PCL_LIBRARIES})
# add opencv
FIND_PACKAGE( OpenCV 3 REQUIRED )

# add Eigen
include_directories("/usr/include/eigen3")

# add head file and library
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# add include
include_directories(${PROJECT_SOURCE_DIR}/include)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES RGBDExtrinsicCalibration
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

add_message_files(
  FILES
  position.msg
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_library(rgbdextrinsiccalibration SHARED
    src/estimatePlane.cpp
    src/getCameraOrientation.cpp
    src/judgeTheCondition.cpp
    src/generatePointCloud.cpp
    src/getExternalParameterFromPlane.cpp
    src/pointCloudTransform.cpp
    src/decomposeMatrixT.cpp
    src/showDepthSensorModel.cpp
)

target_link_libraries(rgbdextrinsiccalibration
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

add_executable(test_sensor test_sensor_depth.cpp)
target_link_libraries(test_sensor
    rgbdextrinsiccalibration
    ${catkin_LIBRARIES}
)

add_executable(main main.cpp)
target_link_libraries(main
    ${PCL_LIBRARIES}
)
