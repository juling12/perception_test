cmake_minimum_required(VERSION 3.0.2)
project(perception_test  CXX C)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options( -g -std=c++11 -O3 -DNDEBUG -pthread)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  eigen_conversions
  tf
  tf_conversions
  tf2
  tf2_eigen
  tf2_ros
  cmake_modules
  iplus_perception
  pluginlib
  class_loader
  )

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Glog  REQUIRED)
find_package(PCL REQUIRED COMPONENTS 
             common io filters kdtree segmentation 
             sample_consensus features visualization)
find_package(Threads REQUIRED)
# find_package(image_projector REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${image_projector_INCLUDE_DIRS}
)

# link_directories(
#   lib
  # ${image_projector_LIBRARY_DIRS}
# )

# add_library(dmtx
#   src/libdmtx/dmtx.c
# )

# add_library(ImageProjector
#   src/ImageProjector.cpp
# ) 

# target_link_libraries(DataMatrixDetector
#   dmtx
# )

add_library(hz_downsampler
  src/hz_downsampler.cpp
) 

add_executable(hz_downsampler_test src/hz_downsampler_test.cpp)
target_link_libraries(hz_downsampler_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  hz_downsampler
)

add_executable(gen_template_test src/gen_template_test.cpp)
target_link_libraries(gen_template_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  gflags
  glog
)

add_executable(icp2d_test src/icp2d_test.cpp)
target_link_libraries(icp2d_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  gflags
  glog
)

add_executable(costmap_prediction src/costmap_prediction.cpp)
target_link_libraries(costmap_prediction
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  gflags
  glog
)

add_executable(pose_test src/pose_test.cpp)
target_link_libraries(pose_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  gflags
  glog
)

install(DIRECTORY  cmake
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install(TARGETS
#    datamatrix_test DataMatrixDetector dmtx 
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Install source headers.
file(GLOB_RECURSE HDRS "*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      ${INSTALL_DIR}
  )
endforeach()

