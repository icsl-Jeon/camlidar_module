cmake_minimum_required(VERSION 2.8.3)
project(camlidar_module)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge_custom

  std_msgs
  message_generation
  
  sensor_msgs
  
  pcl_conversions
  pcl_ros
  message_filters
  image_transport
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  trg_msg.msg
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS message_runtime roscpp message_filters
)


set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp src/camlidar_module.cpp src/keyinput.cpp)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

set(DCMAKE_build_type release)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")

add_executable (camlidar_module ${SRC})
target_link_libraries (camlidar_module ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable (camlidar_BB_tester test/BB_tester.cpp src/camlidar_module.cpp src/keyinput.cpp)
target_link_libraries (camlidar_BB_tester ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable (hsv_target_pcl_extractor src/main_hsv.cpp src/TargetHSV.cpp src/camlidar_module.cpp src/keyinput.cpp)
target_link_libraries (hsv_target_pcl_extractor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

