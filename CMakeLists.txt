cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
project( depth_color_alignment )

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  cv_bridge
  roscpp
  sensor_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable( Color_Depth_Registration src/registration.cpp src/depth2colorAlign.cpp src/depth2colorAlign.h)
target_link_libraries( Color_Depth_Registration 
${catkin_LIBRARIES}
${OpenCV_LIBS}
${cv_bridge_LIBS}
)
  
