cmake_minimum_required(VERSION 2.8.3)
project(scamper_vision)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  nodelet
)

find_package(OpenCV 3 REQUIRED COMPONENTS
		opencv_core
		opencv_highgui
		opencv_videoio
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(TARGET "camera_publisher")
add_executable(${TARGET} src/${TARGET}.cpp)
target_link_libraries(${TARGET} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

set(TARGET "camera_viewer")
add_executable(${TARGET} src/${TARGET}.cpp)
target_link_libraries(${TARGET} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

set(TARGET "scamper_vision")
add_library(${TARGET}
		src/camera_publisher_nl.cpp
		src/camera_viewer_nl.cpp
)
target_link_libraries(${TARGET} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
