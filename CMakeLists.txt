cmake_minimum_required(VERSION 3.3)
project(QR_SLAM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( OpenCV REQUIRED )


set(SOURCE_FILES src/main.cpp src/System.cpp src/System.h)
add_executable(QR_SLAM ${SOURCE_FILES})
target_link_libraries( QR_SLAM ${OpenCV_LIBS} )