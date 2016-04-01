cmake_minimum_required(VERSION 3.3)
project(QR_SLAM)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( OpenCV REQUIRED )


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)



set(SOURCE_FILES
        src/main.cpp
        src/System.cpp
        src/Mapper.cpp
        src/Viewer.cpp
        src/Loopper.cpp
        src/Frame.cpp
        src/KeyFrame.cpp
        src/Tracker.cpp include/Tracker.h)
add_executable(QR_SLAM ${SOURCE_FILES})
target_link_libraries( QR_SLAM ${OpenCV_LIBS} )