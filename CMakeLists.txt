cmake_minimum_required(VERSION 3.28)
project(calibrationresulttest)

set(CMAKE_CXX_STANDARD 17)
#只和cv的配置有关
if(NOT OpenCV_DIR)
    set(OpenCV_DIR D:/opencv/opencv/mingwbuild)
endif()
message(STATUS "OpenCV_DIR = ${OpenCV_DIR}")
find_package(OpenCV REQUIRED)
add_executable(calibrationresulttest main.cpp CameraParams_1_1.h)

target_include_directories(calibrationresulttest PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibrationresulttest PRIVATE ${OpenCV_LIBS})
