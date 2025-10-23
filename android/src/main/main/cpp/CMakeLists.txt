cmake_minimum_required(VERSION 3.22)

project(visualanchors_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(OpenCV REQUIRED COMPONENTS core imgproc objdetect calib3d)

add_library(visualanchors SHARED
        va_qr_detector.cpp
        va_pose.cpp
        va_jni.cpp)

target_include_directories(visualanchors
        PRIVATE
        ${OpenCV_INCLUDE_DIRS})

target_compile_options(visualanchors
        PRIVATE
        -O3
        -DNDEBUG
        -fvisibility=hidden)

target_link_libraries(visualanchors
        PRIVATE
        ${OpenCV_LIBS}
        log)
