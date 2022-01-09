cmake_minimum_required(VERSION 3.20)
project(StructureFromMotion)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect clang. Not officially reported by cmake.
execute_process(COMMAND "${CMAKE_CXX_COMPILER}" "-v" ERROR_VARIABLE CXX_VER_STDERR)
if ("${CXX_VER_STDERR}" MATCHES ".*clang.*")
    set (CMAKE_COMPILER_IS_CLANGXX 1)
endif ()

if (CMAKE_COMPILER_IS_CLANGXX)
    message("Compiling using clang")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    message("Compiling using msvc")
    set(warnings "/W4 /WX /EHsc")
else()
    message( FATAL_ERROR "This project should only be compiled with clang or msvc. Currently detected ${CMAKE_CXX_COMPILER_ID}." )
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(" ")
if (CMAKE_BUILD_TYPE MATCHES Debug)
    message("DEBUG BUILD")
    add_definitions(
            -DDEBUG
    )
elseif(CMAKE_BUILD_TYPE MATCHES Release)
    message("RELEASE BUILD")
else()
    message( FATAL_ERROR "Build type not recognised: ${CMAKE_BUILD_TYPE}." )
endif ()
message(" ")

add_executable(
        StructureFromMotion
        src/main.cpp
        headers/constants.h headers/point_cloud.h headers/camera.h headers/types.h headers/cli_util.h src/cli_util.cpp headers/imagecollection.h src/imagecollection.cpp headers/image.h src/camera.cpp headers/features.h headers/matches.h src/matches.cpp src/features.cpp headers/sfm_util.h src/sfm_util.cpp headers/scene_graph.h src/scene_graph.cpp headers/scene_reconstruction.h src/scene_reconstruction.cpp headers/pose.h src/pose.cpp src/point_cloud.cpp headers/exceptions.h headers/ba_util.h src/ba_util.cpp headers/cost/simple_reprojection_error.h headers/cost/zhang_rba_error.h src/cost/zhang_rba_error.cpp)

target_sources(
        StructureFromMotion
        PUBLIC
)

target_include_directories(
        StructureFromMotion
        PUBLIC
        ${CMAKE_SOURCE_DIR}
        /Users/meganfinch/opencv-4.5.3/include
        /Users/meganfinch/opencv-4.5.3/cmake-build-release
        /Users/meganfinch/opencv-4.5.3/modules/calib3d/include
        /Users/meganfinch/opencv-4.5.3/modules/core/include
        /Users/meganfinch/opencv-4.5.3/modules/features2d/include
        /Users/meganfinch/opencv-4.5.3/modules/highgui/include
        /Users/meganfinch/opencv-4.5.3/modules/flann/include
        /Users/meganfinch/opencv-4.5.3/modules/imgcodecs/include
        /Users/meganfinch/opencv-4.5.3/modules/videoio/include
        /Users/meganfinch/ceres-solver/include
        /Users/meganfinch/ceres-solver/cmake-build-release/config
        /Users/meganfinch/eigen
        /Users/meganfinch/glog/cmake-build-release
        /Users/meganfinch/glog/src
        /Users/meganfinch/gflags/cmake-build-release/include
)


set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (CMAKE_COMPILER_IS_CLANGXX)
    #target_compile_options(sfm PRIVATE "-Werror")
    target_compile_options(StructureFromMotion PRIVATE "-Weverything")
    target_compile_options(StructureFromMotion PRIVATE "-Wno-c99-extensions")
    target_compile_options(StructureFromMotion PRIVATE "-Wno-c++98-compat")
    target_compile_options(StructureFromMotion PRIVATE "-Wno-c++98-compat-pedantic")
    target_compile_options(StructureFromMotion PRIVATE "-Wno-padded")
endif()

target_link_directories(
        StructureFromMotion
        PUBLIC
        "${CMAKE_SOURCE_DIR}"
        "/Users/meganfinch/opencv-4.5.3/cmake-build-release/lib"
        "/Users/meganfinch/ceres-solver/cmake-build-release/lib"
        "/Users/meganfinch/gflags/cmake-build-release/lib"
        "/Users/meganfinch/glog/cmake-build-release/"
)


target_link_libraries(
        StructureFromMotion
        Threads::Threads
        opencv_calib3d
        opencv_core
        opencv_dnn
        opencv_features2d
        opencv_flann
        opencv_gapi
        opencv_highgui
        opencv_imgcodecs
        opencv_imgproc
        opencv_ml
        opencv_objdetect
        opencv_photo
        opencv_stitching
        opencv_videoio
        opencv_video
        ceres
        gflags
        glog
)
