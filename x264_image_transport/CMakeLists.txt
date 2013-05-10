cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

rosbuild_genmsg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(${PROJECT_NAME} src/x264_publisher.cpp src/x264_subscriber.cpp src/manifest.cpp)
target_link_libraries(${PROJECT_NAME} x264 avcodec avformat avutil swscale)

rosbuild_add_executable(x264_test_publisher src/x264_test_publisher.cpp)
target_link_libraries(x264_test_publisher ${PROJECT_NAME})

rosbuild_add_executable(x264_test_subscriber src/x264_test_subscriber.cpp)
target_link_libraries(x264_test_subscriber ${PROJECT_NAME})

