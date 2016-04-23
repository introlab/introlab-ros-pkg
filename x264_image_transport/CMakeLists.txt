cmake_minimum_required(VERSION 2.8.3)
project(x264_image_transport)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure image_transport message_generation rosbag pluginlib std_msgs)

add_message_files(DIRECTORY msg FILES x264Packet.msg)

generate_messages(DEPENDENCIES std_msgs)

#find_package(PkgConfig)
find_library(AVFORMAT_LIBRARY REQUIRED avformat)
find_library(X264_LIBRARY REQUIRED x264)
find_library(AVCODEC_LIBRARY REQUIRED avcodec)
find_library(AVUTIL_LIBRARY REQUIRED avutil)
find_library(SWSCALE_LIBRARY REQUIRED swscale)
   

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/x264Publisher.cfg cfg/x264Subscriber.cfg)

catkin_package(
  INCLUDE_DIRS include cfg/cpp
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include cfg/cpp ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}
)

link_directories()
add_definitions()

add_library(${PROJECT_NAME} src/x264_publisher.cpp src/x264_subscriber.cpp src/manifest.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      x264 avcodec avformat avutil swscale
)

class_loader_hide_library_symbols(${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES x264_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
