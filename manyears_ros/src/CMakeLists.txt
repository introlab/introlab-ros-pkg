set(CMAKE_VERBOSE_MAKEFILE ON)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
        roscpp
        )

FIND_PATH(RTAUDIO_INCLUDE_DIRS RtAudio.h
        /usr/local/manyears/rtaudio/include)

FIND_PATH(MANYEARS_INCLUDE_DIRS overallContext.h
        /usr/local/manyears/rtaudio/include)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${RTAUDIO_INCLUDE_DIRS}
        ${MANYEARS_INCLUDE_DIRS} ../include
    )

FIND_LIBRARY(RTAUDIO_LIBRARIES rtaudio
        /usr/local/manyears/rtaudio/lib)

FIND_LIBRARY(MANYEARS_LIBRARIES manyears
        /usr/local/manyears/lib)

#rosbuild_add_executable(manyears_ros ${manyears_ros_srcs})
#target_link_libraries(${PROJECT_NAME} -lmanyears ${RTAUDIO_LIBRARIES})

ADD_EXECUTABLE(manyears_ros
        ../include/manyears_ros/AudioStream.h
        ../include/manyears_ros/ManyEarsTrackedAudioSource.h
        manyears_ros.cpp
        )

TARGET_LINK_LIBRARIES(manyears_ros ${catkin_LIBRARIES} ${RTAUDIO_LIBRARIES} ${MANYEARS_LIBRARIES})


ADD_EXECUTABLE(rt_audio
        ../include/manyears_ros/AudioStream.h
        rt_audio.cpp
        )

TARGET_LINK_LIBRARIES(rt_audio ${catkin_LIBRARIES} ${RTAUDIO_LIBRARIES})

ADD_EXECUTABLE(manyears_savestream
        ../include/manyears_ros/ManyEarsTrackedAudioSource.h
        manyears_savestream.cpp
        )



TARGET_LINK_LIBRARIES(manyears_savestream ${catkin_LIBRARIES})

ADD_EXECUTABLE(position_exploitation
        ../include/manyears_ros/ManyEarsTrackedAudioSource.h
        sound_position_exploitation.cpp
        )

TARGET_LINK_LIBRARIES(position_exploitation ${catkin_LIBRARIES} ${RTAUDIO_LIBRARIES})

#rosbuild_add_executable(rt_audio ${rtaudio_srcs})
#target_link_libraries(rt_audio ${RTAUDIO_LIBRARIES} -lasound)

#rosbuild_add_executable(position_exploitation sound_position_exploitation.cpp)

#rosbuild_add_executable(manyears_savestream manyears_savestream.cpp)
#target_link_libraries(manyears_savestream -lmanyears)
