SET(manyears_ros_srcs
manyears_ros.cpp
)
rosbuild_add_executable(manyears_ros ${manyears_ros_srcs})
target_link_libraries(manyears_ros -lmanyears)

SET(rtaudio_srcs
rt_audio.cpp
)
rosbuild_add_executable(rt_audio ${rtaudio_srcs})
target_link_libraries(rt_audio -lrtaudio -lasound)

rosbuild_add_executable(position_exploitation sound_position_exploitation.cpp)

rosbuild_add_executable(manyears_savestream manyears_savestream.cpp)
target_link_libraries(manyears_savestream -lmanyears)
