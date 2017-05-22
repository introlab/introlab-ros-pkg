file(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/manyears_ros/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
