file(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/manyears_ros/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/tests.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
