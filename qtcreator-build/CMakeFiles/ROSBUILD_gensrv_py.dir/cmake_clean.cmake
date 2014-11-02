FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/merry_navigation_stack/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/merry_navigation_stack/srv/__init__.py"
  "../src/merry_navigation_stack/srv/_QueryForFrontiers.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
