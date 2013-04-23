FILE(REMOVE_RECURSE
  "../src/sonar/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/sonar.dir/src/sonar.o"
  "../bin/sonar.pdb"
  "../bin/sonar"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/sonar.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
