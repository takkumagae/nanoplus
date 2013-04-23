FILE(REMOVE_RECURSE
  "../src/opticalflow_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/opticalflow_msgs/msg/__init__.py"
  "../src/opticalflow_msgs/msg/_OpticalFlow.py"
  "../src/opticalflow_msgs/msg/_Goal.py"
  "../src/opticalflow_msgs/msg/_StateMsg.py"
  "../src/opticalflow_msgs/msg/_OpticalSensor.py"
  "../src/opticalflow_msgs/msg/_Traj.py"
  "../src/opticalflow_msgs/msg/_OpticalFlowCommand.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
