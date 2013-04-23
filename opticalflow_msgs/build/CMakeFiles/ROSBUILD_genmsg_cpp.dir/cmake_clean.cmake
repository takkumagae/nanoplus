FILE(REMOVE_RECURSE
  "../src/opticalflow_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/opticalflow_msgs/OpticalFlow.h"
  "../msg_gen/cpp/include/opticalflow_msgs/Goal.h"
  "../msg_gen/cpp/include/opticalflow_msgs/StateMsg.h"
  "../msg_gen/cpp/include/opticalflow_msgs/OpticalSensor.h"
  "../msg_gen/cpp/include/opticalflow_msgs/Traj.h"
  "../msg_gen/cpp/include/opticalflow_msgs/OpticalFlowCommand.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
