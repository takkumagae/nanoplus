FILE(REMOVE_RECURSE
  "../src/opticalflow_msgs/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/OpticalFlow.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_OpticalFlow.lisp"
  "../msg_gen/lisp/Goal.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Goal.lisp"
  "../msg_gen/lisp/StateMsg.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_StateMsg.lisp"
  "../msg_gen/lisp/OpticalSensor.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_OpticalSensor.lisp"
  "../msg_gen/lisp/Traj.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Traj.lisp"
  "../msg_gen/lisp/OpticalFlowCommand.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_OpticalFlowCommand.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
