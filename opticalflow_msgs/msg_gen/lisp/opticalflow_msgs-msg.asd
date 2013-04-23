
(cl:in-package :asdf)

(defsystem "opticalflow_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "OpticalFlow" :depends-on ("_package_OpticalFlow"))
    (:file "_package_OpticalFlow" :depends-on ("_package"))
    (:file "Goal" :depends-on ("_package_Goal"))
    (:file "_package_Goal" :depends-on ("_package"))
    (:file "StateMsg" :depends-on ("_package_StateMsg"))
    (:file "_package_StateMsg" :depends-on ("_package"))
    (:file "OpticalSensor" :depends-on ("_package_OpticalSensor"))
    (:file "_package_OpticalSensor" :depends-on ("_package"))
    (:file "Traj" :depends-on ("_package_Traj"))
    (:file "_package_Traj" :depends-on ("_package"))
    (:file "OpticalFlowCommand" :depends-on ("_package_OpticalFlowCommand"))
    (:file "_package_OpticalFlowCommand" :depends-on ("_package"))
  ))