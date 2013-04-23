; Auto-generated. Do not edit!


(cl:in-package sonar-msg)


;//! \htmlinclude Range.msg.html

(cl:defclass <Range> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (min_range
    :reader min_range
    :initarg :min_range
    :type cl:float
    :initform 0.0)
   (max_range
    :reader max_range
    :initarg :max_range
    :type cl:float
    :initform 0.0)
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0))
)

(cl:defclass Range (<Range>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Range>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Range)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sonar-msg:<Range> is deprecated: use sonar-msg:Range instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:header-val is deprecated.  Use sonar-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'min_range-val :lambda-list '(m))
(cl:defmethod min_range-val ((m <Range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:min_range-val is deprecated.  Use sonar-msg:min_range instead.")
  (min_range m))

(cl:ensure-generic-function 'max_range-val :lambda-list '(m))
(cl:defmethod max_range-val ((m <Range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:max_range-val is deprecated.  Use sonar-msg:max_range instead.")
  (max_range m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <Range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sonar-msg:range-val is deprecated.  Use sonar-msg:range instead.")
  (range m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Range>) ostream)
  "Serializes a message object of type '<Range>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Range>) istream)
  "Deserializes a message object of type '<Range>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min_range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Range>)))
  "Returns string type for a message object of type '<Range>"
  "sonar/Range")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Range)))
  "Returns string type for a message object of type 'Range"
  "sonar/Range")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Range>)))
  "Returns md5sum for a message object of type '<Range>"
  "36fb80612fdb39b24648681823b5f0a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Range)))
  "Returns md5sum for a message object of type 'Range"
  "36fb80612fdb39b24648681823b5f0a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Range>)))
  "Returns full string definition for message of type '<Range>"
  (cl:format cl:nil "Header header~%float32 min_range~%float32 max_range~%float32 range~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Range)))
  "Returns full string definition for message of type 'Range"
  (cl:format cl:nil "Header header~%float32 min_range~%float32 max_range~%float32 range~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Range>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Range>))
  "Converts a ROS message object to a list"
  (cl:list 'Range
    (cl:cons ':header (header msg))
    (cl:cons ':min_range (min_range msg))
    (cl:cons ':max_range (max_range msg))
    (cl:cons ':range (range msg))
))
