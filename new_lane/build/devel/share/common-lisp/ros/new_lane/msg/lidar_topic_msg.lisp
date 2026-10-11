; Auto-generated. Do not edit!


(cl:in-package new_lane-msg)


;//! \htmlinclude lidar_topic_msg.msg.html

(cl:defclass <lidar_topic_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lane_num
    :reader lane_num
    :initarg :lane_num
    :type cl:integer
    :initform 0)
   (ACC_speed
    :reader ACC_speed
    :initarg :ACC_speed
    :type cl:float
    :initform 0.0)
   (ACC_flag
    :reader ACC_flag
    :initarg :ACC_flag
    :type cl:boolean
    :initform cl:nil)
   (ACC_index_gap
    :reader ACC_index_gap
    :initarg :ACC_index_gap
    :type cl:integer
    :initform 0))
)

(cl:defclass lidar_topic_msg (<lidar_topic_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lidar_topic_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lidar_topic_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name new_lane-msg:<lidar_topic_msg> is deprecated: use new_lane-msg:lidar_topic_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <lidar_topic_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader new_lane-msg:header-val is deprecated.  Use new_lane-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lane_num-val :lambda-list '(m))
(cl:defmethod lane_num-val ((m <lidar_topic_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader new_lane-msg:lane_num-val is deprecated.  Use new_lane-msg:lane_num instead.")
  (lane_num m))

(cl:ensure-generic-function 'ACC_speed-val :lambda-list '(m))
(cl:defmethod ACC_speed-val ((m <lidar_topic_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader new_lane-msg:ACC_speed-val is deprecated.  Use new_lane-msg:ACC_speed instead.")
  (ACC_speed m))

(cl:ensure-generic-function 'ACC_flag-val :lambda-list '(m))
(cl:defmethod ACC_flag-val ((m <lidar_topic_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader new_lane-msg:ACC_flag-val is deprecated.  Use new_lane-msg:ACC_flag instead.")
  (ACC_flag m))

(cl:ensure-generic-function 'ACC_index_gap-val :lambda-list '(m))
(cl:defmethod ACC_index_gap-val ((m <lidar_topic_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader new_lane-msg:ACC_index_gap-val is deprecated.  Use new_lane-msg:ACC_index_gap instead.")
  (ACC_index_gap m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lidar_topic_msg>) ostream)
  "Serializes a message object of type '<lidar_topic_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'lane_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ACC_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ACC_flag) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'ACC_index_gap)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lidar_topic_msg>) istream)
  "Deserializes a message object of type '<lidar_topic_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lane_num) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ACC_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'ACC_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ACC_index_gap) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lidar_topic_msg>)))
  "Returns string type for a message object of type '<lidar_topic_msg>"
  "new_lane/lidar_topic_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lidar_topic_msg)))
  "Returns string type for a message object of type 'lidar_topic_msg"
  "new_lane/lidar_topic_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lidar_topic_msg>)))
  "Returns md5sum for a message object of type '<lidar_topic_msg>"
  "f3a2e64a65093ac6a9f5ca405937d5aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lidar_topic_msg)))
  "Returns md5sum for a message object of type 'lidar_topic_msg"
  "f3a2e64a65093ac6a9f5ca405937d5aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lidar_topic_msg>)))
  "Returns full string definition for message of type '<lidar_topic_msg>"
  (cl:format cl:nil "Header header~%~%int32 lane_num~%float64 ACC_speed~%~%bool ACC_flag~%int32 ACC_index_gap~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lidar_topic_msg)))
  "Returns full string definition for message of type 'lidar_topic_msg"
  (cl:format cl:nil "Header header~%~%int32 lane_num~%float64 ACC_speed~%~%bool ACC_flag~%int32 ACC_index_gap~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lidar_topic_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     8
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lidar_topic_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'lidar_topic_msg
    (cl:cons ':header (header msg))
    (cl:cons ':lane_num (lane_num msg))
    (cl:cons ':ACC_speed (ACC_speed msg))
    (cl:cons ':ACC_flag (ACC_flag msg))
    (cl:cons ':ACC_index_gap (ACC_index_gap msg))
))
