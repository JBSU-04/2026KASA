;; Auto-generated. Do not edit!


(when (boundp 'new_lane::lidar_topic_msg)
  (if (not (find-package "NEW_LANE"))
    (make-package "NEW_LANE"))
  (shadow 'lidar_topic_msg (find-package "NEW_LANE")))
(unless (find-package "NEW_LANE::LIDAR_TOPIC_MSG")
  (make-package "NEW_LANE::LIDAR_TOPIC_MSG"))

(in-package "ROS")
;;//! \htmlinclude lidar_topic_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass new_lane::lidar_topic_msg
  :super ros::object
  :slots (_header _lane_num _ACC_speed _ACC_flag _ACC_index_gap ))

(defmethod new_lane::lidar_topic_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lane_num __lane_num) 0)
    ((:ACC_speed __ACC_speed) 0.0)
    ((:ACC_flag __ACC_flag) nil)
    ((:ACC_index_gap __ACC_index_gap) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lane_num (round __lane_num))
   (setq _ACC_speed (float __ACC_speed))
   (setq _ACC_flag __ACC_flag)
   (setq _ACC_index_gap (round __ACC_index_gap))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lane_num
   (&optional __lane_num)
   (if __lane_num (setq _lane_num __lane_num)) _lane_num)
  (:ACC_speed
   (&optional __ACC_speed)
   (if __ACC_speed (setq _ACC_speed __ACC_speed)) _ACC_speed)
  (:ACC_flag
   (&optional (__ACC_flag :null))
   (if (not (eq __ACC_flag :null)) (setq _ACC_flag __ACC_flag)) _ACC_flag)
  (:ACC_index_gap
   (&optional __ACC_index_gap)
   (if __ACC_index_gap (setq _ACC_index_gap __ACC_index_gap)) _ACC_index_gap)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _lane_num
    4
    ;; float64 _ACC_speed
    8
    ;; bool _ACC_flag
    1
    ;; int32 _ACC_index_gap
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _lane_num
       (write-long _lane_num s)
     ;; float64 _ACC_speed
       (sys::poke _ACC_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _ACC_flag
       (if _ACC_flag (write-byte -1 s) (write-byte 0 s))
     ;; int32 _ACC_index_gap
       (write-long _ACC_index_gap s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _lane_num
     (setq _lane_num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _ACC_speed
     (setq _ACC_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _ACC_flag
     (setq _ACC_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _ACC_index_gap
     (setq _ACC_index_gap (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get new_lane::lidar_topic_msg :md5sum-) "f3a2e64a65093ac6a9f5ca405937d5aa")
(setf (get new_lane::lidar_topic_msg :datatype-) "new_lane/lidar_topic_msg")
(setf (get new_lane::lidar_topic_msg :definition-)
      "Header header

int32 lane_num
float64 ACC_speed

bool ACC_flag
int32 ACC_index_gap

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :new_lane/lidar_topic_msg "f3a2e64a65093ac6a9f5ca405937d5aa")


