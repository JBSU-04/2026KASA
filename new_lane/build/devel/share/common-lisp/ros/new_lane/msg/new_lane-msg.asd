
(cl:in-package :asdf)

(defsystem "new_lane-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "lidar_topic_msg" :depends-on ("_package_lidar_topic_msg"))
    (:file "_package_lidar_topic_msg" :depends-on ("_package"))
  ))