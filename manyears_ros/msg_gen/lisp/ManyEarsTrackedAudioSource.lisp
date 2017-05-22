; Auto-generated. Do not edit!


(cl:in-package manyears_ros-msg)


;//! \htmlinclude ManyEarsTrackedAudioSource.msg.html

(cl:defclass <ManyEarsTrackedAudioSource> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (sequence
    :reader sequence
    :initarg :sequence
    :type cl:integer
    :initform 0)
   (tracked_sources
    :reader tracked_sources
    :initarg :tracked_sources
    :type (cl:vector manyears_ros-msg:SourceInfo)
   :initform (cl:make-array 0 :element-type 'manyears_ros-msg:SourceInfo :initial-element (cl:make-instance 'manyears_ros-msg:SourceInfo))))
)

(cl:defclass ManyEarsTrackedAudioSource (<ManyEarsTrackedAudioSource>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ManyEarsTrackedAudioSource>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ManyEarsTrackedAudioSource)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manyears_ros-msg:<ManyEarsTrackedAudioSource> is deprecated: use manyears_ros-msg:ManyEarsTrackedAudioSource instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ManyEarsTrackedAudioSource>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_ros-msg:header-val is deprecated.  Use manyears_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'sequence-val :lambda-list '(m))
(cl:defmethod sequence-val ((m <ManyEarsTrackedAudioSource>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_ros-msg:sequence-val is deprecated.  Use manyears_ros-msg:sequence instead.")
  (sequence m))

(cl:ensure-generic-function 'tracked_sources-val :lambda-list '(m))
(cl:defmethod tracked_sources-val ((m <ManyEarsTrackedAudioSource>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_ros-msg:tracked_sources-val is deprecated.  Use manyears_ros-msg:tracked_sources instead.")
  (tracked_sources m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ManyEarsTrackedAudioSource>) ostream)
  "Serializes a message object of type '<ManyEarsTrackedAudioSource>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sequence)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tracked_sources))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tracked_sources))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ManyEarsTrackedAudioSource>) istream)
  "Deserializes a message object of type '<ManyEarsTrackedAudioSource>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sequence)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tracked_sources) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tracked_sources)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'manyears_ros-msg:SourceInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ManyEarsTrackedAudioSource>)))
  "Returns string type for a message object of type '<ManyEarsTrackedAudioSource>"
  "manyears_ros/ManyEarsTrackedAudioSource")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ManyEarsTrackedAudioSource)))
  "Returns string type for a message object of type 'ManyEarsTrackedAudioSource"
  "manyears_ros/ManyEarsTrackedAudioSource")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ManyEarsTrackedAudioSource>)))
  "Returns md5sum for a message object of type '<ManyEarsTrackedAudioSource>"
  "001908c0eeb8958ce0b3f7443ad1ff4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ManyEarsTrackedAudioSource)))
  "Returns md5sum for a message object of type 'ManyEarsTrackedAudioSource"
  "001908c0eeb8958ce0b3f7443ad1ff4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ManyEarsTrackedAudioSource>)))
  "Returns full string definition for message of type '<ManyEarsTrackedAudioSource>"
  (cl:format cl:nil "~%Header header~%~%#id~%uint32 sequence~%~%#Array of tracked sources~%manyears_ros/SourceInfo[] tracked_sources~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: manyears_ros/SourceInfo~%#Tracked source information~%uint32 source_id~%geometry_msgs/Point source_pos~%float32 longitude   # In degrees~%float32 latitude    # In degrees ~%float32 source_energy~%float32[] separation_data # Separation data (audio stream)~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ManyEarsTrackedAudioSource)))
  "Returns full string definition for message of type 'ManyEarsTrackedAudioSource"
  (cl:format cl:nil "~%Header header~%~%#id~%uint32 sequence~%~%#Array of tracked sources~%manyears_ros/SourceInfo[] tracked_sources~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: manyears_ros/SourceInfo~%#Tracked source information~%uint32 source_id~%geometry_msgs/Point source_pos~%float32 longitude   # In degrees~%float32 latitude    # In degrees ~%float32 source_energy~%float32[] separation_data # Separation data (audio stream)~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ManyEarsTrackedAudioSource>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tracked_sources) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ManyEarsTrackedAudioSource>))
  "Converts a ROS message object to a list"
  (cl:list 'ManyEarsTrackedAudioSource
    (cl:cons ':header (header msg))
    (cl:cons ':sequence (sequence msg))
    (cl:cons ':tracked_sources (tracked_sources msg))
))
