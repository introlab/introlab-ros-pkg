; Auto-generated. Do not edit!


(cl:in-package manyears_ros-msg)


;//! \htmlinclude AudioStream.msg.html

(cl:defclass <AudioStream> (roslisp-msg-protocol:ros-message)
  ((frame_number
    :reader frame_number
    :initarg :frame_number
    :type cl:integer
    :initform 0)
   (stream_buffer
    :reader stream_buffer
    :initarg :stream_buffer
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass AudioStream (<AudioStream>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AudioStream>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AudioStream)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name manyears_ros-msg:<AudioStream> is deprecated: use manyears_ros-msg:AudioStream instead.")))

(cl:ensure-generic-function 'frame_number-val :lambda-list '(m))
(cl:defmethod frame_number-val ((m <AudioStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_ros-msg:frame_number-val is deprecated.  Use manyears_ros-msg:frame_number instead.")
  (frame_number m))

(cl:ensure-generic-function 'stream_buffer-val :lambda-list '(m))
(cl:defmethod stream_buffer-val ((m <AudioStream>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader manyears_ros-msg:stream_buffer-val is deprecated.  Use manyears_ros-msg:stream_buffer instead.")
  (stream_buffer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AudioStream>) ostream)
  "Serializes a message object of type '<AudioStream>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_number)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stream_buffer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'stream_buffer))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AudioStream>) istream)
  "Deserializes a message object of type '<AudioStream>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frame_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frame_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frame_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frame_number)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stream_buffer) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stream_buffer)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AudioStream>)))
  "Returns string type for a message object of type '<AudioStream>"
  "manyears_ros/AudioStream")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AudioStream)))
  "Returns string type for a message object of type 'AudioStream"
  "manyears_ros/AudioStream")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AudioStream>)))
  "Returns md5sum for a message object of type '<AudioStream>"
  "1ed157802f56fd7971c695615769bab2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AudioStream)))
  "Returns md5sum for a message object of type 'AudioStream"
  "1ed157802f56fd7971c695615769bab2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AudioStream>)))
  "Returns full string definition for message of type '<AudioStream>"
  (cl:format cl:nil "#Audio stream~%uint32 frame_number~%int16[] stream_buffer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AudioStream)))
  "Returns full string definition for message of type 'AudioStream"
  (cl:format cl:nil "#Audio stream~%uint32 frame_number~%int16[] stream_buffer~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AudioStream>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stream_buffer) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AudioStream>))
  "Converts a ROS message object to a list"
  (cl:list 'AudioStream
    (cl:cons ':frame_number (frame_number msg))
    (cl:cons ':stream_buffer (stream_buffer msg))
))
