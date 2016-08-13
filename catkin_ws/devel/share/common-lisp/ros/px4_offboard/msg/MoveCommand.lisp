; Auto-generated. Do not edit!


(cl:in-package px4_offboard-msg)


;//! \htmlinclude MoveCommand.msg.html

(cl:defclass <MoveCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (code
    :reader code
    :initarg :code
    :type cl:integer
    :initform 0))
)

(cl:defclass MoveCommand (<MoveCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_offboard-msg:<MoveCommand> is deprecated: use px4_offboard-msg:MoveCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MoveCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:header-val is deprecated.  Use px4_offboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <MoveCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:distance-val is deprecated.  Use px4_offboard-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <MoveCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:code-val is deprecated.  Use px4_offboard-msg:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveCommand>) ostream)
  "Serializes a message object of type '<MoveCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveCommand>) istream)
  "Deserializes a message object of type '<MoveCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveCommand>)))
  "Returns string type for a message object of type '<MoveCommand>"
  "px4_offboard/MoveCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveCommand)))
  "Returns string type for a message object of type 'MoveCommand"
  "px4_offboard/MoveCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveCommand>)))
  "Returns md5sum for a message object of type '<MoveCommand>"
  "f9b934f688da69b3a0fce6b02d08ed62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveCommand)))
  "Returns md5sum for a message object of type 'MoveCommand"
  "f9b934f688da69b3a0fce6b02d08ed62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveCommand>)))
  "Returns full string definition for message of type '<MoveCommand>"
  (cl:format cl:nil "Header header~%float64 distance~%int32 code~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveCommand)))
  "Returns full string definition for message of type 'MoveCommand"
  (cl:format cl:nil "Header header~%float64 distance~%int32 code~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveCommand
    (cl:cons ':header (header msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':code (code msg))
))
