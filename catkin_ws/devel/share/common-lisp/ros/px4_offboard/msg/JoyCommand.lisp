; Auto-generated. Do not edit!


(cl:in-package px4_offboard-msg)


;//! \htmlinclude JoyCommand.msg.html

(cl:defclass <JoyCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (failsafe
    :reader failsafe
    :initarg :failsafe
    :type cl:boolean
    :initform cl:nil)
   (arm
    :reader arm
    :initarg :arm
    :type cl:boolean
    :initform cl:nil)
   (offboard
    :reader offboard
    :initarg :offboard
    :type cl:boolean
    :initform cl:nil)
   (takeoff
    :reader takeoff
    :initarg :takeoff
    :type cl:boolean
    :initform cl:nil)
   (land
    :reader land
    :initarg :land
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JoyCommand (<JoyCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoyCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoyCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_offboard-msg:<JoyCommand> is deprecated: use px4_offboard-msg:JoyCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:header-val is deprecated.  Use px4_offboard-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:position-val is deprecated.  Use px4_offboard-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:yaw-val is deprecated.  Use px4_offboard-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:orientation-val is deprecated.  Use px4_offboard-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'failsafe-val :lambda-list '(m))
(cl:defmethod failsafe-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:failsafe-val is deprecated.  Use px4_offboard-msg:failsafe instead.")
  (failsafe m))

(cl:ensure-generic-function 'arm-val :lambda-list '(m))
(cl:defmethod arm-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:arm-val is deprecated.  Use px4_offboard-msg:arm instead.")
  (arm m))

(cl:ensure-generic-function 'offboard-val :lambda-list '(m))
(cl:defmethod offboard-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:offboard-val is deprecated.  Use px4_offboard-msg:offboard instead.")
  (offboard m))

(cl:ensure-generic-function 'takeoff-val :lambda-list '(m))
(cl:defmethod takeoff-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:takeoff-val is deprecated.  Use px4_offboard-msg:takeoff instead.")
  (takeoff m))

(cl:ensure-generic-function 'land-val :lambda-list '(m))
(cl:defmethod land-val ((m <JoyCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_offboard-msg:land-val is deprecated.  Use px4_offboard-msg:land instead.")
  (land m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoyCommand>) ostream)
  "Serializes a message object of type '<JoyCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'failsafe) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arm) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'offboard) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'takeoff) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'land) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoyCommand>) istream)
  "Deserializes a message object of type '<JoyCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
    (cl:setf (cl:slot-value msg 'failsafe) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'arm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'offboard) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'takeoff) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'land) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoyCommand>)))
  "Returns string type for a message object of type '<JoyCommand>"
  "px4_offboard/JoyCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoyCommand)))
  "Returns string type for a message object of type 'JoyCommand"
  "px4_offboard/JoyCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoyCommand>)))
  "Returns md5sum for a message object of type '<JoyCommand>"
  "1ff5c348546b7bfead04686358bc531d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoyCommand)))
  "Returns md5sum for a message object of type 'JoyCommand"
  "1ff5c348546b7bfead04686358bc531d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoyCommand>)))
  "Returns full string definition for message of type '<JoyCommand>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point position~%float64 yaw~%geometry_msgs/Quaternion orientation~%bool failsafe~%bool arm ~%bool offboard~%bool takeoff~%bool  land~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoyCommand)))
  "Returns full string definition for message of type 'JoyCommand"
  (cl:format cl:nil "Header header~%geometry_msgs/Point position~%float64 yaw~%geometry_msgs/Quaternion orientation~%bool failsafe~%bool arm ~%bool offboard~%bool takeoff~%bool  land~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoyCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoyCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'JoyCommand
    (cl:cons ':header (header msg))
    (cl:cons ':position (position msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':failsafe (failsafe msg))
    (cl:cons ':arm (arm msg))
    (cl:cons ':offboard (offboard msg))
    (cl:cons ':takeoff (takeoff msg))
    (cl:cons ':land (land msg))
))
