; Auto-generated. Do not edit!


(cl:in-package merry_navigation_stack-srv)


;//! \htmlinclude QueryForFrontiers-request.msg.html

(cl:defclass <QueryForFrontiers-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass QueryForFrontiers-request (<QueryForFrontiers-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryForFrontiers-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryForFrontiers-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name merry_navigation_stack-srv:<QueryForFrontiers-request> is deprecated: use merry_navigation_stack-srv:QueryForFrontiers-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <QueryForFrontiers-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader merry_navigation_stack-srv:id-val is deprecated.  Use merry_navigation_stack-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryForFrontiers-request>) ostream)
  "Serializes a message object of type '<QueryForFrontiers-request>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryForFrontiers-request>) istream)
  "Deserializes a message object of type '<QueryForFrontiers-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryForFrontiers-request>)))
  "Returns string type for a service object of type '<QueryForFrontiers-request>"
  "merry_navigation_stack/QueryForFrontiersRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryForFrontiers-request)))
  "Returns string type for a service object of type 'QueryForFrontiers-request"
  "merry_navigation_stack/QueryForFrontiersRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryForFrontiers-request>)))
  "Returns md5sum for a message object of type '<QueryForFrontiers-request>"
  "b59605035d5cd447976dd8ed473a4ffd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryForFrontiers-request)))
  "Returns md5sum for a message object of type 'QueryForFrontiers-request"
  "b59605035d5cd447976dd8ed473a4ffd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryForFrontiers-request>)))
  "Returns full string definition for message of type '<QueryForFrontiers-request>"
  (cl:format cl:nil "int64 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryForFrontiers-request)))
  "Returns full string definition for message of type 'QueryForFrontiers-request"
  (cl:format cl:nil "int64 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryForFrontiers-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryForFrontiers-request>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryForFrontiers-request
    (cl:cons ':id (id msg))
))
;//! \htmlinclude QueryForFrontiers-response.msg.html

(cl:defclass <QueryForFrontiers-response> (roslisp-msg-protocol:ros-message)
  ((currentPose
    :reader currentPose
    :initarg :currentPose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (frontier
    :reader frontier
    :initarg :frontier
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass QueryForFrontiers-response (<QueryForFrontiers-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QueryForFrontiers-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QueryForFrontiers-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name merry_navigation_stack-srv:<QueryForFrontiers-response> is deprecated: use merry_navigation_stack-srv:QueryForFrontiers-response instead.")))

(cl:ensure-generic-function 'currentPose-val :lambda-list '(m))
(cl:defmethod currentPose-val ((m <QueryForFrontiers-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader merry_navigation_stack-srv:currentPose-val is deprecated.  Use merry_navigation_stack-srv:currentPose instead.")
  (currentPose m))

(cl:ensure-generic-function 'frontier-val :lambda-list '(m))
(cl:defmethod frontier-val ((m <QueryForFrontiers-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader merry_navigation_stack-srv:frontier-val is deprecated.  Use merry_navigation_stack-srv:frontier instead.")
  (frontier m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QueryForFrontiers-response>) ostream)
  "Serializes a message object of type '<QueryForFrontiers-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'currentPose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'frontier) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QueryForFrontiers-response>) istream)
  "Deserializes a message object of type '<QueryForFrontiers-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'currentPose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'frontier) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QueryForFrontiers-response>)))
  "Returns string type for a service object of type '<QueryForFrontiers-response>"
  "merry_navigation_stack/QueryForFrontiersResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryForFrontiers-response)))
  "Returns string type for a service object of type 'QueryForFrontiers-response"
  "merry_navigation_stack/QueryForFrontiersResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QueryForFrontiers-response>)))
  "Returns md5sum for a message object of type '<QueryForFrontiers-response>"
  "b59605035d5cd447976dd8ed473a4ffd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QueryForFrontiers-response)))
  "Returns md5sum for a message object of type 'QueryForFrontiers-response"
  "b59605035d5cd447976dd8ed473a4ffd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QueryForFrontiers-response>)))
  "Returns full string definition for message of type '<QueryForFrontiers-response>"
  (cl:format cl:nil "geometry_msgs/PoseStamped currentPose~%geometry_msgs/PoseStamped frontier~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QueryForFrontiers-response)))
  "Returns full string definition for message of type 'QueryForFrontiers-response"
  (cl:format cl:nil "geometry_msgs/PoseStamped currentPose~%geometry_msgs/PoseStamped frontier~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QueryForFrontiers-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'currentPose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'frontier))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QueryForFrontiers-response>))
  "Converts a ROS message object to a list"
  (cl:list 'QueryForFrontiers-response
    (cl:cons ':currentPose (currentPose msg))
    (cl:cons ':frontier (frontier msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'QueryForFrontiers)))
  'QueryForFrontiers-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'QueryForFrontiers)))
  'QueryForFrontiers-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QueryForFrontiers)))
  "Returns string type for a service object of type '<QueryForFrontiers>"
  "merry_navigation_stack/QueryForFrontiers")