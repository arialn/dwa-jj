; Auto-generated. Do not edit!


(cl:in-package tbug-msg)


;//! \htmlinclude goalStatusResult.msg.html

(cl:defclass <goalStatusResult> (roslisp-msg-protocol:ros-message)
  ((robot0_thereOrNot
    :reader robot0_thereOrNot
    :initarg :robot0_thereOrNot
    :type cl:integer
    :initform 0)
   (robot1_thereOrNot
    :reader robot1_thereOrNot
    :initarg :robot1_thereOrNot
    :type cl:integer
    :initform 0)
   (robot2_thereOrNot
    :reader robot2_thereOrNot
    :initarg :robot2_thereOrNot
    :type cl:integer
    :initform 0))
)

(cl:defclass goalStatusResult (<goalStatusResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <goalStatusResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'goalStatusResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tbug-msg:<goalStatusResult> is deprecated: use tbug-msg:goalStatusResult instead.")))

(cl:ensure-generic-function 'robot0_thereOrNot-val :lambda-list '(m))
(cl:defmethod robot0_thereOrNot-val ((m <goalStatusResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tbug-msg:robot0_thereOrNot-val is deprecated.  Use tbug-msg:robot0_thereOrNot instead.")
  (robot0_thereOrNot m))

(cl:ensure-generic-function 'robot1_thereOrNot-val :lambda-list '(m))
(cl:defmethod robot1_thereOrNot-val ((m <goalStatusResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tbug-msg:robot1_thereOrNot-val is deprecated.  Use tbug-msg:robot1_thereOrNot instead.")
  (robot1_thereOrNot m))

(cl:ensure-generic-function 'robot2_thereOrNot-val :lambda-list '(m))
(cl:defmethod robot2_thereOrNot-val ((m <goalStatusResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tbug-msg:robot2_thereOrNot-val is deprecated.  Use tbug-msg:robot2_thereOrNot instead.")
  (robot2_thereOrNot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <goalStatusResult>) ostream)
  "Serializes a message object of type '<goalStatusResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot0_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robot0_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'robot0_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'robot0_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot1_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robot1_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'robot1_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'robot1_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot2_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robot2_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'robot2_thereOrNot)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'robot2_thereOrNot)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <goalStatusResult>) istream)
  "Deserializes a message object of type '<goalStatusResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot0_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robot0_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'robot0_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'robot0_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot1_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robot1_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'robot1_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'robot1_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'robot2_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'robot2_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'robot2_thereOrNot)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'robot2_thereOrNot)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<goalStatusResult>)))
  "Returns string type for a message object of type '<goalStatusResult>"
  "tbug/goalStatusResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'goalStatusResult)))
  "Returns string type for a message object of type 'goalStatusResult"
  "tbug/goalStatusResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<goalStatusResult>)))
  "Returns md5sum for a message object of type '<goalStatusResult>"
  "2ce792ce8df4eddb005f1e3ae8982006")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'goalStatusResult)))
  "Returns md5sum for a message object of type 'goalStatusResult"
  "2ce792ce8df4eddb005f1e3ae8982006")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<goalStatusResult>)))
  "Returns full string definition for message of type '<goalStatusResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%uint32 robot0_thereOrNot~%uint32 robot1_thereOrNot~%uint32 robot2_thereOrNot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'goalStatusResult)))
  "Returns full string definition for message of type 'goalStatusResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%uint32 robot0_thereOrNot~%uint32 robot1_thereOrNot~%uint32 robot2_thereOrNot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <goalStatusResult>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <goalStatusResult>))
  "Converts a ROS message object to a list"
  (cl:list 'goalStatusResult
    (cl:cons ':robot0_thereOrNot (robot0_thereOrNot msg))
    (cl:cons ':robot1_thereOrNot (robot1_thereOrNot msg))
    (cl:cons ':robot2_thereOrNot (robot2_thereOrNot msg))
))
