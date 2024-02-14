; Auto-generated. Do not edit!


(cl:in-package rmus_solution-srv)


;//! \htmlinclude switch-request.msg.html

(cl:defclass <switch-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass switch-request (<switch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <switch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'switch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rmus_solution-srv:<switch-request> is deprecated: use rmus_solution-srv:switch-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <switch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rmus_solution-srv:mode-val is deprecated.  Use rmus_solution-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <switch-request>) ostream)
  "Serializes a message object of type '<switch-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <switch-request>) istream)
  "Deserializes a message object of type '<switch-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<switch-request>)))
  "Returns string type for a service object of type '<switch-request>"
  "rmus_solution/switchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'switch-request)))
  "Returns string type for a service object of type 'switch-request"
  "rmus_solution/switchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<switch-request>)))
  "Returns md5sum for a message object of type '<switch-request>"
  "9ae8834e0ab84ed7c29d5f2c87a2b979")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'switch-request)))
  "Returns md5sum for a message object of type 'switch-request"
  "9ae8834e0ab84ed7c29d5f2c87a2b979")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<switch-request>)))
  "Returns full string definition for message of type '<switch-request>"
  (cl:format cl:nil "uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'switch-request)))
  "Returns full string definition for message of type 'switch-request"
  (cl:format cl:nil "uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <switch-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <switch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'switch-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude switch-response.msg.html

(cl:defclass <switch-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:fixnum
    :initform 0))
)

(cl:defclass switch-response (<switch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <switch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'switch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rmus_solution-srv:<switch-response> is deprecated: use rmus_solution-srv:switch-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <switch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rmus_solution-srv:success-val is deprecated.  Use rmus_solution-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <switch-response>) ostream)
  "Serializes a message object of type '<switch-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <switch-response>) istream)
  "Deserializes a message object of type '<switch-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'success)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<switch-response>)))
  "Returns string type for a service object of type '<switch-response>"
  "rmus_solution/switchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'switch-response)))
  "Returns string type for a service object of type 'switch-response"
  "rmus_solution/switchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<switch-response>)))
  "Returns md5sum for a message object of type '<switch-response>"
  "9ae8834e0ab84ed7c29d5f2c87a2b979")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'switch-response)))
  "Returns md5sum for a message object of type 'switch-response"
  "9ae8834e0ab84ed7c29d5f2c87a2b979")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<switch-response>)))
  "Returns full string definition for message of type '<switch-response>"
  (cl:format cl:nil "uint8 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'switch-response)))
  "Returns full string definition for message of type 'switch-response"
  (cl:format cl:nil "uint8 success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <switch-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <switch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'switch-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'switch)))
  'switch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'switch)))
  'switch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'switch)))
  "Returns string type for a service object of type '<switch>"
  "rmus_solution/switch")