; Auto-generated. Do not edit!


(cl:in-package rmus_solution-srv)


;//! \htmlinclude graspsignal-request.msg.html

(cl:defclass <graspsignal-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (call
    :reader call
    :initarg :call
    :type cl:string
    :initform ""))
)

(cl:defclass graspsignal-request (<graspsignal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <graspsignal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'graspsignal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rmus_solution-srv:<graspsignal-request> is deprecated: use rmus_solution-srv:graspsignal-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <graspsignal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rmus_solution-srv:mode-val is deprecated.  Use rmus_solution-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'call-val :lambda-list '(m))
(cl:defmethod call-val ((m <graspsignal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rmus_solution-srv:call-val is deprecated.  Use rmus_solution-srv:call instead.")
  (call m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <graspsignal-request>) ostream)
  "Serializes a message object of type '<graspsignal-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'call))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'call))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <graspsignal-request>) istream)
  "Deserializes a message object of type '<graspsignal-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'call) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'call) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<graspsignal-request>)))
  "Returns string type for a service object of type '<graspsignal-request>"
  "rmus_solution/graspsignalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'graspsignal-request)))
  "Returns string type for a service object of type 'graspsignal-request"
  "rmus_solution/graspsignalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<graspsignal-request>)))
  "Returns md5sum for a message object of type '<graspsignal-request>"
  "ba5c545fc7cb2bb25e9f962a2a65350e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'graspsignal-request)))
  "Returns md5sum for a message object of type 'graspsignal-request"
  "ba5c545fc7cb2bb25e9f962a2a65350e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<graspsignal-request>)))
  "Returns full string definition for message of type '<graspsignal-request>"
  (cl:format cl:nil "uint8 mode~%string call~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'graspsignal-request)))
  "Returns full string definition for message of type 'graspsignal-request"
  (cl:format cl:nil "uint8 mode~%string call~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <graspsignal-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'call))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <graspsignal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'graspsignal-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':call (call msg))
))
;//! \htmlinclude graspsignal-response.msg.html

(cl:defclass <graspsignal-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type cl:boolean
    :initform cl:nil)
   (response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass graspsignal-response (<graspsignal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <graspsignal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'graspsignal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rmus_solution-srv:<graspsignal-response> is deprecated: use rmus_solution-srv:graspsignal-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <graspsignal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rmus_solution-srv:res-val is deprecated.  Use rmus_solution-srv:res instead.")
  (res m))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <graspsignal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rmus_solution-srv:response-val is deprecated.  Use rmus_solution-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <graspsignal-response>) ostream)
  "Serializes a message object of type '<graspsignal-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'res) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <graspsignal-response>) istream)
  "Deserializes a message object of type '<graspsignal-response>"
    (cl:setf (cl:slot-value msg 'res) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<graspsignal-response>)))
  "Returns string type for a service object of type '<graspsignal-response>"
  "rmus_solution/graspsignalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'graspsignal-response)))
  "Returns string type for a service object of type 'graspsignal-response"
  "rmus_solution/graspsignalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<graspsignal-response>)))
  "Returns md5sum for a message object of type '<graspsignal-response>"
  "ba5c545fc7cb2bb25e9f962a2a65350e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'graspsignal-response)))
  "Returns md5sum for a message object of type 'graspsignal-response"
  "ba5c545fc7cb2bb25e9f962a2a65350e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<graspsignal-response>)))
  "Returns full string definition for message of type '<graspsignal-response>"
  (cl:format cl:nil "bool res~%string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'graspsignal-response)))
  "Returns full string definition for message of type 'graspsignal-response"
  (cl:format cl:nil "bool res~%string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <graspsignal-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <graspsignal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'graspsignal-response
    (cl:cons ':res (res msg))
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'graspsignal)))
  'graspsignal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'graspsignal)))
  'graspsignal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'graspsignal)))
  "Returns string type for a service object of type '<graspsignal>"
  "rmus_solution/graspsignal")