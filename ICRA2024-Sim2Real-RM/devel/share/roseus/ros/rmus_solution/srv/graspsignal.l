;; Auto-generated. Do not edit!


(when (boundp 'rmus_solution::graspsignal)
  (if (not (find-package "RMUS_SOLUTION"))
    (make-package "RMUS_SOLUTION"))
  (shadow 'graspsignal (find-package "RMUS_SOLUTION")))
(unless (find-package "RMUS_SOLUTION::GRASPSIGNAL")
  (make-package "RMUS_SOLUTION::GRASPSIGNAL"))
(unless (find-package "RMUS_SOLUTION::GRASPSIGNALREQUEST")
  (make-package "RMUS_SOLUTION::GRASPSIGNALREQUEST"))
(unless (find-package "RMUS_SOLUTION::GRASPSIGNALRESPONSE")
  (make-package "RMUS_SOLUTION::GRASPSIGNALRESPONSE"))

(in-package "ROS")





(defclass rmus_solution::graspsignalRequest
  :super ros::object
  :slots (_mode _call ))

(defmethod rmus_solution::graspsignalRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:call __call) "")
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _call (string __call))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:call
   (&optional __call)
   (if __call (setq _call __call)) _call)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; string _call
    4 (length _call)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; string _call
       (write-long (length _call) s) (princ _call s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _call
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _call (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rmus_solution::graspsignalResponse
  :super ros::object
  :slots (_res _response ))

(defmethod rmus_solution::graspsignalResponse
  (:init
   (&key
    ((:res __res) nil)
    ((:response __response) "")
    )
   (send-super :init)
   (setq _res __res)
   (setq _response (string __response))
   self)
  (:res
   (&optional (__res :null))
   (if (not (eq __res :null)) (setq _res __res)) _res)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; bool _res
    1
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _res
       (if _res (write-byte -1 s) (write-byte 0 s))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _res
     (setq _res (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rmus_solution::graspsignal
  :super ros::object
  :slots ())

(setf (get rmus_solution::graspsignal :md5sum-) "ba5c545fc7cb2bb25e9f962a2a65350e")
(setf (get rmus_solution::graspsignal :datatype-) "rmus_solution/graspsignal")
(setf (get rmus_solution::graspsignal :request) rmus_solution::graspsignalRequest)
(setf (get rmus_solution::graspsignal :response) rmus_solution::graspsignalResponse)

(defmethod rmus_solution::graspsignalRequest
  (:response () (instance rmus_solution::graspsignalResponse :init)))

(setf (get rmus_solution::graspsignalRequest :md5sum-) "ba5c545fc7cb2bb25e9f962a2a65350e")
(setf (get rmus_solution::graspsignalRequest :datatype-) "rmus_solution/graspsignalRequest")
(setf (get rmus_solution::graspsignalRequest :definition-)
      "uint8 mode
string call
---
bool res
string response
")

(setf (get rmus_solution::graspsignalResponse :md5sum-) "ba5c545fc7cb2bb25e9f962a2a65350e")
(setf (get rmus_solution::graspsignalResponse :datatype-) "rmus_solution/graspsignalResponse")
(setf (get rmus_solution::graspsignalResponse :definition-)
      "uint8 mode
string call
---
bool res
string response
")



(provide :rmus_solution/graspsignal "ba5c545fc7cb2bb25e9f962a2a65350e")


