;; Auto-generated. Do not edit!


(when (boundp 'rmus_solution::setgoal)
  (if (not (find-package "RMUS_SOLUTION"))
    (make-package "RMUS_SOLUTION"))
  (shadow 'setgoal (find-package "RMUS_SOLUTION")))
(unless (find-package "RMUS_SOLUTION::SETGOAL")
  (make-package "RMUS_SOLUTION::SETGOAL"))
(unless (find-package "RMUS_SOLUTION::SETGOALREQUEST")
  (make-package "RMUS_SOLUTION::SETGOALREQUEST"))
(unless (find-package "RMUS_SOLUTION::SETGOALRESPONSE")
  (make-package "RMUS_SOLUTION::SETGOALRESPONSE"))

(in-package "ROS")





(defclass rmus_solution::setgoalRequest
  :super ros::object
  :slots (_point _call ))

(defmethod rmus_solution::setgoalRequest
  (:init
   (&key
    ((:point __point) 0)
    ((:call __call) "")
    )
   (send-super :init)
   (setq _point (round __point))
   (setq _call (string __call))
   self)
  (:point
   (&optional __point)
   (if __point (setq _point __point)) _point)
  (:call
   (&optional __call)
   (if __call (setq _call __call)) _call)
  (:serialization-length
   ()
   (+
    ;; int32 _point
    4
    ;; string _call
    4 (length _call)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _point
       (write-long _point s)
     ;; string _call
       (write-long (length _call) s) (princ _call s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _point
     (setq _point (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _call
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _call (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rmus_solution::setgoalResponse
  :super ros::object
  :slots (_res _response ))

(defmethod rmus_solution::setgoalResponse
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

(defclass rmus_solution::setgoal
  :super ros::object
  :slots ())

(setf (get rmus_solution::setgoal :md5sum-) "5dd6c60142d9c86b4dfc9ca1994ff38a")
(setf (get rmus_solution::setgoal :datatype-) "rmus_solution/setgoal")
(setf (get rmus_solution::setgoal :request) rmus_solution::setgoalRequest)
(setf (get rmus_solution::setgoal :response) rmus_solution::setgoalResponse)

(defmethod rmus_solution::setgoalRequest
  (:response () (instance rmus_solution::setgoalResponse :init)))

(setf (get rmus_solution::setgoalRequest :md5sum-) "5dd6c60142d9c86b4dfc9ca1994ff38a")
(setf (get rmus_solution::setgoalRequest :datatype-) "rmus_solution/setgoalRequest")
(setf (get rmus_solution::setgoalRequest :definition-)
      "int32 point
string call
---
bool res
string response

")

(setf (get rmus_solution::setgoalResponse :md5sum-) "5dd6c60142d9c86b4dfc9ca1994ff38a")
(setf (get rmus_solution::setgoalResponse :datatype-) "rmus_solution/setgoalResponse")
(setf (get rmus_solution::setgoalResponse :definition-)
      "int32 point
string call
---
bool res
string response

")



(provide :rmus_solution/setgoal "5dd6c60142d9c86b4dfc9ca1994ff38a")


