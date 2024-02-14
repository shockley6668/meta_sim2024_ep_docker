;; Auto-generated. Do not edit!


(when (boundp 'rmus_solution::switch)
  (if (not (find-package "RMUS_SOLUTION"))
    (make-package "RMUS_SOLUTION"))
  (shadow 'switch (find-package "RMUS_SOLUTION")))
(unless (find-package "RMUS_SOLUTION::SWITCH")
  (make-package "RMUS_SOLUTION::SWITCH"))
(unless (find-package "RMUS_SOLUTION::SWITCHREQUEST")
  (make-package "RMUS_SOLUTION::SWITCHREQUEST"))
(unless (find-package "RMUS_SOLUTION::SWITCHRESPONSE")
  (make-package "RMUS_SOLUTION::SWITCHRESPONSE"))

(in-package "ROS")





(defclass rmus_solution::switchRequest
  :super ros::object
  :slots (_mode ))

(defmethod rmus_solution::switchRequest
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass rmus_solution::switchResponse
  :super ros::object
  :slots (_success ))

(defmethod rmus_solution::switchResponse
  (:init
   (&key
    ((:success __success) 0)
    )
   (send-super :init)
   (setq _success (round __success))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; uint8 _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _success
       (write-byte _success s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _success
     (setq _success (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass rmus_solution::switch
  :super ros::object
  :slots ())

(setf (get rmus_solution::switch :md5sum-) "9ae8834e0ab84ed7c29d5f2c87a2b979")
(setf (get rmus_solution::switch :datatype-) "rmus_solution/switch")
(setf (get rmus_solution::switch :request) rmus_solution::switchRequest)
(setf (get rmus_solution::switch :response) rmus_solution::switchResponse)

(defmethod rmus_solution::switchRequest
  (:response () (instance rmus_solution::switchResponse :init)))

(setf (get rmus_solution::switchRequest :md5sum-) "9ae8834e0ab84ed7c29d5f2c87a2b979")
(setf (get rmus_solution::switchRequest :datatype-) "rmus_solution/switchRequest")
(setf (get rmus_solution::switchRequest :definition-)
      "uint8 mode
---
uint8 success
")

(setf (get rmus_solution::switchResponse :md5sum-) "9ae8834e0ab84ed7c29d5f2c87a2b979")
(setf (get rmus_solution::switchResponse :datatype-) "rmus_solution/switchResponse")
(setf (get rmus_solution::switchResponse :definition-)
      "uint8 mode
---
uint8 success
")



(provide :rmus_solution/switch "9ae8834e0ab84ed7c29d5f2c87a2b979")


