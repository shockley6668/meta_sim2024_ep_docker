
(cl:in-package :asdf)

(defsystem "rmus_solution-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "graspsignal" :depends-on ("_package_graspsignal"))
    (:file "_package_graspsignal" :depends-on ("_package"))
    (:file "setgoal" :depends-on ("_package_setgoal"))
    (:file "_package_setgoal" :depends-on ("_package"))
    (:file "switch" :depends-on ("_package_switch"))
    (:file "_package_switch" :depends-on ("_package"))
  ))