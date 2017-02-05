
(cl:in-package :asdf)

(defsystem "px4_offboard-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JoyCommand" :depends-on ("_package_JoyCommand"))
    (:file "_package_JoyCommand" :depends-on ("_package"))
    (:file "CtrlState" :depends-on ("_package_CtrlState"))
    (:file "_package_CtrlState" :depends-on ("_package"))
  ))