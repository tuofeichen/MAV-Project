
(cl:in-package :asdf)

(defsystem "px4_offboard-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JoyCommand" :depends-on ("_package_JoyCommand"))
    (:file "_package_JoyCommand" :depends-on ("_package"))
    (:file "MoveCommand" :depends-on ("_package_MoveCommand"))
    (:file "_package_MoveCommand" :depends-on ("_package"))
  ))