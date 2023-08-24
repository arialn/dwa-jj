
(cl:in-package :asdf)

(defsystem "tbug-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "goalStatusAction" :depends-on ("_package_goalStatusAction"))
    (:file "_package_goalStatusAction" :depends-on ("_package"))
    (:file "goalStatusActionFeedback" :depends-on ("_package_goalStatusActionFeedback"))
    (:file "_package_goalStatusActionFeedback" :depends-on ("_package"))
    (:file "goalStatusActionGoal" :depends-on ("_package_goalStatusActionGoal"))
    (:file "_package_goalStatusActionGoal" :depends-on ("_package"))
    (:file "goalStatusActionResult" :depends-on ("_package_goalStatusActionResult"))
    (:file "_package_goalStatusActionResult" :depends-on ("_package"))
    (:file "goalStatusFeedback" :depends-on ("_package_goalStatusFeedback"))
    (:file "_package_goalStatusFeedback" :depends-on ("_package"))
    (:file "goalStatusGoal" :depends-on ("_package_goalStatusGoal"))
    (:file "_package_goalStatusGoal" :depends-on ("_package"))
    (:file "goalStatusResult" :depends-on ("_package_goalStatusResult"))
    (:file "_package_goalStatusResult" :depends-on ("_package"))
  ))