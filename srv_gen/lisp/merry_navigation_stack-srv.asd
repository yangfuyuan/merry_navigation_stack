
(cl:in-package :asdf)

(defsystem "merry_navigation_stack-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "QueryForFrontiers" :depends-on ("_package_QueryForFrontiers"))
    (:file "_package_QueryForFrontiers" :depends-on ("_package"))
  ))