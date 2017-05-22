
(cl:in-package :asdf)

(defsystem "manyears_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AudioStream" :depends-on ("_package_AudioStream"))
    (:file "_package_AudioStream" :depends-on ("_package"))
    (:file "ManyEarsTrackedAudioSource" :depends-on ("_package_ManyEarsTrackedAudioSource"))
    (:file "_package_ManyEarsTrackedAudioSource" :depends-on ("_package"))
    (:file "SourceInfo" :depends-on ("_package_SourceInfo"))
    (:file "_package_SourceInfo" :depends-on ("_package"))
  ))