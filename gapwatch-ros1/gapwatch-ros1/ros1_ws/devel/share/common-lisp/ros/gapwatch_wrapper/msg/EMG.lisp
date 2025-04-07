; Auto-generated. Do not edit!


(cl:in-package gapwatch_wrapper-msg)


;//! \htmlinclude EMG.msg.html

(cl:defclass <EMG> (roslisp-msg-protocol:ros-message)
  ((emg
    :reader emg
    :initarg :emg
    :type (cl:vector cl:float)
   :initform (cl:make-array 80 :element-type 'cl:float :initial-element 0.0))
   (battery
    :reader battery
    :initarg :battery
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 1 :element-type 'cl:fixnum :initial-element 0))
   (counter
    :reader counter
    :initarg :counter
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 1 :element-type 'cl:fixnum :initial-element 0))
   (ts
    :reader ts
    :initarg :ts
    :type (cl:vector cl:integer)
   :initform (cl:make-array 1 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass EMG (<EMG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EMG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EMG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gapwatch_wrapper-msg:<EMG> is deprecated: use gapwatch_wrapper-msg:EMG instead.")))

(cl:ensure-generic-function 'emg-val :lambda-list '(m))
(cl:defmethod emg-val ((m <EMG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gapwatch_wrapper-msg:emg-val is deprecated.  Use gapwatch_wrapper-msg:emg instead.")
  (emg m))

(cl:ensure-generic-function 'battery-val :lambda-list '(m))
(cl:defmethod battery-val ((m <EMG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gapwatch_wrapper-msg:battery-val is deprecated.  Use gapwatch_wrapper-msg:battery instead.")
  (battery m))

(cl:ensure-generic-function 'counter-val :lambda-list '(m))
(cl:defmethod counter-val ((m <EMG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gapwatch_wrapper-msg:counter-val is deprecated.  Use gapwatch_wrapper-msg:counter instead.")
  (counter m))

(cl:ensure-generic-function 'ts-val :lambda-list '(m))
(cl:defmethod ts-val ((m <EMG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gapwatch_wrapper-msg:ts-val is deprecated.  Use gapwatch_wrapper-msg:ts instead.")
  (ts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EMG>) ostream)
  "Serializes a message object of type '<EMG>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'emg))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'battery))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'counter))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) ele) ostream))
   (cl:slot-value msg 'ts))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EMG>) istream)
  "Deserializes a message object of type '<EMG>"
  (cl:setf (cl:slot-value msg 'emg) (cl:make-array 80))
  (cl:let ((vals (cl:slot-value msg 'emg)))
    (cl:dotimes (i 80)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'battery) (cl:make-array 1))
  (cl:let ((vals (cl:slot-value msg 'battery)))
    (cl:dotimes (i 1)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'counter) (cl:make-array 1))
  (cl:let ((vals (cl:slot-value msg 'counter)))
    (cl:dotimes (i 1)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  (cl:setf (cl:slot-value msg 'ts) (cl:make-array 1))
  (cl:let ((vals (cl:slot-value msg 'ts)))
    (cl:dotimes (i 1)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EMG>)))
  "Returns string type for a message object of type '<EMG>"
  "gapwatch_wrapper/EMG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EMG)))
  "Returns string type for a message object of type 'EMG"
  "gapwatch_wrapper/EMG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EMG>)))
  "Returns md5sum for a message object of type '<EMG>"
  "a78d4218423202e0323f407914156a1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EMG)))
  "Returns md5sum for a message object of type 'EMG"
  "a78d4218423202e0323f407914156a1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EMG>)))
  "Returns full string definition for message of type '<EMG>"
  (cl:format cl:nil "float32[80] emg~%uint8[1] battery~%uint8[1] counter~%uint64[1] ts~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EMG)))
  "Returns full string definition for message of type 'EMG"
  (cl:format cl:nil "float32[80] emg~%uint8[1] battery~%uint8[1] counter~%uint64[1] ts~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EMG>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'emg) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'battery) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'counter) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'ts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EMG>))
  "Converts a ROS message object to a list"
  (cl:list 'EMG
    (cl:cons ':emg (emg msg))
    (cl:cons ':battery (battery msg))
    (cl:cons ':counter (counter msg))
    (cl:cons ':ts (ts msg))
))
