;; Auto-generated. Do not edit!


(when (boundp 'gapwatch_wrapper::EMG)
  (if (not (find-package "GAPWATCH_WRAPPER"))
    (make-package "GAPWATCH_WRAPPER"))
  (shadow 'EMG (find-package "GAPWATCH_WRAPPER")))
(unless (find-package "GAPWATCH_WRAPPER::EMG")
  (make-package "GAPWATCH_WRAPPER::EMG"))

(in-package "ROS")
;;//! \htmlinclude EMG.msg.html


(defclass gapwatch_wrapper::EMG
  :super ros::object
  :slots (_emg _battery _counter _ts ))

(defmethod gapwatch_wrapper::EMG
  (:init
   (&key
    ((:emg __emg) (make-array 80 :initial-element 0.0 :element-type :float))
    ((:battery __battery) (make-array 1 :initial-element 0 :element-type :char))
    ((:counter __counter) (make-array 1 :initial-element 0 :element-type :char))
    ((:ts __ts) (make-array 1 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _emg __emg)
   (setq _battery __battery)
   (setq _counter __counter)
   (setq _ts __ts)
   self)
  (:emg
   (&optional __emg)
   (if __emg (setq _emg __emg)) _emg)
  (:battery
   (&optional __battery)
   (if __battery (setq _battery __battery)) _battery)
  (:counter
   (&optional __counter)
   (if __counter (setq _counter __counter)) _counter)
  (:ts
   (&optional __ts)
   (if __ts (setq _ts __ts)) _ts)
  (:serialization-length
   ()
   (+
    ;; float32[80] _emg
    (* 4    80)
    ;; uint8[1] _battery
    (* 1    1)
    ;; uint8[1] _counter
    (* 1    1)
    ;; uint64[1] _ts
    (* 8    1)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[80] _emg
     (dotimes (i 80)
       (sys::poke (elt _emg i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint8[1] _battery
     (princ _battery s)
     ;; uint8[1] _counter
     (princ _counter s)
     ;; uint64[1] _ts
     (dotimes (i 1)
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _ts i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _ts i)) (= (length ((elt _ts i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _ts i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _ts i) . bv) 1) -1) s))
             ((and (class (elt _ts i)) (= (length ((elt _ts i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _ts i) . bv) 0) s)
              (write-long (if (>= (elt _ts i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _ts i) s)(write-long (if (>= (elt _ts i) 0) 0 #xffffffff) s)))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[80] _emg
   (dotimes (i (length _emg))
     (setf (elt _emg i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; uint8[1] _battery
   (setq _battery (make-array 1 :element-type :char))
   (replace _battery buf :start2 ptr-) (incf ptr- 1)
   ;; uint8[1] _counter
   (setq _counter (make-array 1 :element-type :char))
   (replace _counter buf :start2 ptr-) (incf ptr- 1)
   ;; uint64[1] _ts
   (dotimes (i (length _ts))
     
#+(or :alpha :irix6 :x86_64)
      (setf (elt _ts i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _ts i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     )
   ;;
   self)
  )

(setf (get gapwatch_wrapper::EMG :md5sum-) "a78d4218423202e0323f407914156a1d")
(setf (get gapwatch_wrapper::EMG :datatype-) "gapwatch_wrapper/EMG")
(setf (get gapwatch_wrapper::EMG :definition-)
      "float32[80] emg
uint8[1] battery
uint8[1] counter
uint64[1] ts


")



(provide :gapwatch_wrapper/EMG "a78d4218423202e0323f407914156a1d")


