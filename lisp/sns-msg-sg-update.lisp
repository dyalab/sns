(in-package :sns)

(defstruct sns-sg-update
  frame
  parent
  q
  v
  copy-frame
  name)

(defun get-sg-update-frame (ptr)
   (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'frame))

(defun get-sg-update-parent (ptr)
   (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'parent))

(defun get-sg-update-q (ptr)
  (msg-decode 'quaternion
              (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'q)))

(defun get-sg-update-v (ptr)
  (msg-decode 'vec3
              (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'v)))

(defun get-sg-copy-frame (ptr)
  (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'copy-frame))

(defun get-sg-update-name (ptr)
  (convert-from-foreign
   (foreign-slot-pointer ptr '(:struct sns-msg-sg-update) 'name)
   :string))


(defun read-sns-msg-sg-update (chan &key (wait nil) (last nil))
  (multiple-value-bind (ptr r frame-size)
      (ach::get-foreign-alloc chan :wait wait :last last)
    (declare (ignore frame-size))
    (if (or (eq r :ok)
            (eq r :missed-frame))
        (let* ((frame      (get-sg-update-frame ptr))
               (parent     (get-sg-update-parent ptr))
               (q          (get-sg-update-q ptr))
               (v          (get-sg-update-v ptr))
               (copy-frame (get-sg-update-copy-frame ptr))
               (name       (get-sg-update-name ptr))
               (sg         (make-sns-sg-update)))
          (setf (sns-sg-udpate-frame sg)      frame
                (sns-sg-update-parent sg)     parent
                (sns-sg-update-q sg)          q
                (sns-sg-update-v sg)          v
                (sns-sg-update-copy-frame sg) copy-frame
                (sns-sg-update-name sg)       name)
          (value r sg)
          (values r nil)))))

  (defun write-sns-msg-sg-update (chan data)
    (let* ((ptr (sns-msg-text-heap-alloc (+ (length (sns-sg-update-name data)) 1)))
           (txt (foreign-slot-pointer ptr '(:struct sns-msg-sg-update) 'name))
           (q (foreign-slot-pointer ptr '(:struct sns-msg-sg-update) 'q))
           (v (foreign-slot-pointer ptr '(:struct sns-msg-sg-update) 'v))
           (i 0))
      (loop for el across (sns-sg-update-name data)
         do (setf (mem-aref txt
                            :char i)
                  (char-code el))
         do (incf i 1))

      (setf i 0)
      (loop for val in (sns-msg-sg-update-q)
         do (setf (mem-aref q
                            :double i)
                  val)
         do (incf i 1))

      (setf i 0)
      (loop for val in (sns-msg-sg-update-v)
         do (setf (mem-aref v
                            :double i)
                  val)
         do (incf i 1))


      (setf (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'type)
            (sns-sg-update-type data)

            (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'frame)
            (sns-sg-update-frame data)

            (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'parent)
            (sns-sg-update-parent data)

            (foreign-slot-value ptr '(:struct sns-msg-sg-update) 'copy-frame)
            (sns-sg-update-copy-frame data))


      (ach::check-status (sns-msg-text-put (ach::ach-handle-pointer chan)
                                           ptr)
                         "Failed to put message on channel")))
