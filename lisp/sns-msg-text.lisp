(in-package :sns)

(defun get-text (ptr)
  (foreign-slot-pointer ptr '(:struct sns-msg-text) 'text))

(defun read-sns-msg-text (chan &key (wait nil) (last nil))
  (multiple-value-bind (ptr r frame-size)
      (ach::get-foreign-alloc chan :wait wait :last last)
    (declare (ignore frame-size))
    (if (or (eq r :ok)
              (eq r :missed-frame))
      (let* ((txt (sns::get-text ptr)))
        (values r (convert-from-foreign txt :string)))
      (values r nil))))

(defun write-sns-msg-text (chan data)
  (let* ((ptr (sns-msg-text-heap-alloc (+ (length data) 1)))
         (txt (foreign-slot-pointer ptr '(:struct sns-msg-text) 'text))
         (i 0))
    (loop for el across data
       do (setf (mem-aref txt
                          :char i)
                (char-code el))
       do (incf i 1))
    (ach::check-status (sns-msg-text-put (ach::ach-handle-pointer chan)
                                         ptr)
                       "Failed to put message on channel")))
