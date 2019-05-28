(loop for x in '("sns_msg_log" "sns_msg_vector" "sns_msg_tf" "sns_msg_wt_tf"
		 "sns_msg_tf_dx" "sns_msg_motor_ref" "sns_msg_tag_motor_ref"
		 "sns_msg_motor_state" "sns_msg_joystick")
   do (loop for (func ret inp) in '(("_size_n" uint32-t ((n uint32-t)))
				    ("_size" uint32-t ((msg :pointer)))
				    ("_check_size" ssize-t ((msg :pointer)(mem_size size-t)))
				    ("_init" :void ((msg :pointer)(n uint32-t)))
				    ("_heap_alloc" :pointer ((n uint32-t)))
				    ("_region_alloc" :pointer ((reg :pointer)(n uint32-t)))
				    ("_local_alloc" :pointer ((n uint32-t)))
				    ("_put" ach::ach-status-t ((chan :pointer)(msg :pointer)))
				    ("_local_get" ach::ach-status-t
				     ((chan :pointer)(pmsg :pointer)(frame-size :pointer)
				      (timespec :pointer)(options :int))))
	 do (let* ((name (concatenate 'string x func))
		   (symb-name (intern (string-upcase (substitute #\- #\_ name)))))
	      (format t "~S~%" (append `(defwrapper (,name ,symb-name) ,ret)
		      inp)))))
