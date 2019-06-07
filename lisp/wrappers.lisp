;;;; -*- Lisp -*-
;;;;
;;;; Copyright (c) 2019, Colorado School of Mines
;;;; All rights reserved.
;;;;
;;;; Author(s): Matthew A. Schack <mschack@mines.edu>
;;;;
;;;;
;;;; This file is provided under the following "BSD-style" License:
;;;;
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above copyright
;;;;     notice, this list of conditions and the following disclaimer.
;;;;
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials provided
;;;;     with the distribution.
;;;;
;;;;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
;;;;   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
;;;;   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
;;;;   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;;;;   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;;;   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;;;;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;;;;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
;;;;   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;;;;   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;;;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;;;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;;;   POSSIBILITY OF SUCH DAMAGE.


;;;; These functions are defined as static inline functions in a .h
;;;; file. As such aren't loaded when we load the shared object. So we
;;;; have to wrap them in another c function in order for Grovel to
;;;; accept them.

;; Basically Grovel is going to create a shared object with these
;; functions wrapped in another function so they are no longer static
;; inline functions. Then load and define them like we would any other
;; c-function.
(progn
  (in-package :sns)
  (flag "--std=gnu99")
  (flag #.(concatenate 'string
		       "-I"
		       "/usr/local/include/amino-1.0"))
  (flag "-lach -lsns")

  (include "stdlib.h")
  (include "sns.h")
  #.(cons 'progn (loop for x in '("sns_msg_log" "sns_msg_vector" "sns_msg_tf" "sns_msg_wt_tf"
		 "sns_msg_tf_dx" "sns_msg_motor_ref" "sns_msg_tag_motor_ref"
		 "sns_msg_motor_state" "sns_msg_joystick")
   append (loop for (func ret inp) in '(("_size_n" uint32-t ((n uint32-t)))
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
	 collect (let* ((name (concatenate 'string x func))
		   (symb-name (intern (string-upcase (substitute #\- #\_ name)))))
	      (append `(defwrapper (,name ,symb-name) ,ret)
		      inp)))))
  )
