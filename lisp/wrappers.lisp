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

  (DEFWRAPPER ("sns_msg_log_size_n" SNS-MSG-LOG-SIZE-N) UINT32-T (N UINT32-T))
  (DEFWRAPPER ("sns_msg_log_size" SNS-MSG-LOG-SIZE) UINT32-T (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_log_check_size" SNS-MSG-LOG-CHECK-SIZE) SSIZE-T
    (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_log_init" SNS-MSG-LOG-INIT) :VOID (MSG :POINTER)
	      (N UINT32-T))
  (DEFWRAPPER ("sns_msg_log_heap_alloc" SNS-MSG-LOG-HEAP-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_log_region_alloc" SNS-MSG-LOG-REGION-ALLOC) :POINTER
    (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_log_local_alloc" SNS-MSG-LOG-LOCAL-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_log_put" SNS-MSG-LOG-PUT) ACH::ACH-STATUS-T
    (CHAN :POINTER) (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_log_local_get" SNS-MSG-LOG-LOCAL-GET) ACH::ACH-STATUS-T
    (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER) (TIMESPEC :POINTER)
    (OPTIONS :INT))
  (DEFWRAPPER ("sns_msg_vector_size_n" SNS-MSG-VECTOR-SIZE-N) UINT32-T
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_vector_size" SNS-MSG-VECTOR-SIZE) UINT32-T (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_vector_check_size" SNS-MSG-VECTOR-CHECK-SIZE) SSIZE-T
    (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_vector_init" SNS-MSG-VECTOR-INIT) :VOID (MSG :POINTER)
	      (N UINT32-T))
  (DEFWRAPPER ("sns_msg_vector_heap_alloc" SNS-MSG-VECTOR-HEAP-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_vector_region_alloc" SNS-MSG-VECTOR-REGION-ALLOC)
      :POINTER (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_vector_local_alloc" SNS-MSG-VECTOR-LOCAL-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_vector_put" SNS-MSG-VECTOR-PUT) ACH::ACH-STATUS-T
    (CHAN :POINTER) (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_vector_local_get" SNS-MSG-VECTOR-LOCAL-GET)
      ACH::ACH-STATUS-T (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER)
      (TIMESPEC :POINTER) (OPTIONS :INT))
  (DEFWRAPPER ("sns_msg_tf_size_n" SNS-MSG-TF-SIZE-N) UINT32-T (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_size" SNS-MSG-TF-SIZE) UINT32-T (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_tf_check_size" SNS-MSG-TF-CHECK-SIZE) SSIZE-T
    (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_tf_init" SNS-MSG-TF-INIT) :VOID (MSG :POINTER)
	      (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_heap_alloc" SNS-MSG-TF-HEAP-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_region_alloc" SNS-MSG-TF-REGION-ALLOC) :POINTER
    (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_local_alloc" SNS-MSG-TF-LOCAL-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_put" SNS-MSG-TF-PUT) ACH::ACH-STATUS-T (CHAN :POINTER)
	      (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_tf_local_get" SNS-MSG-TF-LOCAL-GET) ACH::ACH-STATUS-T
    (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER) (TIMESPEC :POINTER)
    (OPTIONS :INT))
  (DEFWRAPPER ("sns_msg_wt_tf_size_n" SNS-MSG-WT-TF-SIZE-N) UINT32-T (N UINT32-T))
  (DEFWRAPPER ("sns_msg_wt_tf_size" SNS-MSG-WT-TF-SIZE) UINT32-T (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_wt_tf_check_size" SNS-MSG-WT-TF-CHECK-SIZE) SSIZE-T
    (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_wt_tf_init" SNS-MSG-WT-TF-INIT) :VOID (MSG :POINTER)
	      (N UINT32-T))
  (DEFWRAPPER ("sns_msg_wt_tf_heap_alloc" SNS-MSG-WT-TF-HEAP-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_wt_tf_region_alloc" SNS-MSG-WT-TF-REGION-ALLOC) :POINTER
    (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_wt_tf_local_alloc" SNS-MSG-WT-TF-LOCAL-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_wt_tf_put" SNS-MSG-WT-TF-PUT) ACH::ACH-STATUS-T
    (CHAN :POINTER) (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_wt_tf_local_get" SNS-MSG-WT-TF-LOCAL-GET)
      ACH::ACH-STATUS-T (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER)
      (TIMESPEC :POINTER) (OPTIONS :INT))
  (DEFWRAPPER ("sns_msg_tf_dx_size_n" SNS-MSG-TF-DX-SIZE-N) UINT32-T (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_dx_size" SNS-MSG-TF-DX-SIZE) UINT32-T (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_tf_dx_check_size" SNS-MSG-TF-DX-CHECK-SIZE) SSIZE-T
    (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_tf_dx_init" SNS-MSG-TF-DX-INIT) :VOID (MSG :POINTER)
	      (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_dx_heap_alloc" SNS-MSG-TF-DX-HEAP-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_dx_region_alloc" SNS-MSG-TF-DX-REGION-ALLOC) :POINTER
    (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_dx_local_alloc" SNS-MSG-TF-DX-LOCAL-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tf_dx_put" SNS-MSG-TF-DX-PUT) ACH::ACH-STATUS-T
    (CHAN :POINTER) (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_tf_dx_local_get" SNS-MSG-TF-DX-LOCAL-GET)
      ACH::ACH-STATUS-T (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER)
      (TIMESPEC :POINTER) (OPTIONS :INT))
  (DEFWRAPPER ("sns_msg_motor_ref_size_n" SNS-MSG-MOTOR-REF-SIZE-N) UINT32-T
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_ref_size" SNS-MSG-MOTOR-REF-SIZE) UINT32-T
    (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_motor_ref_check_size" SNS-MSG-MOTOR-REF-CHECK-SIZE)
      SSIZE-T (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_motor_ref_init" SNS-MSG-MOTOR-REF-INIT) :VOID
    (MSG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_ref_heap_alloc" SNS-MSG-MOTOR-REF-HEAP-ALLOC)
      :POINTER (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_ref_region_alloc" SNS-MSG-MOTOR-REF-REGION-ALLOC)
      :POINTER (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_ref_local_alloc" SNS-MSG-MOTOR-REF-LOCAL-ALLOC)
      :POINTER (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_ref_put" SNS-MSG-MOTOR-REF-PUT) ACH::ACH-STATUS-T
    (CHAN :POINTER) (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_motor_ref_local_get" SNS-MSG-MOTOR-REF-LOCAL-GET)
      ACH::ACH-STATUS-T (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER)
      (TIMESPEC :POINTER) (OPTIONS :INT))
  (DEFWRAPPER ("sns_msg_tag_motor_ref_size_n" SNS-MSG-TAG-MOTOR-REF-SIZE-N)
      UINT32-T (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tag_motor_ref_size" SNS-MSG-TAG-MOTOR-REF-SIZE) UINT32-T
    (MSG :POINTER))
  (DEFWRAPPER
      ("sns_msg_tag_motor_ref_check_size" SNS-MSG-TAG-MOTOR-REF-CHECK-SIZE) SSIZE-T
    (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_tag_motor_ref_init" SNS-MSG-TAG-MOTOR-REF-INIT) :VOID
    (MSG :POINTER) (N UINT32-T))
  (DEFWRAPPER
      ("sns_msg_tag_motor_ref_heap_alloc" SNS-MSG-TAG-MOTOR-REF-HEAP-ALLOC) :POINTER
    (N UINT32-T))
  (DEFWRAPPER
      ("sns_msg_tag_motor_ref_region_alloc" SNS-MSG-TAG-MOTOR-REF-REGION-ALLOC)
      :POINTER (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER
      ("sns_msg_tag_motor_ref_local_alloc" SNS-MSG-TAG-MOTOR-REF-LOCAL-ALLOC)
      :POINTER (N UINT32-T))
  (DEFWRAPPER ("sns_msg_tag_motor_ref_put" SNS-MSG-TAG-MOTOR-REF-PUT)
      ACH::ACH-STATUS-T (CHAN :POINTER) (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_tag_motor_ref_local_get" SNS-MSG-TAG-MOTOR-REF-LOCAL-GET)
      ACH::ACH-STATUS-T (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER)
      (TIMESPEC :POINTER) (OPTIONS :INT))
  (DEFWRAPPER ("sns_msg_motor_state_size_n" SNS-MSG-MOTOR-STATE-SIZE-N) UINT32-T
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_state_size" SNS-MSG-MOTOR-STATE-SIZE) UINT32-T
    (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_motor_state_check_size" SNS-MSG-MOTOR-STATE-CHECK-SIZE)
      SSIZE-T (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_motor_state_init" SNS-MSG-MOTOR-STATE-INIT) :VOID
    (MSG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_state_heap_alloc" SNS-MSG-MOTOR-STATE-HEAP-ALLOC)
      :POINTER (N UINT32-T))
  (DEFWRAPPER
      ("sns_msg_motor_state_region_alloc" SNS-MSG-MOTOR-STATE-REGION-ALLOC) :POINTER
    (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_state_local_alloc" SNS-MSG-MOTOR-STATE-LOCAL-ALLOC)
      :POINTER (N UINT32-T))
  (DEFWRAPPER ("sns_msg_motor_state_put" SNS-MSG-MOTOR-STATE-PUT)
      ACH::ACH-STATUS-T (CHAN :POINTER) (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_motor_state_local_get" SNS-MSG-MOTOR-STATE-LOCAL-GET)
      ACH::ACH-STATUS-T (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER)
      (TIMESPEC :POINTER) (OPTIONS :INT))
  (DEFWRAPPER ("sns_msg_joystick_size_n" SNS-MSG-JOYSTICK-SIZE-N) UINT32-T
    (N UINT32-T))
  (DEFWRAPPER ("sns_msg_joystick_size" SNS-MSG-JOYSTICK-SIZE) UINT32-T
    (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_joystick_check_size" SNS-MSG-JOYSTICK-CHECK-SIZE) SSIZE-T
    (MSG :POINTER) (MEM_SIZE SIZE-T))
  (DEFWRAPPER ("sns_msg_joystick_init" SNS-MSG-JOYSTICK-INIT) :VOID
    (MSG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_joystick_heap_alloc" SNS-MSG-JOYSTICK-HEAP-ALLOC)
      :POINTER (N UINT32-T))
  (DEFWRAPPER ("sns_msg_joystick_region_alloc" SNS-MSG-JOYSTICK-REGION-ALLOC)
      :POINTER (REG :POINTER) (N UINT32-T))
  (DEFWRAPPER ("sns_msg_joystick_local_alloc" SNS-MSG-JOYSTICK-LOCAL-ALLOC)
      :POINTER (N UINT32-T))
  (DEFWRAPPER ("sns_msg_joystick_put" SNS-MSG-JOYSTICK-PUT) ACH::ACH-STATUS-T
    (CHAN :POINTER) (MSG :POINTER))
  (DEFWRAPPER ("sns_msg_joystick_local_get" SNS-MSG-JOYSTICK-LOCAL-GET)
      ACH::ACH-STATUS-T (CHAN :POINTER) (PMSG :POINTER) (FRAME-SIZE :POINTER)
      (TIMESPEC :POINTER) (OPTIONS :INT)))
