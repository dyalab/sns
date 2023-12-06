;;;; -*- Lisp -*-
;;;;
;;;; Copyright (c) 2014, Georgia Tech Research Corporation
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;; Georgia Tech Humanoid Robotics Lab
;;;; Under Direction of Prof. Mike Stilman
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

(progn
  (in-package :sns)

  (cc-flags "--std=gnu99")
  (cc-flags #.(concatenate 'string
                           "-I"
                           "/usr/local/include/amino-1.0"))
  (cc-flags "-lach -lsns")

  (include "stdlib.h")
  (include "syslog.h")
  (include "sns.h")
  (include "sns/motor.h")

  (ctype ach-status-t "ach_status_t")
  (ctype uint32-t "uint32_t")
  (ctype uint64-t "uint64_t")
  (ctype size-t "size_t")
  (ctype ssize-t "ssize_t")

  (cstruct msg-header "struct sns_msg_header"
           (sec "sec" :type :int64)
           (dur-nsec "dur_nsec" :type :int64)
           (nsec "nsec" :type :uint32)
           (n "n" :type :uint32)
           (from-pid "from_pid" :type :int64)
           (from-host "from_host" :type :char :count "SNS_HOSTNAME_LEN")
           (ident "ident" :type :char :count "SNS_IDENT_LEN")
           )

  (cstruct sns-msg-log "struct sns_msg_log"
           (header "header" :type (:struct msg-header))
           (priority "priority" :type :int)
           (text "text" :type :pointer))

  (cstruct sns-msg-text "struct sns_msg_text"
           (header "header" :type (:struct msg-header))
           (text  "text" :type :pointer))

  (cenum sns-sg-update
         ((:add "SNS_ADD_FRAME"))
         ((:remove "SNS_REMOVE_FRAME"))
         ((:reparent "SNS_REPARENT_FRAME")))

  (cstruct sns-msg-sg-update "struct sns_msg_sg_update"
           (header "header" :type (:struct msg-header))
           (type "type" :type sns-sg-update)
           (frame "frame" :type :int64)
           (parent "parent" :type :int64)
           (q "q" :type :pointer)
           (v "v" :type :pointer)
           (copy-frame "copy_frame" :type :int64)
           (name "name" :type :pointer))

  (cstruct sns-msg-vector "struct sns_msg_vector"
           (header "header" :type (:struct msg-header))
           (x "x" :type :pointer))

  (cstruct sns-msg-matrix "struct sns_msg_matrix"
           (header "header" :type (:struct msg-header))
           (rows "rows" :type uint64-t)
           (cols "cols" :type uint64-t)
           (x "x" :type :pointer))

  (cstruct sns-msg-tf "struct sns_msg_tf"
           (header "header" :type (:struct msg-header))
           (tf "tf" :type :pointer))

  (cstruct sns-msg-wt-tf "struct sns_msg_wt_tf"
           (header "header" :type (:struct msg-header))
           (wt-tf "wt_tf" :type :pointer))

  (cstruct sns-msg-tf-dx "struct sns_msg_tf_dx"
           (header "header" :type (:struct msg-header))
           (tf-dx "tf_dx" :type :pointer))

  (cenum motor-mode
         ((:halt "SNS_MOTOR_MODE_HALT"))
         ((:pos "SNS_MOTOR_MODE_POS"))
         ((:vel "SNS_MOTOR_MODE_VEL"))
         ((:torq "SNS_MOTOR_MODE_TORQ"))
         ((:cur "SNS_MOTOR_MODE_CUR"))
         ((:reset "SNS_MOTOR_MODE_RESET"))
         ((:pos-offset "SNS_MOTOR_MODE_POS_OFFSET")))

  (cstruct sns-msg-motor-ref "struct sns_msg_motor_ref"
           (header "header" :type (:struct msg-header))
           (mode "mode" :type motor-mode)
           (u "u" :type :pointer))

  (cstruct sns-msg-tag-motor-ref "struct sns_msg_tag_motor_ref"
          (header "header" :type (:struct msg-header))
          (mode "mode" :type motor-mode)
          (u "u" :type :pointer))

  (cstruct sns-msg-motor-state "struct sns_msg_motor_state"
           (header "header" :type (:struct msg-header))
           (mode "mode" :type motor-mode)
           (X "X" :type :pointer))

  (cstruct sns-msg-joystick "struct sns_msg_joystick"
           (header "header" :type (:struct msg-header))
           (buttons "buttons" :type uint64-t)
           (axis "axis" :type :pointer)))
