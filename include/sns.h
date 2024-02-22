/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SNS_H
#define SNS_H

/**
 * @file  sns.h
 * @brief top-level include file for sns
 *
 * @author Neil T. Dantam
 */

/**
 * Max length for hostnames in SNS messages
 */
#define SNS_HOSTNAME_LEN 8

/**
 * Max length for daemon identifier in SNS messages
 */
#define SNS_IDENT_LEN 8

/**
 * Max length for daemon backtraces
 */
#define SNS_BACKTRACE_LEN 32

/**
 * Type to use for floating point values.
 */
typedef double sns_real_t;

// clang-format off
// ach requires Amino to tell it what certain types are
#include <amino-1.0/amino.h>
#include <ach.h>
// clang-format on
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <syslog.h>
#include <time.h>

#include "sns/daemon.h"
#include "sns/msg.h"
#include "sns/path.h"
#include "sns/sdh_tactile.h"
#include "sns/util.h"

#endif  // SNS_H
