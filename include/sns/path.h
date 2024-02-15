/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * Copyright (c) 2023, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/**
 * @file path.h
 * @brief Define struct to send path message
 * @date 2013/10/02 - last modified 2013/10/03
 */
#ifndef SNS_PATH_H
#define SNS_PATH_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sns/msg.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct sns_msg_path_dense
 * @brief Struct that stores a densely sampled path
 * @todo
 */
struct sns_msg_path_dense {
    /**
     * The message header
     */
    struct sns_msg_header header;
    uint32_t n_dof;    /**< Degrees of Freedom (e.g. 7 for LWA) */
    uint32_t n_steps;  /**< Number of points in the path */
    sns_real_t t0;     /**< Initial time */
    sns_real_t period; /**< Message period (interval between points) */
    sns_real_t x[1];   /**< count is n_dof*n_steps */
};

/**
 * @function sns_msg_path_dense_size_tn
 * @brief Calculates the size of any message with its n_steps and n_dof
 */
static inline size_t
sns_msg_path_dense_size_tn(size_t _n_steps, size_t _n_dofs)
{
    static const struct sns_msg_path_dense *msg;
    return sizeof(*msg) - sizeof(msg->x[0]) +
           sizeof(msg->x[0]) * _n_steps * _n_dofs;
}

/**
 * @function sns_msg_path_dense_size
 * @brief Returns the size of the message according to its n_steps and n_dof
 */
static inline size_t
sns_msg_path_dense_size(const struct sns_msg_path_dense *_msg)
{
    return sns_msg_path_dense_size_tn(_msg->n_steps, _msg->n_dof);
}

// DECLARATIONS
/**
 * Allocate message
 */
struct sns_msg_path_dense *
sns_msg_path_dense_alloc(uint32_t _n_steps, uint32_t _n_dof);
/**
 * print message
 */
void
sns_path_dense_dump(FILE *_out, const struct sns_msg_path_dense *_msg);

#ifdef __cplusplus
}
#endif
#endif  // SNS_PATH_H