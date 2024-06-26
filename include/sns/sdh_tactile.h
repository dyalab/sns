/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * Copyright (c) 2023, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@mines.edu>
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

#ifndef SNS_SDH_TACTILE_H
#define SNS_SDH_TACTILE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sns.h>
#include <sns/msg.h>

#ifdef _cplusplus
extern "C" {
#endif

struct sns_msg_sdh_tactile {
    struct sns_msg_header header;
    float cog_x[6]; /**< CoG x coordinate in each of the 6 SDH pads */
    float cog_y[6]; /**< CoG y coordinate in each of the 6 SDH pads */
    float area[6];  /**< Contact area per pad */
    float force[6]; /**< Force applied in each pad */
    uint16_t x[1];  // Tactile info per each independent cell
};

SNS_DEF_MSG_VAR(sns_msg_sdh_tactile, x);
SNS_DEC_MSG_PLUGINS(sns_msg_sdh_tactile);

#ifdef _cplusplus
}
#endif

#endif  // SNS_SDH_TACTILE_H
