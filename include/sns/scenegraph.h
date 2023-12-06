/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SNS_SCENEGRPH_H
#define SNS_SCENEGRPH_H

#include <amino/rx/scenegraph.h>
#include <ach.h>


#ifdef __cplusplus
extern "C" {
#endif

void sg_make_addition_and_send( struct aa_rx_sg *scenegraph,
                                struct ach_channel* chan,
                                char* name, size_t n_name,
                                aa_rx_frame_id parent,
                                aa_rx_frame_id copy,
                                double* q,
                                double* v,
                                struct aa_mem_region* reg );

enum ach_status sg_make_addition( struct sns_msg_sg_update* msg, struct aa_rx_sg *scenegraph );

void sg_make_reparent_and_send( struct aa_rx_sg *scenegraph,
                                struct ach_channel* chan,
                                aa_rx_frame_id frame,
                                aa_rx_frame_id parent,
                                double* q,
                                double* v,
                                struct aa_mem_region* reg );

enum ach_status sg_make_reparent(struct sns_msg_sg_update* msg, struct aa_rx_sg *scenegraph );

void sg_make_remove_and_send( struct aa_rx_sg *scenegraph,
                              struct ach_channel* chan,
                              aa_rx_frame_id frame,
                              struct aa_mem_region* reg );

enum ach_status sg_make_remove(struct sns_msg_sg_update* msg, struct aa_rx_sg* scenegraph );

#ifdef __cplusplus
}
#endif

#endif //SNS_SCENEGRPH_H
