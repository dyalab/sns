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

#include "sns.h"
#include "sns/scenegraph.h"

#include <amino/rx/rxerr.h>
#include <amino/rx/scenegraph.h>


void sg_make_addition_and_send(struct aa_rx_sg* scenegraph,
                               struct ach_channel* chan, char* name,
                               size_t n_name, aa_rx_frame_id parent,
                               aa_rx_frame_id copy, double* q, double* v,
                               struct aa_mem_region* reg) {
  struct sns_msg_sg_update* msg;
  if (reg)
    msg = sns_msg_sg_update_region_alloc(reg, n_name);
  else
    msg = sns_msg_sg_update_heap_alloc(n_name);

  msg->type = SNS_ADD_FRAME;
  AA_MEM_CPY(msg->name, name, n_name);
  msg->parent = parent;
  msg->copy_frame = copy;

  double E1[7];

  if (q) {
    AA_MEM_CPY(E1, q, 4);
  } else {
    E1[0] = 0;
    E1[1] = 0;
    E1[2] = 0;
    E1[3] = 1;
  }

  if (v) {
    AA_MEM_CPY(&E1[4], v, 3);
  } else {
    E1[4] = 0;
    E1[5] = 0;
    E1[6] = 0;
  }

  AA_MEM_CPY(msg->q, E1, 4);
  AA_MEM_CPY(msg->v, &E1[4], 3);

  sg_make_addition(msg, scenegraph);

  struct timespec cur_time;
  clock_gettime(ACH_DEFAULT_CLOCK, &cur_time);
  sns_msg_set_time(&msg->header, &cur_time, 0);
  sns_msg_sg_update_put(chan, msg);
}

enum ach_status sg_make_addition(struct sns_msg_sg_update* msg,
                                 struct aa_rx_sg* scenegraph) {
  aa_rx_sg_add_frame_fixed(scenegraph,
                           aa_rx_sg_frame_name(scenegraph, msg->parent),
                           msg->name, msg->q, msg->v);
  aa_rx_sg_init(scenegraph);
  aa_rx_sg_copy_frame_geom(
      scenegraph, aa_rx_sg_frame_name(scenegraph, msg->copy_frame), msg->name);
  aa_rx_sg_init(scenegraph);
  return ACH_OK;
}

void sg_make_reparent_and_send(struct aa_rx_sg* scenegraph,
                               struct ach_channel* chan, aa_rx_frame_id frame,
                               aa_rx_frame_id parent, double* q, double* v,
                               struct aa_mem_region* reg) {
  struct sns_msg_sg_update* msg;
  if (reg)
    msg = sns_msg_sg_update_region_alloc(reg, 1);
  else
    msg = sns_msg_sg_update_heap_alloc(1);

  msg->type = SNS_REPARENT_FRAME;
  msg->frame = frame;
  msg->parent = parent;

  double E1[7];

  if (q) {
    AA_MEM_CPY(E1, q, 4);
  } else {
    E1[0] = 0;
    E1[1] = 0;
    E1[2] = 0;
    E1[3] = 1;
  }

  if (v) {
    AA_MEM_CPY(&E1[4], v, 3);
  } else {
    E1[4] = 0;
    E1[5] = 0;
    E1[6] = 0;
  }

  AA_MEM_CPY(msg->q, E1, 4);
  AA_MEM_CPY(msg->v, &E1[4], 3);

  sg_make_reparent(msg, scenegraph);

  struct timespec cur_time;
  clock_gettime(ACH_DEFAULT_CLOCK, &cur_time);
  sns_msg_set_time(&msg->header, &cur_time, 0);
  sns_msg_sg_update_put(chan, msg);
}

enum ach_status sg_make_reparent(struct sns_msg_sg_update* msg,
                                 struct aa_rx_sg* scenegraph) {
  aa_rx_frame_id frame = msg->frame;
  aa_rx_frame_id new_parent = msg->parent;

  fprintf(stderr, "frame: %s (%u). Parent: %s (%u)\n",
          aa_rx_sg_frame_name(scenegraph, frame), frame,
          aa_rx_sg_frame_name(scenegraph, new_parent), new_parent);
  const double E1[7];
  AA_MEM_CPY(E1, msg->q, 4);
  AA_MEM_CPY(&E1[4], msg->v, 3);

  fprintf(stderr, "reparenting...");
  int r = aa_rx_sg_init(scenegraph);
  aa_rx_sg_reparent_id(scenegraph, new_parent, frame, E1);
  r = aa_rx_sg_init(scenegraph);
  if (r != AA_RX_OK) {
    SNS_LOG(LOG_ERR, "Got bad return code from init! %d", r)
  }
  return ACH_OK;
}

enum ach_status sg_make_remove(struct sns_msg_sg_update* msg,
                               struct aa_rx_sg* scenegraph) {
  aa_rx_sg_rm_frame(scenegraph, aa_rx_sg_frame_name(scenegraph, msg->frame));
  aa_rx_sg_init(scenegraph);
  return ACH_OK;
}

void sg_make_remove_and_send(struct aa_rx_sg* scenegraph,
                             struct ach_channel* chan, aa_rx_frame_id frame,
                             struct aa_mem_region* reg) {
  struct sns_msg_sg_update* msg;
  if (reg)
    msg = sns_msg_sg_update_region_alloc(reg, 1);
  else
    msg = sns_msg_sg_update_heap_alloc(1);

  msg->type = SNS_REMOVE_FRAME;
  msg->frame = frame;

  sg_make_remove(msg, scenegraph);

  struct timespec cur_time;
  clock_gettime(ACH_DEFAULT_CLOCK, &cur_time);
  sns_msg_set_time(&msg->header, &cur_time, 0);
  sns_msg_sg_update_put(chan, msg);
}
