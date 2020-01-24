/*
 * Copyright (c) 2019 Colorado School of Mines
 *
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

#include "config.h"

#include <getopt.h>

#include <amino.h>
#include <ach.h>

#include <amino/rx/scenegraph.h>

#include <amino/ct/state.h>
#include <amino/ct/traj.h>

#include "sns.h"
#include "sns/motor.h"
#include "sns/event.h"
#include <ach/experimental.h>

struct cx {
  struct aa_rx_sg *scenegraph;

  struct sns_motor_channel *arm_in;
  struct sns_motor_channel *arm_out;

  struct sns_motor_ref_set *ref_set;
  struct sns_motor_state_set *state_set;

  aa_rx_frame_id end_effector;

  struct sns_evhandler *handlers;

  double t;
  double duration;
  int64_t timestep;
  struct timespec cur_time;

  struct aa_ct_state *state_ref;
  struct aa_ct_pt_list *pt_list;
  struct aa_ct_seg_list *seg_list;

  struct aa_mem_region* reg;
  struct aa_ct_limit* limit;
};


static void halt( struct cx *cx );
static void reset_lists( struct cx *cx);
enum ach_status send_interp(void *cx_);
enum ach_status get_points(void *cx_, void* msg, size_t frame_size);

double opt_frequency = 10;
const double epsilon = 0.01;

int main(int argc, char **argv){
  struct cx cx = {0};
  char *opt_end_effector=NULL, *opt_input_channel=NULL;


  int c = 0;
  while( (c = getopt( argc, argv, "u:y:e:h?f:p:" SNS_OPTSTRING)) != -1 ) {
    switch(c) {
      SNS_OPTCASES_VERSION("sns-amino-controller",
                           "Copyright (c) 2018, Colorado School of Mines\n",
                           "Matthew A. Schack")
    case 'f':
      opt_frequency = atoi(optarg);
      break;
    case 'u':
      sns_motor_channel_push( optarg, &cx.arm_out );
      break;
    case 'y':
      sns_motor_channel_push( optarg, &cx.arm_in );
      break;
    case 'p':
      opt_input_channel = strdup(optarg);
      break;
    case 'e':
     opt_end_effector = optarg;
      break;
    case '?':   /* help     */
    case 'h':
      puts( "Usage: sns-amino-ctrl -u <from-arm> -y <to-arm>\n"
            "Teleop a robot.\n"
            "\n"
            "Options:\n"
            "  -y <channel>,             current state of the arm (state)\n"
            "  -u <channel>,             Motor velocities to move (ref)\n"
            "  -p <channel>,             input points to move to\n"
            "  -e <frame>,               end-effector frame\n"
            "  -V,                       Print program version\n"
            "  -?,                       display this help and exit\n"
            "Report bugs to " PACKAGE_BUGREPORT );
      exit(EXIT_SUCCESS);
    default:
      SNS_DIE("Unknown Option: `%c'\n", c);
      break;
    }
  }

  sns_init();
  SNS_REQUIRE(cx.arm_in, "Need input channel from arm");
  SNS_REQUIRE(cx.arm_out, "Need output channel to arm");
  SNS_REQUIRE(opt_input_channel, "Need input channel from point generator");

  /* Load Scene Plugin */
  cx.scenegraph = sns_scene_load();
  aa_rx_sg_init(cx.scenegraph);

  if( opt_end_effector ) {
    cx.end_effector = aa_rx_sg_frame_id(cx.scenegraph,opt_end_effector);
    SNS_REQUIRE( cx.end_effector > 0, "Invalid end-effector frame: `%s'", opt_end_effector );
  } else {
    cx.end_effector = AA_RX_FRAME_NONE;
  }

  /* Output channel to arm */
  sns_motor_ref_init(cx.scenegraph,
                     cx.arm_out, &cx.ref_set,
                     0, NULL );

  SNS_LOG(LOG_DEBUG,"Ref set size: %lu\n",cx.ref_set->n_q);


  /* Input channel from arm */
  size_t n_state = sns_motor_channel_count(cx.arm_in);
  size_t handler_count = n_state + 1;
  cx.handlers = AA_NEW_AR(struct sns_evhandler, handler_count);
  sns_motor_state_init(cx.scenegraph,
                       cx.arm_in, &cx.state_set,
                       n_state,cx.handlers);


  /* Input channel from point generator */
  struct ach_channel pt_chan;
  sns_chan_open(&pt_chan, opt_input_channel, NULL);

  cx.handlers[n_state].channel = &pt_chan;
  cx.handlers[n_state].context = &cx;
  cx.handlers[n_state].handler = get_points;
  cx.handlers[n_state].ach_options = ACH_O_FIRST;




  /* Initalize memory for amino states */
  cx.reg = aa_mem_region_create(32768); //arbitrarly large number

  /* Make limit states */
  size_t n_q = cx.ref_set->n_q;
  double min_q[n_q];
  double min_dq[n_q];
  double min_ddq[n_q];
  double max_q[n_q];
  double max_dq[n_q];
  double max_ddq[n_q];

  for(size_t i=0; i< n_q; i++){
    min_q[i] = -M_PI/2;
    min_dq[i] = -0.25;
    min_ddq[i] = -.1;
    max_q[i] = M_PI/2;
    max_dq[i] = 0.25;
    max_ddq[i] = .1;
  }

  struct aa_ct_state min;
  min.n_q = n_q;
  min.q = min_q;
  min.dq = min_dq;
  min.ddq = min_ddq;

  struct aa_ct_state max;
  max.n_q = n_q;
  max.q = max_q;
  max.dq = max_dq;
  max.ddq = max_ddq;

  struct aa_ct_limit limit;
  limit.min = &min;
  limit.max = &max;

  cx.limit = &limit;



  /* Prepare frequency calculations */
  cx.duration = (1 / opt_frequency);
  cx.timestep = (int64_t) (cx.duration *1e9);

  struct timespec sleep_dur;
  sleep_dur.tv_sec = cx.timestep / (int64_t) (2*1e9);
  sleep_dur.tv_nsec = cx.timestep % (int64_t) (2*1e9);

  struct timespec time;
  clock_gettime(ACH_DEFAULT_CLOCK, &time);
  cx.cur_time = time;
  cx.t=0;

  /* State allocation */
  cx.state_ref = aa_ct_state_alloc(cx.reg,n_q,n_q);


  /* Initalize ref set */
  for( size_t i = 0; i < cx.ref_set->n_q; i++ ) {
    cx.ref_set->u[i] = 0;
    cx.ref_set->meta[i].mode = SNS_MOTOR_MODE_VEL;
  }


  /* Loop through points and send the interpolation */
  enum ach_status r = sns_evhandle(cx.handlers,handler_count,
                                   &sleep_dur, send_interp, &cx,
                                   sns_sig_term_default,
                                   ACH_EV_O_PERIODIC_TIMEOUT);

  halt(&cx);
  SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could not handle event: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );

  sns_end();
  return 0;
}


enum ach_status send_interp(void *cx_){

  struct cx *cx = (struct cx*)cx_;

  /* Don't do anything if we don't know where to go */
  if (!cx->seg_list){
    return ACH_OK;
  }

  /* Update time */
  cx->t += cx->duration;
  SNS_LOG(LOG_DEBUG, "time: %f\n",cx->t);


  /* interpolate */
  int r = aa_ct_seg_list_eval(cx->seg_list,cx->state_ref,cx->t);
  if(sns_cx.shutdown){
    return ACH_CANCELED;
  }


  double kp = 5; //TODO: make this not a magic number
  int h = 1; //check to see if we should just send the halt command.

  for(size_t i=0; i < cx->ref_set->n_q; i++){
    double state_q = cx->state_set->state->q[i];
    double ref_q = cx->state_ref->q[i];

    /* Remap to [0,2*pi) */
    int offset = state_q / ( 2 * M_PI );
    state_q -= offset*(2*M_PI);
    cx->state_set->state->q[i] = state_q;

    offset = ref_q / (2*M_PI);
    ref_q -= offset*(2*M_PI);
    cx->state_ref->q[i] = ref_q;

    /* Calculate new motor velocity */
    double new_vel = cx->state_ref->dq[i]+kp*(ref_q - state_q);
    if(fabs(new_vel) < epsilon) new_vel = 0;
    else h = 0;
    cx->ref_set->u[i] = new_vel;

    SNS_LOG(LOG_DEBUG, "%lu reference q: %f. Actual q: %f. dq: %f\n", i, ref_q,
            state_q, new_vel);
  }


  /* Increment time */
  cx->cur_time = sns_time_add_ns(cx->cur_time,cx->timestep);

  /* Send message */
  if (!r && h) { // halt and deallocate memory if we've reached the end
    reset_lists(cx);
  } else  sns_motor_ref_put(cx->ref_set, &cx->cur_time, cx->timestep);
  return ACH_OK;
}

enum ach_status get_points(void *cx_, void *msg_, size_t frame_size){
  struct cx *cx = (struct cx*) cx_;
  struct sns_msg_vector *msg = (struct sns_msg_vector*) msg_;
  size_t n_q = cx->ref_set->n_q;


  if(msg->x[0] == 1.0 && cx->seg_list && cx->pt_list){
    reset_lists(cx);
  }

  if( !cx->pt_list){
    cx->pt_list = aa_ct_pt_list_create(cx->reg);
  }

  SNS_LOG(LOG_DEBUG, "Array pointer %p\n", msg->x);
  /* Add new point to the point list */
  aa_ct_pt_list_add_q(cx->pt_list,n_q,&msg->x[1]);


  /* Create seg list from new set of points */
  if( !cx->seg_list){
    cx->t = 0; //reset time
  }else{
    aa_ct_seg_list_destroy(cx->seg_list); //dealloc seg list
  }

  /* Only create a seg list if there are enough points to generate a segment */
  if(aa_ct_pt_list_start_state(cx->pt_list) != aa_ct_pt_list_final_state(cx->pt_list)){
      cx->seg_list = aa_ct_tjq_trap_generate(cx->reg,cx->pt_list,cx->limit);
      clock_gettime(ACH_DEFAULT_CLOCK, &cx->cur_time);
    }

  return ACH_OK;
}

static void reset_lists(struct cx *cx){
  halt(cx);
  aa_ct_seg_list_destroy(cx->seg_list);
  aa_ct_pt_list_destroy(cx->pt_list);
  cx->seg_list = NULL;
  cx->pt_list = NULL;
}

static void
halt( struct cx *cx ){
    for( size_t i = 0; i < cx->ref_set->n_q; i++ ) {
        cx->ref_set->meta[i].mode = SNS_MOTOR_MODE_HALT;
        cx->ref_set->u[i]= 0;
    }

    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    sns_motor_ref_put( cx->ref_set, &now, 1e9 );

    for( size_t i = 0; i < cx->ref_set->n_q; i++ ) {
        cx->ref_set->meta[i].mode = SNS_MOTOR_MODE_VEL;
        cx->ref_set->u[i]= 0;
    }
}
