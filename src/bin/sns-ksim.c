/*
 * Copyright (c) 2017, Rice University.
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

#include <poll.h>


#include <getopt.h>

#include <cblas.h>

#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>
#include <amino/ct/state.h>

#include <amino/rx/scene_gl.h>
#include <amino/rx/scene_win.h>

#include "sns.h"
#include <sns/event.h>
#include <ach/experimental.h>
#include <sns/motor.h>

struct cx {
  struct aa_rx_sg *scenegraph;

  struct sns_motor_channel *ref_in;
  struct sns_motor_channel *state_out;

  struct sns_motor_ref_set *ref_set;
  struct sns_motor_state_set *state_set;

  struct timespec t;

  size_t n_q;
  uint64_t seq;

  struct aa_rx_win * win;

  struct sns_evhandler *handlers;
  size_t n_handlers;
  struct timespec period;

};


/* Run io */
void io(struct cx *cx);
/* Pthreads start function for io */
void* io_start(void *cx);

/* Call periodically from io thread */
enum ach_status io_periodic( void *cx );

/* Perform a simulation step */
enum ach_status simulate( struct cx *cx );

/* Generically apply some change to the scenegraph */
enum ach_status handle_change(void *cx_, void *msg_, size_t frame_size);

/* Reparent frames */
enum ach_status handle_reparent(struct cx *cx, struct sns_msg_sg_update* msg, size_t size);

/* Add new frame */
enum ach_status handle_addition(struct cx *cx, struct sns_msg_sg_update* msg, size_t size);

/* Remove frame */
enum ach_status handle_removal(struct cx *cx, struct sns_msg_sg_update* msg, size_t size);

int main(int argc, char **argv)
{
    struct cx cx;
    AA_MEM_ZERO(&cx,1);

    const double opt_sim_frequecy = 100;
    struct sns_motor_channel *last_mc = NULL;
    char *opt_change_chan = NULL;
    struct ach_channel change;

    /* Parse Options */
    {
        int c = 0;
        opterr = 0;
        while( (c = getopt( argc, argv, "y:u:p:h?c:" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-ksim",
                                     "Copyright (c) 2017, Rice University\n",
                                     "Neil T. Dantam")
            case 'y':
                sns_motor_channel_push( optarg, &cx.state_out );
                last_mc = cx.state_out;
                break;
            case 'u':
                sns_motor_channel_push( optarg, &cx.ref_in );
                last_mc = cx.ref_in;
                break;
            case 'p':
                if( last_mc ) {
                    last_mc->priority = atoi(optarg);
                } else {
                    SNS_DIE("No channel specified for priority argument");
                }
                break;
            case 'c':
              opt_change_chan = strdup(optarg);
              sns_chan_open(&change, opt_change_chan, NULL);
              break;
            case '?':   /* help     */
            case 'h':
                puts( "Usage: sns-ksim -u REF_CHANNEL -y STATE_CHANNEL\n"
                      "Kinematically simulate a robot.\n"
                      "\n"
                      "Options:\n"
                      "  -y <channel>,             state output channel\n"
                      "  -u <channel>,             reference input channel\n"
                      "  -p <priority>,            channel priority\n"
                      "  -V,                       Print program version\n"
                      "  -?,                       display this help and exit\n"
                      "\n"
                      "Environment:\n"
                      "  SNS_SCENE_PLUGIN          Shared object (plugin) defining the scene\n"
                      "\n"
                      "  SNS_SCENE_NAME            Name of the scene within the plugin\n"
                      "\n"
                      "  SNS_CHANNEL_MAP_name      Channel remap list for `name'\n"
                      "\n"
                      "Examples:\n"
                      "  sns-ksim -y state -u ref\n"
                      "\n"
                      "Report bugs to <ntd@rice.edu>"
                    );
                exit(EXIT_SUCCESS);
            default:
                SNS_DIE("Unknown Option: `%c'\n", c);
                break;
            }
        }
    }
    sns_init();

    /* Scene Plugin */
    SNS_LOG(LOG_DEBUG, "before scene load\n");
    cx.scenegraph = sns_scene_load();

    aa_rx_sg_init(cx.scenegraph);
     SNS_LOG(LOG_DEBUG, "before config count\n");
    cx.n_q = aa_rx_sg_config_count(cx.scenegraph);
    SNS_LOG(LOG_DEBUG, "before motor stat init\n");
    /* State */
    sns_motor_state_init(cx.scenegraph,
                         cx.state_out, &cx.state_set,
                         0, NULL);
    clock_gettime(ACH_DEFAULT_CLOCK, &cx.t);

    /* Reference */
    SNS_REQUIRE( cx.ref_in, "Need reference channel");
    {
        size_t n_ref = sns_motor_channel_count(cx.ref_in);

        if( opt_change_chan ){
          cx.handlers = AA_NEW_AR( struct sns_evhandler, n_ref + 1);
          cx.handlers[n_ref].channel = &change;
          cx.handlers[n_ref].context = &cx;
          cx.handlers[n_ref].handler = handle_change;
          cx.handlers[n_ref].ach_options = ACH_O_FIRST;
          cx.n_handlers = n_ref + 1;
        }else{
          cx.handlers = AA_NEW_AR( struct sns_evhandler, n_ref);
          cx.n_handlers = n_ref;
        }
        sns_motor_ref_init(cx.scenegraph,
                             cx.ref_in, &cx.ref_set,
                             n_ref, cx.handlers);
    }

    SNS_LOG(LOG_INFO, "Simulation Frequency: %.3fkHz\n", opt_sim_frequecy/1e3);
    long period_ns = (long)(1e9 / opt_sim_frequecy);

    cx.period.tv_sec = (time_t)period_ns / (time_t)1e9;
    cx.period.tv_nsec = period_ns % (long)1e9;

    SNS_LOG( LOG_DEBUG, "Simulation Period: %lus + %ldns\n",
             cx.period.tv_sec, cx.period.tv_nsec );


    /* Start threads */
    pthread_t io_thread;
    if( pthread_create(&io_thread, NULL, io_start, &cx) ) {
        SNS_DIE("Could not create simulation thread: `%s'", strerror(errno));
    }

    /* Start GUI in main thread */
    cx.win = aa_rx_win_default_create ( "sns-ksim", 800, 600 );
    aa_rx_win_set_sg(cx.win, cx.scenegraph);
    sns_start();
    aa_rx_win_run();

    /* Stop threads */
    sns_cx.shutdown = 1;
    if( pthread_join(io_thread, NULL) ) {
        SNS_LOG(LOG_ERR, "Could not join simulation thread: `%s'", strerror(errno));
    }
    sns_end();



    return 0;
}

void parse_string(char* dst, char* src, size_t* len){
  size_t i=0;
  while( ' ' != src[i] && '\0' != src[i]) i++;

  *len = i;

  strncpy(dst, src, i);
  dst[i] = '\0';
  SNS_LOG(LOG_DEBUG, "parsed string: %s\n", dst);
}

enum ach_status handle_change(void *cx_, void *msg_, size_t frame_size){
  struct cx *cx = (struct cx*) cx_;
  struct sns_msg_sg_update *msg = (struct sns_msg_sg_update*) msg_;

  if(SNS_LOG_PRIORITY(LOG_INFO)) sns_msg_sg_update_dump(stdout, msg);


  enum sns_sg_update type = msg->type;
  SNS_LOG(LOG_DEBUG, "action: %s\n", sns_sg_update_str(type));


  if(type == SNS_REPARENT_FRAME){
      return sg_make_reparent(msg, cx->scenegraph);
  }else if(type == SNS_ADD_FRAME){
      return sg_make_addition(msg, cx->scenegraph);
  }else if(type == SNS_REMOVE_FRAME){
      return sg_make_remove(msg, cx->scenegraph);
  }else{
      SNS_DIE("unknown change detected: %ld\n", type);
      return ACH_OK;
  }
}


enum ach_status handle_reparent(struct cx *cx, struct sns_msg_sg_update* msg, size_t size){
    aa_rx_frame_id frame = msg->frame;
    aa_rx_frame_id new_parent = msg->parent;

    SNS_LOG(LOG_DEBUG,"frame: %s. Parent: %s\n",
            aa_rx_sg_frame_name(cx->scenegraph, frame),
            aa_rx_sg_frame_name(cx->scenegraph, new_parent));
    const double E1[7];
    AA_MEM_CPY(E1, msg->q, 4);
    AA_MEM_CPY(&E1[4], msg->v, 3);

    aa_rx_sg_reparent_id(cx->scenegraph, new_parent, frame, E1);
    aa_rx_sg_init(cx->scenegraph);
    return ACH_OK;
}

enum ach_status handle_addition(struct cx *cx, struct sns_msg_sg_update* msg, size_t size)
{
    aa_rx_sg_add_frame_fixed(cx->scenegraph,
                             aa_rx_sg_frame_name(cx->scenegraph, msg->parent),
                             msg->name, msg->q, msg->v);
    aa_rx_sg_init(cx->scenegraph);
    aa_rx_sg_copy_frame_geom(cx->scenegraph,
                             aa_rx_sg_frame_name(cx->scenegraph, msg->copy_frame),
                             msg->name);
    aa_rx_sg_init(cx->scenegraph);
    return ACH_OK;
}

 enum ach_status handle_removal(struct cx *cx, struct sns_msg_sg_update* msg, size_t size){
     aa_rx_sg_rm_frame(cx->scenegraph, aa_rx_sg_frame_name(cx->scenegraph, msg->frame));
     aa_rx_sg_init(cx->scenegraph);
     return ACH_OK;
 }


void* io_start(void *cx) {
    io((struct cx*)cx);
    return NULL;
}

void io(struct cx *cx) {
    /* Run Loop */
    enum ach_status r = sns_evhandle( cx->handlers, cx->n_handlers,
                                      &cx->period, io_periodic, cx,
                                      sns_sig_term_default,
                                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could asdf not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );

    /* stop window */
    if( cx->win ) {
        aa_rx_win_stop(cx->win);
    }
}

enum ach_status io_periodic( void *cx_ )
{
    struct cx *cx = (struct cx*)cx_;

    /* Step simulation */
    simulate(cx);

    /* Post state */
    struct aa_ct_state *state = sns_motor_state_get(cx->state_set);
    sns_motor_state_put( cx->state_set, &cx->t, (int64_t)1e9 );

    /* Update display */
    if( cx->win ) aa_rx_win_set_config( cx->win, state->n_q, state->q );

    /* check cancelation */
    if( sns_cx.shutdown ) {
        return ACH_CANCELED;
    } else {
        return ACH_OK;
    }
}


enum ach_status simulate( struct cx *cx )
{
    struct timespec now;
    clock_gettime(ACH_DEFAULT_CLOCK, &now);
    double dt = aa_tm_timespec2sec( aa_tm_sub(now, cx->t) );
    cx->t = now;

    struct aa_ct_state *state = sns_motor_state_get(cx->state_set);

    /* Collect references */
    sns_motor_ref_collate(&cx->t, cx->ref_set);

    /* Process References */
    assert(cx->n_q == cx->ref_set->n_q );
    for( size_t i = 0; i < cx->ref_set->n_q; i ++ ) {
        struct sns_motor_ref_meta *m = cx->ref_set->meta+i;
        double u = cx->ref_set->u[i];
        double *q = state->q+i;
        double *dq = state->dq+i;
        if( aa_tm_cmp(now,m->expiration) < 0 ) {
            switch(m->mode) {
            case SNS_MOTOR_MODE_POS:
                *q = u;
                *dq = 0;
                break;
            case SNS_MOTOR_MODE_VEL:
                *dq = u;
                (*q) += dt * (*dq);
                break;
            case SNS_MOTOR_MODE_HALT:
                *dq = 0;
                break;
            default:
                SNS_LOG(LOG_WARNING, "Unhandled mode for motor %lu", i );
            }
        } else {
            /* reference has expired */
          *dq = 0;
        }
    }

    return ACH_OK;
}
