/*
 * Copyright (c) 2019, Colorado School of Mines.
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

#include "sns.h"
#include <sns/event.h>
#include <ach/experimental.h>
#include <sns/msg.h>

#include <amino/rx/scenegraph.h>

struct cx {
    char * filePath;
};


/* write an abbreviated message to the channel */
enum ach_status write_motor_state(void *cx_, void *msg_, size_t frame_size);
enum ach_status write_sg_update(void *cx_, void *msg_, size_t frame_size);
enum ach_status write_text(void *cx_, void *msg_, size_t frame_size);

int main(int argc, char **argv)
{
    /* Parse Options */
    int c = 0;
    struct ach_channel chan;
    struct cx cx ={0};
    while( (c = getopt( argc, argv, SNS_OPTSTRING)) != -1 ) {
        switch(c) {
            SNS_OPTCASES_VERSION("msg-simple",
                                 "Copyright (c) 2019, Colorado School of Mines\n",
                                 "Matthew A. Schack")
        case '?':   /* help     */
        case 'h':
            puts( "Usage: msg-simple channel type filePath\n"
                  "Prints simple messages to a file. Useful for quick integration tests\n"
                  "\n"
                  "Options:\n"
                  "  -V,                       Print program version\n"
                  "  -?,                       display this help and exit\n"
                  "Report bugs to <mschack@mines.edu>"
                );
            exit(EXIT_SUCCESS);
        default:
            SNS_DIE("Unknown Option: `%c'\n", c);
            break;
        }
    }
    if(argc != 4){
        SNS_DIE("3 arguments are required. See msg-simple -h\n");
    }

    struct sns_evhandler handlers[1];
    sns_chan_open(&chan, argv[1], NULL);
    handlers[0].channel = &chan;
    handlers[0].context = &cx;
    if(strcmp(argv[2], "motor_state") == 0)
        handlers[0].handler = write_motor_state;
    else if(strcmp(argv[2], "sg_update") == 0)
        handlers[0].handler = write_sg_update;
    else if(strcmp(argv[2], "text") == 0)
        handlers[0].handler = write_text;
    else
        SNS_DIE("Unknown channel type: %s\n", argv[2]);

    handlers[0].ach_options = ACH_O_FIRST;
    cx.filePath = strdup(argv[3]);
    fprintf(stdout, "File path:%s\n", cx.filePath);

    sns_init();

    enum ach_status r = sns_evhandle( handlers, 1,
                                      NULL, NULL, NULL,
                                      sns_sig_term_default,
                                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could asdf not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );
    return 0;
}


enum ach_status write_motor_state(void *cx_, void *msg_, size_t frame_size)
{
    struct cx *cx = (struct cx*) cx_;
    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state*) msg_;
    FILE *f;
    f = fopen(cx->filePath, "w+");
    for(uint32_t i=0; i < msg->header.n; i++){
        fprintf(f, "%f ", msg->X[i].pos);
    }
    fprintf(f, "\n");
    fclose(f);
    return ACH_OK;
}

enum ach_status write_sg_update(void *cx_, void *msg_, size_t frame_size)
{
    struct cx *cx = (struct cx*) cx_;
    struct sns_msg_sg_update *msg = (struct sns_msg_sg_update*) msg_;
    FILE *f;
    f = fopen(cx->filePath, "a+");
    if(msg->type == SNS_ADD_FRAME){
        fprintf(f, "ADD ");
        for(uint32_t i=0; i < msg->header.n; i++){
            fprintf(f, "%c", msg->name[i]);
        }
        fprintf(f, " %ld ", msg->parent);
        fprintf(f, "%ld\n", msg->copy_frame);
    }else if(msg->type == SNS_REMOVE_FRAME){
        fprintf(f, "REMOVE ");
        for(uint32_t i=0; i < msg->header.n; i++){
            fprintf(f, "%c", msg->name[i]);
        }
        fprintf(f, "\n");
    }else if(msg->type == SNS_REPARENT_FRAME){
        fprintf(f, "REPARENT ");
        for(uint32_t i=0; i < msg->header.n; i++){
            fprintf(f, "%c", msg->name[i]);
        }
        fprintf(f, " %ld (", msg->parent);
        for(size_t i=0; i < 4; i++){
            fprintf(f, "%f ", msg->q[i]);
        }
        fprintf(f, ") (");
        for(size_t i=0; i < 3; i++){
            fprintf(f, "%f ", msg->v[i]);
        }
        fprintf(f, ")\n");
    }
    fclose(f);
    return ACH_OK;
}

enum ach_status write_text(void *cx_, void *msg_, size_t frame_size)
{
    struct cx *cx = (struct cx*) cx_;
    struct sns_msg_text *msg = (struct sns_msg_text*) msg_;
    FILE *f;
    f = fopen(cx->filePath, "w+");
    for(uint32_t i=0; i< msg->header.n; i++){
        fprintf(f, "%c", msg->text[i]);
    }
    fprintf(f, "\n");
    fclose(f);
    return ACH_OK;
}
