/*
 * Copyright (c) 2015, Rice University.
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


#include <poll.h>
#include <sns.h>
#include <getopt.h>

struct ach_channel g_channel_out;

struct mplex_input {
    struct ach_channel channel;
    struct sns_msg_header *msg;
    const char *name;
    size_t max;
    size_t frame_size;
    _Bool updated;
} ;

int do_poll( struct mplex_input *M, struct pollfd *pfd, size_t n );
void get_msg( struct mplex_input *m );
void do_output( struct mplex_input *M, size_t n, struct ach_channel *out  );

static const char *opt_chan_out = NULL;

int main(int argc, char **argv)
{
    sns_init();
    struct aa_mem_rlist *names_list = aa_mem_rlist_alloc( aa_mem_region_local_get() );
    size_t g_n_channels = 0;
    /* Parse Options */
    {
        int c = 0;
        opterr = 0;
        while( (c = getopt( argc, argv, "c:o:?V" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES
            case 'o':
                opt_chan_out = optarg;
                break;
            case '?':   /* help     */
            case 'h':
                puts( "Usage: sns-mplex -o OUTPUT_CHANNEL INPUT_CHANNELS...\n"
                      "Multiplex SNS messages.  Reads messages from all input channels,\n"
                      "and relays only the highet priority, unexpired messages.\n"
                      "\n"
                      "Options:\n"
                      "  -o,                       message output channel\n"
                      "  -V,                       Print program version\n"
                      "  -?,                       display this help and exit\n"
                      "\n"
                      "Examples:\n"
                      "  sns-mplex -chan -o out-chan input-0 input-1 input-2\n"
                      "\n"
                      "Report bugs to <ntd@rice.edu>"
                    );
                exit(EXIT_SUCCESS);
            default:
                aa_mem_rlist_push_ptr( names_list, optarg );
                g_n_channels++;

                break;
            }
        }
        while( optind < argc ) {
            aa_mem_rlist_push_ptr( names_list, argv[optind++] );
            g_n_channels++;
        }
    }


    SNS_REQUIRE(g_n_channels, "no input channels given\n");
    SNS_REQUIRE( opt_chan_out, "Need output channel");
    sns_chan_open( &g_channel_out, opt_chan_out , NULL );

    struct mplex_input *g_mplex_input = AA_NEW_AR(struct mplex_input, g_n_channels);
    struct pollfd *g_pollfd = AA_NEW_AR(struct pollfd, g_n_channels);

    // Initialize arrays
    for( size_t j = g_n_channels; j; j--) {
        size_t i = j-1;
        struct mplex_input *m = g_mplex_input + i;

        m->name = (const char*)aa_mem_rlist_pop(names_list);

        // open channel
        sns_chan_open( &m->channel, m->name, NULL );

        enum ach_status r;

        // check mapping
        enum ach_map map;
        r = ach_channel_mapping(&m->channel, &map );
        SNS_REQUIRE( ACH_OK == r, "Couldn't get channel mapping: %s", ach_result_to_string(r) );
        SNS_REQUIRE( ACH_MAP_KERNEL == map, "Not a kernel channel: %s", m->name );

        // init poll struct
        r = ach_channel_fd(&m->channel, &g_pollfd[i].fd);
        SNS_REQUIRE(ACH_OK==r, "Couldn't get channel fd");
        g_pollfd[i].events=POLLIN;

        SNS_LOG(LOG_DEBUG, "Initialized input channel %s\n", m->name );
    }

    // Run Loop
    while( !sns_cx.shutdown) {
        if( 0 < do_poll(g_mplex_input, g_pollfd, g_n_channels) ) {
            do_output(g_mplex_input, g_n_channels, &g_channel_out);
        }
    }

    return 0;
}

// TODO: wrap up poll() in an event-handling library call
int do_poll( struct mplex_input *M, struct pollfd *pfd, size_t n )
{
    // poll channels
    int r_poll = poll(pfd, n, -1);
    if( r_poll < 0 ) {
        if( EINTR == errno ) return r_poll;
        else SNS_DIE( "Poll failed: %s", strerror(errno) );
    }

    if( r_poll > 0 ) {
        // get the input
        for( size_t i = 0;  i < n; i++ ) {
            M[i].updated = 0;
            if( pfd[i].revents & POLLIN ) {
                // data on channel i
                SNS_LOG(LOG_DEBUG+1, "Data on channel %s\n", M[i].name );
                get_msg(M+i);
            }
        }
    }
    return r_poll;
}

void get_msg( struct mplex_input *m )
{
    if( ! m->msg ) {
        // lazily allocate message
        void *msg;
        enum ach_status r = sns_msg_local_get( &m->channel, &msg,
                                               &m->frame_size, NULL, ACH_O_LAST );
        SNS_REQUIRE( (ACH_OK == r || ACH_MISSED_FRAME == r),
                     "Could not get ach message on %s: %s\n",
                     m->name, ach_result_to_string(r) );

        if( m->frame_size < sizeof(struct sns_msg_header) ) {
            SNS_LOG(LOG_ERR, "Invalid message size on channel %s\n", m->name);
        } else {
            m->msg = (struct sns_msg_header*)malloc(m->frame_size);
            m->max = m->frame_size;
            memcpy(m->msg, msg, m->frame_size);
        }
        m->updated = 1;
        aa_mem_region_local_pop( msg );
    } else {
        enum ach_status r = ach_get( &m->channel, m->msg, m->max,
                                     &m->frame_size, NULL, ACH_O_LAST );
        switch(r) {
        case ACH_OK:
        case ACH_MISSED_FRAME:
            // got it
            m->updated = 1;
            break;
        case ACH_OVERFLOW:
            // TODO: maybe handle this?
            SNS_LOG(LOG_ERR, "Message overflow on channel %s\n", m->name);
            break;
        default:
            SNS_DIE("Could not get message from channel %s: %s\n",
                    m->name, ach_result_to_string(r) );
        }
    }
}

void do_output( struct mplex_input *M, size_t n, struct ach_channel *out  )
{
    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    for( size_t i = 0; i < n; i++ ) {
        struct mplex_input *m = M + i;
        if( m->msg && ! sns_msg_is_expired(m->msg, &now) ) {
            // found the active message
            if( m->updated ) {
                SNS_LOG(LOG_DEBUG+1, "Sending output from %s\n", m->name );
                enum ach_status r = ach_put( out, m->msg, m->frame_size );
                SNS_REQUIRE( ACH_OK == r, "Could not put output message: %s\n",
                             ach_result_to_string(r) );
            } // else nothing new to send
            break;
        }
    }
}