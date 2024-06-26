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

// clang-format off
#include <sns.h>
// clang-format on

#include <ach/experimental.h>
#include <getopt.h>
#include <poll.h>

struct mplex_input {
    struct ach_channel channel;
    struct sns_msg_header *msg;
    const char *name;
    size_t max;
    size_t frame_size;
    _Bool updated;
};

struct cx {
    struct mplex_input *in;
    struct ach_channel out;
    size_t n;
};

enum ach_status
handle(void *cx, struct ach_channel *channel);
enum ach_status
periodic(void *cx);

int
main(int argc, char **argv)
{
    sns_init();
    struct aa_mem_rlist *names_list =
        aa_mem_rlist_alloc(aa_mem_region_local_get());
    struct cx cx = {0};

    const char *opt_chan_out = NULL;
    long opt_period_ns       = (long)1e6; /* 1 ms */

    /* Parse Options */
    {
        int c  = 0;
        opterr = 0;
        while ((c = getopt(argc, argv, "c:o:f:?V" SNS_OPTSTRING)) != -1) {
            switch (c) {
                SNS_OPTCASES
                case 'o':
                    opt_chan_out = optarg;
                    break;
                case 'f':
                    opt_period_ns = (long)(1e9 / atof(optarg));
                    break;
                case '?': /* help     */
                case 'h':
                    // clang-format off
                    puts(
                        "Usage: sns-mplex -o OUTPUT_CHANNEL INPUT_CHANNELS...\n"
                        "Multiplex SNS messages.  Reads messages from all input channels,\n"
                        "and relays only the highet priority, unexpired messages.\n"
                        "\n"
                        "Options:\n"
                        "  -o,                       message output channel\n"
                        "  -f,                       frequency (Hz)\n"
                        "  -V,                       Print program version\n"
                        "  -?,                       display this help and exit\n"
                        "\n"
                        "Examples:\n"
                        "  sns-mplex -chan -o out-chan input-0 input-1 input-2\n"
                        "\n"
                        "Report bugs to <ntd@rice.edu>");
                    // clang-format on
                    exit(EXIT_SUCCESS);
                default:
                    aa_mem_rlist_push_ptr(names_list, optarg);
                    cx.n++;

                    break;
            }
        }
        while (optind < argc) {
            aa_mem_rlist_push_ptr(names_list, argv[optind++]);
            cx.n++;
        }
    }

    SNS_REQUIRE(cx.n, "no input channels given\n");
    SNS_REQUIRE(opt_chan_out, "Need output channel");

    sns_chan_open(&cx.out, opt_chan_out, NULL);
    cx.in = AA_NEW_AR(struct mplex_input, cx.n);

    struct ach_evhandler *handlers = AA_NEW_AR(struct ach_evhandler, cx.n);

    // Initialize arrays
    for (size_t j = cx.n; j; j--) {
        size_t i = j - 1;

        cx.in[i].name = (const char *)aa_mem_rlist_pop(names_list);

        // open channel
        sns_chan_open(&cx.in[i].channel, cx.in[i].name, NULL);

        // init handler
        handlers[i].channel = &cx.in[i].channel;
        handlers[i].context = cx.in + i;
        handlers[i].handler = handle;

        SNS_LOG(LOG_DEBUG, "Initialized input channel %s\n", cx.in[i].name);
    }
    struct timespec period = {(time_t)(opt_period_ns / (time_t)1e9),
                              (opt_period_ns % (long)1e9)};

    SNS_LOG(LOG_DEBUG, "Period: %09lu.%08ld\n", period.tv_sec, period.tv_nsec);
    // Run Loop
    while (!sns_cx.shutdown) {
        errno = 0;

        enum ach_status r = ach_evhandle(handlers, cx.n, &period, periodic, &cx,
                                         ACH_EV_O_PERIODIC_INPUT);
        if (sns_cx.shutdown) break;
        SNS_REQUIRE(ACH_OK == r, "Could not handle events: %s, %s\n",
                    ach_result_to_string(r), strerror(errno));
    }

    return 0;
}

enum ach_status
handle(void *cx, struct ach_channel *channel)
{
    struct mplex_input *m = (struct mplex_input *)cx;
    SNS_LOG(LOG_DEBUG, "Event on channel %s\n", m->name);

    if (!m->msg) {
        // lazily allocate message
        void *msg;
        enum ach_status r =
            sns_msg_local_get(channel, &msg, &m->frame_size, NULL, ACH_O_LAST);
        if (ACH_STALE_FRAMES == r) return r;
        SNS_REQUIRE((ACH_OK == r || ACH_MISSED_FRAME == r),
                    "Could not get ach message on %s: %s\n", m->name,
                    ach_result_to_string(r));

        if (m->frame_size < sizeof(struct sns_msg_header)) {
            SNS_LOG(LOG_ERR, "Invalid message size on channel %s\n", m->name);
        } else {
            m->msg = (struct sns_msg_header *)malloc(m->frame_size);
            m->max = m->frame_size;
            memcpy(m->msg, msg, m->frame_size);
        }
        m->updated = 1;
        aa_mem_region_local_pop(msg);
    } else {
        enum ach_status r =
            ach_get(channel, m->msg, m->max, &m->frame_size, NULL, ACH_O_LAST);
        switch (r) {
            case ACH_OK:
            case ACH_MISSED_FRAME:
                // got it
                m->updated = 1;
                break;
            case ACH_STALE_FRAMES:
                return r;
            case ACH_OVERFLOW:
                // TODO: maybe handle this?
                SNS_LOG(LOG_ERR, "Message overflow on channel %s\n", m->name);
                break;
            default:
                SNS_DIE("Could not get message from channel %s: %s\n", m->name,
                        ach_result_to_string(r));
        }
    }

    return ACH_OK;
}

enum ach_status
periodic(void *_cx)
{
    SNS_LOG(LOG_DEBUG, "periodic handler\n");

    struct cx *cx = (struct cx *)_cx;

    struct timespec now;
    clock_gettime(ACH_DEFAULT_CLOCK, &now);
    for (size_t i = 0; i < cx->n; i++) {
        struct mplex_input *m = cx->in + i;
        if (m->msg && !sns_msg_is_expired(m->msg, &now)) {
            // found the active message
            if (m->updated) {
                SNS_LOG(LOG_DEBUG + 1, "Sending output from %s\n", m->name);
                enum ach_status r = ach_put(&cx->out, m->msg, m->frame_size);
                SNS_REQUIRE(ACH_OK == r, "Could not put output message: %s\n",
                            ach_result_to_string(r));
            }  // else nothing new to send
            break;
        }
    }

    /* Mark messages as un-updated */
    for (size_t i = 0; i < cx->n; i++) {
        cx->in[i].updated = 0;
    }

    return ACH_OK;
}

/* int do_poll( struct mplex_input *M, struct pollfd *pfd, size_t n ) */
/* { */
/*     // poll channels */
/*     int r_poll = poll(pfd, n, -1); */
/*     if( r_poll < 0 ) { */
/*         if( EINTR == errno ) return r_poll; */
/*         else SNS_DIE( "Poll failed: %s", strerror(errno) ); */
/*     } */

/*     if( r_poll > 0 ) { */
/*         // get the input */
/*         for( size_t i = 0;  i < n; i++ ) { */
/*             M[i].updated = 0; */
/*             if( pfd[i].revents & POLLIN ) { */
/*                 // data on channel i */
/*                 SNS_LOG(LOG_DEBUG+1, "Data on channel %s\n", M[i].name ); */
/*                 get_msg(M+i); */
/*             } */
/*         } */
/*     } */
/*     return r_poll; */
/* } */
