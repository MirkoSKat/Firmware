/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ext_vehicles_app.c
 * App whichs collects the uORB messages from mavlink_receive.cpp and saves 
 * them to SD card.
 *
 * @author MirkoSKat <mirkoschimkat@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <math.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/ext_rssi_status.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int ext_rssi_app_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int ext_rssi_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
    if (reason) {
        warnx("%s\n", reason);
    }

    PX4_INFO("usage: ext_rssi_app {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int ext_rssi_app_main(int argc, char *argv[])
{

    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {
        FILE *sd;
        sd = fopen("/fs/microsd/RSSI.csv","a");
        fprintf(sd,"Start\n");
        fclose(sd);

        if (thread_running) {
            PX4_INFO("App already running\n");
            /* this is not an error */
            return 0;
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("daemon",
                            SCHED_DEFAULT,
                            SCHED_PRIORITY_DEFAULT,
                            2000,
                            ext_rssi_thread_main,
                            (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        FILE *sd;
        sd = fopen("/fs/microsd/RSSI.csv","a");
        fprintf(sd,"Stop\n");
        fclose(sd);

        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            PX4_INFO("\trunning\n");
        }
        else {
            PX4_INFO("\tnot started\n");
        }
            return 0;
    }

    usage("unrecognized command");
    return 1;
}

int ext_rssi_thread_main(int argc, char *argv[])
{
    PX4_INFO("starting\n");
    thread_running = true;

    /*--- Subscribe ---*/
    int ext_rssi_status_fd = orb_subscribe(ORB_ID(ext_rssi_status));

    px4_pollfd_struct_t fds[] = {
        { .fd = ext_rssi_status_fd, .events = POLLIN},
    };


    while (!thread_should_exit) {
        int poll_ret = px4_poll(fds, 1, 1000);

        if(poll_ret < 0){
            PX4_INFO("ERROR return value from poll(): %d", poll_ret);
            thread_should_exit = true;
        }
        else if((poll_ret > 0)&&(fds[0].revents & POLLIN)){
            FILE *sd;
            sd = fopen("/fs/microsd/RSSI.csv","a");
            if(sd == NULL){
                PX4_INFO("ERROR opening file on sd card");
                thread_should_exit = true;
            }
            else{
                PX4_INFO("Save data");
                struct ext_rssi_status_s raw;
                orb_copy(ORB_ID(ext_rssi_status), ext_rssi_status_fd, &raw);
                fprintf(sd,"%i;%i;%i;%i;%i;%i;%i;%i;%i; \n", (int)raw.timestamp, (int)raw.radio_id, (int)raw.rxerrors, (int)raw.fixed, (int)raw.rssi, (int)raw.remrssi, (int)raw.txbuf, (int)raw.noise, (int)raw.remnoise);
                fclose(sd);
            }
        }
//		sleep(10);
    }

    PX4_INFO("exiting.\n");

    thread_running = false;

    return 0;
}
