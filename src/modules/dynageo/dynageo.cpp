/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file dynageo.cpp
 * Minimal application for testing dynageo UORB data
 *
 * @author Jordan Coult <jordan@robodub.com>
 */

#include <px4_log.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");
    return OK;
}

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/dynageo.h>

extern "C" __EXPORT int dynageo_main(int argc, char **argv);

int dynageo_main(int argc, char **argv)
{
    int dynageo_sub_fd = orb_subscribe(ORB_ID(dynageo));
    orb_set_interval(dynageo_sub_fd, 200); // limit the update rate to 200ms

    px4_pollfd_struct_t fds[1];
    fds[0].fd = dynageo_sub_fd, fds[0].events = POLLIN;

    int error_counter = 0;

    for (int i=0; i<200; i++)
    {
        int poll_ret = px4_poll(fds, 1, 1000);

        if (poll_ret == 0)
        {
            PX4_ERR("Got no data within a second");
        }

        else if (poll_ret < 0)
        {
            if (error_counter < 10 || error_counter % 50 == 0)
            {
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;
        }

        else
        {
            if (fds[0].revents & POLLIN)
            {
                struct dynageo_s dg;
                orb_copy(ORB_ID(dynageo), dynageo_sub_fd, &dg);
                PX4_INFO("Morph Mode: %d", dg.failed_arm);
                PX4_INFO("Roll Scaling: (%f,%f,%f,%f)", (double)dg.roll[0], (double)dg.roll[1], (double)dg.roll[2], (double)dg.roll[3]);
                PX4_INFO("Roll Scaling: (%f,%f,%f,%f)", (double)dg.pitch[0], (double)dg.pitch[1], (double)dg.pitch[2], (double)dg.pitch[3]);
             }
        }
    }
    return 0;
}