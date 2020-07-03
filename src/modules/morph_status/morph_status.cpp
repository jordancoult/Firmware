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
 * @file morph_status.cpp
 * Minimal test application for printing morph_status UORB data
 *
 * @author Jordan Coult <jordan@robodub.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/morph_status.h>

extern "C" __EXPORT int morph_status_main(int argc, char **argv);

int morph_status_main(int argc, char **argv)
{
    int morph_sub_fd = orb_subscribe(ORB_ID(morph_status));
    orb_set_interval(morph_sub_fd, 200); // limit the update rate to 200ms

    px4_pollfd_struct_t fds[1];
    fds[0].fd = morph_sub_fd, fds[0].events = POLLIN;

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
                struct morph_status_s ms;
                orb_copy(ORB_ID(morph_status), morph_sub_fd, &ms);
                PX4_INFO("Morph Mode: %d", ms.mode);
                PX4_INFO("Morph Angles: (%f,%f,%f,%f)", (double)ms.angles[0], (double)ms.angles[1], (double)ms.angles[2], (double)ms.angles[3]);
                PX4_INFO("Morph raw_cg: (%f,%f)", (double)ms.raw_cg[0], (double)ms.raw_cg[1]);
                PX4_INFO("Morph filt_cg: (%f,%f)", (double)ms.filt_cg[0], (double)ms.filt_cg[1]);
                PX4_INFO("Morph ct: (%f,%f)", (double)ms.ct[0], (double)ms.ct[1]);
                PX4_INFO("Morph shutter_open: %d \n", ms.shutter_open);  // bool is promoted to int when passed to printf
             }
        }
    }
    return 0;
}