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

    for (int i=0; i<10; i++)
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