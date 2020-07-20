/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file mixer.cpp
 *
 * Control channel input/output mixer and failsafe.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_config.h>
#include <syslog.h>

#include <sys/types.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

//#include <mixer/mixer.h>
#include <output_limit/output_limit.h>
#include <rc/sbus.h>

#include <uORB/topics/actuator_controls.h>

#include "mixer.h"

extern "C" {
	/* #define DEBUG */
#include "px4io.h"
}

/*
 * Maximum interval in us before FMU signal is considered lost
 */
#define FMU_INPUT_DROP_LIMIT_US		500000

/* current servo arm/disarm state */
static volatile bool mixer_servos_armed = false;
static volatile bool should_arm = false;
static volatile bool should_arm_nothrottle = false;
static volatile bool should_always_enable_pwm = false;
static volatile bool in_mixer = false;

extern int _sbus_fd;

/* selected control values and count for mixing */
enum mixer_source {
	MIX_NONE,
	MIX_DISARMED,
	MIX_FMU,
	MIX_OVERRIDE,
	MIX_FAILSAFE,
	MIX_OVERRIDE_FMU_OK
};

static volatile mixer_source source;

void
mixer_tick(void)
{
}


/*
 * XXX error handling here should be more aggressive; currently it is
 * possible to get STATUS_FLAGS_MIXER_OK set even though the mixer has
 * not loaded faithfully.
 */

//static char mixer_text[PX4IO_MAX_MIXER_LENGTH];		/* large enough for one mixer */
//static unsigned mixer_text_length = 0;
//static bool mixer_update_pending = false;

int
mixer_handle_text(const void *buffer, size_t length)
{

	return 0;
}

void
mixer_set_failsafe()
{
	
}
