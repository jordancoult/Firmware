/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file battery_internal_r.cpp
 * Module for periodically estimating battery's internal resistance
 *
 * @author Jordan Coult <coultjordanc@gmail.com>
 */

#include "BatteryInternalR.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

BatteryInternalR::BatteryInternalR() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

BatteryInternalR::~BatteryInternalR()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool BatteryInternalR::init()
{
	ScheduleOnInterval(1000_ms); // 1000 ms interval, 1 Hz rate

	return true;
}

void BatteryInternalR::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// GRAB BATTERY DATA
	_battery_status_sub.update();
	const battery_status_s &bstat = _battery_status_sub.get();
	
	// CALCULATE TOTAL RESISTANCE
	float a_delta = 0;
	if (last_voltage < 0) {  // first iteration
		last_voltage = abs(bstat.voltage_v);
		last_current = abs(bstat.current_a);
		return;
	}
	a_delta = abs(bstat.current_a - last_current);
	float new_r = -1;
	if (a_delta > max_delta) {
		max_delta = a_delta;
		new_r = abs(bstat.voltage_v - last_voltage) / a_delta;
	}
	last_voltage = bstat.voltage_v;
	last_current = bstat.current_a;

	// UPDATE PARAMETER
	if (new_r > 0) {
		int cells;
		param_get( param_find("BAT_N_CELLS"), &cells);
		if (cells > 0) {
			// calculate cell resistance and constrain
			new_r = new_r/cells;
			new_r = new_r > (float)0.2 ? (float)0.2 : new_r; // range: -1 to 0.2, step: 0.01, default: -1
			new_r = new_r < -1 ? -1 : new_r;
			void* new_r_ptr = &new_r;
			param_set( param_find("BAT1_R_INTERNAL"), new_r_ptr ); 
		}
	}

	perf_end(_loop_perf);
}

int BatteryInternalR::task_spawn(int argc, char *argv[])
{
	BatteryInternalR *instance = new BatteryInternalR();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int BatteryInternalR::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int BatteryInternalR::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BatteryInternalR::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module for periodically estimating battery's internal resistance.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_internal_r", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int battery_internal_r_main(int argc, char *argv[])
{
	return BatteryInternalR::main(argc, argv);
}