/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "AckermannDrive.hpp"

using namespace time_literals;
using namespace matrix;

AckermannDrive::AckermannDrive() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool AckermannDrive::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void AckermannDrive::updateParams()
{
	ModuleParams::updateParams();
	_max_speed = _param_rdd_max_wheel_speed.get() * _param_rdd_wheel_radius.get();
	_max_angular_velocity = _max_speed / (_param_rdd_wheel_base.get() / 2.f);
}

void AckermannDrive::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	hrt_abstime now = hrt_absolute_time();

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		if(_parameter_update_sub.copy(&pupdate)){
			updateParams();
		}
	}

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
			_manual_driving = vehicle_control_mode.flag_control_manual_enabled; // change this when more modes are supported
		}
	}

	if (_manual_driving) {
		// Manual mode
		// directly produce setpoints from the manual control setpoint (joystick)
		if (_manual_control_setpoint_sub.updated()) {
			differential_drive_setpoint_s _differential_drive_setpoint{};
			manual_control_setpoint_s manual_control_setpoint{};

			if (_manual_control_setpoint_sub.copy(&manual_control_setpoint)) {
				_differential_drive_setpoint.timestamp = now;
				_differential_drive_setpoint.speed = manual_control_setpoint.throttle * _param_rdd_speed_scale.get() * _max_speed;
				_differential_drive_setpoint.yaw_rate = manual_control_setpoint.roll * _param_rdd_ang_velocity_scale.get() *
									_max_angular_velocity;
				_differential_drive_setpoint_pub.publish(_differential_drive_setpoint);
			}
		}
	}

	_ackermann_drive_control.control();
	_ackermann_drive_kinematics.allocate();
}

int AckermannDrive::task_spawn(int argc, char *argv[])
{
	AckermannDrive *instance = new AckermannDrive();

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

int AckermannDrive::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AckermannDrive::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover state machine.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ackermann_drive", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ackermann_drive_main(int argc, char *argv[])
{
	return AckermannDrive::main(argc, argv);
}
