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

#include "AckermannDriveKinematics.hpp"

using namespace time_literals;
using namespace matrix;

AckermannDriveKinematics::AckermannDriveKinematics(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
}

void AckermannDriveKinematics::allocate()
{
	if (_differential_drive_control_output_sub.updated()) {
		_differential_drive_control_output_sub.copy(&_differential_drive_control_output);
	}

	double steering = _differential_drive_control_output.yaw_rate;
	double speed = _differential_drive_control_output.speed;
	hrt_abstime now = hrt_absolute_time();

	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get(); // should be 3 see rc.rover_differential_defaults
	actuator_motors.control[0] = speed;
	actuator_motors.control[1] = speed;
	actuator_motors.timestamp = now;
	_actuator_motors_pub.publish(actuator_motors);

	// publish data to actuator_servos (output module)
	actuator_servos_s actuator_servos{};
	actuator_servos.control[0] = -steering;
	actuator_servos.control[1] = -steering;
	actuator_servos.timestamp = now;
	_actuator_servos_pub.publish(actuator_servos);
}
