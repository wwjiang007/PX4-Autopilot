/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/differential_drive_setpoint.h>

/**
 * @brief Differential Drive Kinematics class for computing the kinematics of a differential drive robot.
 *
 * This class provides functions to set the wheel base and radius, and to compute the inverse kinematics
 * given linear velocity and yaw rate.
 */
class AckermannDriveKinematics : public ModuleParams
{
public:
	AckermannDriveKinematics(ModuleParams *parent);
	~AckermannDriveKinematics() = default;

	void allocate();

private:
	uORB::Subscription _differential_drive_control_output_sub{ORB_ID(differential_drive_control_output)};

	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

	differential_drive_setpoint_s _differential_drive_control_output{};

	float _max_speed{0.f};
	float _max_angular_velocity{0.f};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};
