/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/action_request.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sent_come_home.h>

using namespace time_literals;

extern "C" __EXPORT int my_module_main(int argc, char *argv[]);


class MyModule : public ModuleBase<MyModule>, public ModuleParams
{
public:
        // static MyModule* instance;
	MyModule(int example_param, bool example_flag);

	virtual ~MyModule();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MyModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void test();

	float _heading_to_home;
	float _distance_to_home;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);
	void sendActionRequest(int8_t action, int8_t source, int8_t mode);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
        uORB::Publication<action_request_s> _action_request_pub{ORB_ID(action_request)};
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};

	private:
		int _params_sub{-1};
		int _vehicle_status_sub{-1};
		int _gps_sub{-1};
		int _home_sub{-1};
		int _local_pos_sub{-1};
		// int _command_pub{-1};
		// int _velocity_pub{-1};
		struct vehicle_status_s _vehicle_status{};
		struct vehicle_global_position_s _gps{};
		struct home_position_s _home{};
		struct vehicle_local_position_s _local_pos{};

		const float GPS_FAILURE_TIMEOUT = 30.0;
		const float ACCELERATION = 2.0;


		bool _armed{false};
		bool _home_valid{false};
		bool _gps_valid{false};
		bool _local_pos_valid{false};
		bool _rth_initiated{false};
		bool _params_updated{false};
		bool _vehicle_status_updated{false};
		bool _gps_updated{false};
		bool _home_updated{false};
		bool _local_pos_updated{false};

		matrix::Vector3f _home_direction_xy;
		float _home_distance{0.0f};
		hrt_abstime _last_gps_failure_time{0};

		void run1();
		void checkRTHCondition();
		bool check_rth_conditions();
		bool check_conditions();
		void calculateHomeDirectionAndDistance();
		void waitForDistance(double distance);
		void sendCommandToFlightBackToHome();
};

