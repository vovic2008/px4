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

#include "my_module.h"


#include <cmath>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include "../commander/Commander.hpp"
#include "../commander/px4_custom_mode.h"



static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}
int MyModule::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int MyModule::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */


	get_instance()->test();
	return print_usage("done command");
}

void MyModule::test(){
	sendCommandToFlightBackToHome();
}


int MyModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MyModule *MyModule::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':{

			break;
		}
		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	MyModule *instance = new MyModule(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

MyModule::MyModule(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void MyModule::run()
{
	// run1();
	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int home_position_sub = orb_subscribe(ORB_ID(home_position));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = sensor_combined_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			struct home_position_s home_position;
			struct vehicle_status_s vehicle_status;
			struct vehicle_global_position_s vehicle_global_position;
			struct vehicle_local_position_s vehicle_local_position;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			orb_copy(ORB_ID(home_position), home_position_sub, &home_position);
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
			orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &vehicle_global_position);
			orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &vehicle_local_position);
			// TODO: do something with the data...

			PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)sensor_combined.accelerometer_m_s2[0],
					 (double)sensor_combined.accelerometer_m_s2[1],
					 (double)sensor_combined.accelerometer_m_s2[2]);


			PX4_INFO("home_position:\t%8.4f\t%8.4f\t%8.4f\t%.2f",
					 (double)home_position.yaw,
					 (double)home_position.x,
					 (double)home_position.y,
					 (double)home_position.valid_lpos);

			PX4_INFO("vehicle_status:\t%8.4d",
					 (int)vehicle_status.gcs_connection_lost);

			float home_x = home_position.lat * 1e-7;
			float home_y = home_position.lon * 1e-7;
			float current_x = vehicle_global_position.lat * 1e-7;
			float current_y = vehicle_global_position.lon * 1e-7;

			float home_xm = home_position.x;
			float home_ym = home_position.y;
			float current_xm = vehicle_local_position.x;
			float current_ym = vehicle_local_position.y;
			float heading_to_home = atan2(home_y - current_y, home_x - current_x);
			float distance_to_home = sqrt((home_xm - current_xm) * (home_xm - current_xm) + (home_ym - current_ym)*(home_ym - current_ym));
			PX4_INFO("heading_to_home:\t%8.4f \t distance_to_home \t %8.4f",
					 (double)math::degrees(heading_to_home),
					 (double)distance_to_home);
			_distance_to_home = distance_to_home;
			_heading_to_home = (float)math::degrees(heading_to_home);

		        /* advertise attitude topic */
			//struct vehicle_attitude_s att;
			//memset(&att, 0, sizeof(att));
			//orb_advertise(ORB_ID(vehicle_attitude), &att);
			//sendActionRequest(action_request_s::ACTION_SWITCH_MODE, action_request_s::SOURCE_RC_SWITCH,
			//						  vehicle_status_s::NAVIGATION_STATE_POSCTL);
			px4_sleep(40);
			sendCommandToFlightBackToHome();

// 			uORB::SubscriptionData<vehicle_local_position_s> local_position_sub{ORB_ID(vehicle_local_position)};
//   			uORB::SubscriptionData<vehicle_attitude_s> attitude_sub{ORB_ID(vehicle_attitude)};

// 			while (true)
//   {
// 				// Update the local position and heading
// 				local_position_sub.update();
// 				attitude_sub.update();

// 				// Check if the target position has been reached
// 				if (fabs(local_position_sub.get().x - target_position.x) < 0.1 &&
// 					fabs(local_position_sub.get().y - target_position.y) < 0.1 &&
// 					fabs(local_position_sub.get().z - target_position.z) < 0.1 )
// 				{
// 				// Target position has been reached, exit the loop
// 				break;
// 				}

// 				// Sleep for a short period of time before checking again
// 				px4_usleep(200000);
// 			}


		}

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
	orb_unsubscribe(home_position_sub);
	orb_unsubscribe(vehicle_status_sub);
	orb_unsubscribe(vehicle_global_position_sub);
	orb_unsubscribe(vehicle_local_position_sub);
}

void MyModule::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

void MyModule::sendCommandToFlightBackToHome(){
	if (check_conditions()){
	uORB::Subscription vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	/* Get the latest GPS publication */
	sensor_gps_s gps_pos;

	if (vehicle_gps_position_sub.copy(&gps_pos)) {
		PX4_INFO("GPS fix Type:\t%8.4f",(double)gps_pos.fix_type);
	}
		send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_ALTCTL);
		px4_sleep(1);

		uORB::Publication<sent_come_home_s> sent_come_home_pub{ORB_ID(sent_come_home)};
		uint64_t timestamp_us = hrt_absolute_time();
		sent_come_home_s sch{};
		sch.distance = _distance_to_home;
		sch.direction = _heading_to_home;
		sch.timestamp = timestamp_us;

		int published = sent_come_home_pub.publish(sch);
		printf("distance: %f\n", (double)sch.distance);
printf("direction: %f\n", (double)sch.direction);
printf("timestamp: %llu\n", (unsigned long long)sch.timestamp);
		PX4_INFO("publish:\t%8.4f\t%8.4f\t%8.4f\t%8.4f", (double)published,(double)_distance_to_home,(double)_heading_to_home,(double)sch.distance);
	}
}


void MyModule::sendActionRequest(int8_t action, int8_t source, int8_t mode)
{
	// We catch default unassigned mode slots which have value -1
	if (action == action_request_s::ACTION_SWITCH_MODE && mode < 0) {
		return;
	}

	action_request_s action_request{};
	action_request.action = action;
	action_request.source = source;
	action_request.mode = mode;
	action_request.timestamp = hrt_absolute_time();
	_action_request_pub.publish(action_request);
}

int MyModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int my_module_main(int argc, char *argv[])
{
	return MyModule::main(argc, argv);
}

bool MyModule::check_rth_conditions()
{
    // check for loss of GPS and controller signals
    if (!_gps_valid && (hrt_absolute_time() - _last_gps_failure_time > GPS_FAILURE_TIMEOUT) && !_local_pos_valid) {
        return true;
    }
    return false;
}



bool MyModule::check_conditions()
{
    return true;
    // check for loss of GPS and controller signals
    if (!_gps_valid && (hrt_absolute_time() - _last_gps_failure_time > GPS_FAILURE_TIMEOUT) && !_local_pos_valid) {
        return true;
    }
    return false;
}

void MyModule::calculateHomeDirectionAndDistance()
{
    // Calculate direction and distance to home using GPS and home position
    if (_home_valid) {
        _home_direction_xy(0) = _home.lat - _gps.lat;
        _home_direction_xy(1) = _home.lon - _gps.lon;
        _home_direction_xy.normalize();
        _home_distance = sqrtf((_home.lat - _gps.lat) * (_home.lat - _gps.lat) + (_home.lon - _gps.lon) * (_home.lon - _gps.lon));
    }
}

void MyModule::waitForDistance(double distance)
{
    // Wait for drone to travel certain distance
    while (_local_pos.xy_valid && (double)sqrtf((_local_pos.x) * (_local_pos.x) + (_local_pos.y) * (_local_pos.y)) < distance) {
        orb_check(_local_pos_sub, &_local_pos_updated);
        if (_local_pos_updated) {
            orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
        }
    }
}

void MyModule::checkRTHCondition()
{
    if (_armed && check_rth_conditions()) {
        if (!_rth_initiated) {
            // Initiate RTH
            _rth_initiated = true;
            // Set velocity towards home direction at a set acceleration
            matrix::Vector3f velocity = {0.f, 0.f, 0.f};
            velocity(0) = _home_direction_xy(0) * ACCELERATION;
            velocity(1) = _home_direction_xy(1) * ACCELERATION;
        //     _velocity_pub.publish(velocity);
            waitForDistance((double)_home_distance);
            velocity(0) = velocity(1) = 0.0f;
        //     _velocity_pub.publish(velocity);
        }
    } else {
        _rth_initiated = false;
    }
}

void MyModule::run1()
{
    while (!should_exit()) {
        orb_check(_params_sub, &_params_updated);
        orb_check(_vehicle_status_sub, &_vehicle_status_updated);
        orb_check(_gps_sub, &_gps_updated);
        orb_check(_home_sub, &_home_updated);
        orb_check(_local_pos_sub, &_local_pos_updated);

        if (_params_updated) {
            // handle parameter update
        }

        if (_vehicle_status_updated) {
            orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
            _armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
        }

        if (_gps_updated) {
            orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &_gps);
            if (_gps.terrain_alt_valid) {
                _gps_valid = true;
                _last_gps_failure_time = 0;
                calculateHomeDirectionAndDistance();
            } else {
                _gps_valid = false;
                if (_last_gps_failure_time == 0) {
                    _last_gps_failure_time = hrt_absolute_time();
                }
            }
        }

        if (_home_updated) {
            orb_copy(ORB_ID(home_position), _home_sub, &_home);
            _home_valid = _home.valid_alt && _home.valid_hpos;
        }

        if (_local_pos_updated) {
            orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
            _local_pos_valid = PX4_ISFINITE(_local_pos.x) && PX4_ISFINITE(_local_pos.y);
        }

        checkRTHCondition();
    }
}

MyModule::~MyModule()
{
    if (_params_sub >= 0) {
        orb_unsubscribe(_params_sub);
    }
    if (_vehicle_status_sub >= 0) {
        orb_unsubscribe(_vehicle_status_sub);
    }
    if (_gps_sub >= 0) {
        orb_unsubscribe(_gps_sub);
    }
    if (_home_sub >= 0) {
        orb_unsubscribe(_home_sub);
    }
    if (_local_pos_sub >= 0) {
        orb_unsubscribe(_local_pos_sub);
    }
}
