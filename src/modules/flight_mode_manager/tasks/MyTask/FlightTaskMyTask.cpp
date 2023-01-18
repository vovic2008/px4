/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskMyTask.cpp
 */

#include "FlightTaskMyTask.hpp"
#include <matrix/matrix/math.hpp>
bool a = false;

bool FlightTaskMyTask::activate(trajectory_setpoint_s last_setpoint)
{
  bool ret = FlightTaskManualAltitude::activate(last_setpoint);
    // subscribe to the sent_come_home topic
    sent_come_home_sub1 = _sent_come_home_sub.subscribe();
  PX4_INFO("FlightTaskMyTask activate was called! ret: %d", ret); // report if activation was successful
  return ret;
}

bool FlightTaskMyTask::update()
{
  bool ret = FlightTaskManualAltitude::update();

  subscribe();
  if (!a){
	PX4_INFO("Current yaw %f",(double)_yaw); // report update
	a = true;
  }

  check_stop();

  return ret;
}

void FlightTaskMyTask::check_stop()
{
	float current_time_s = hrt_absolute_time() / 1e6f;
	if(current_time_s>_time_in_seconds_to_stop&& _time_in_seconds_to_stop!=0){
		stop();
	}
}

void FlightTaskMyTask::subscribe()
{
	sent_come_home_s raw{};
	_sent_come_home_sub.update(&raw);
        _sent_come_home_updated = _sent_come_home_sub.updated();
	/* one could wait for multiple topics with this technique, just using one here */
	// px4_pollfd_struct_t fds[] = {
	// { .fd = sent_come_home_sub1,   .events = POLLIN },
	// };
        // PX4_INFO("subscribe:\t%8.4f\t%8.4f",
	// 					(double)check,
	// 					(double)_sent_come_home_updated);
// PX4_INFO("UP");

	/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
	// px4_poll(fds, 1, 1000);
	// if (fds[0].revents & POLLIN) {
		/* obtained data for the first file descriptor */
		if (_sent_come_home_updated){
			// struct sent_come_home_s raw;
			/* copy sensors raw data into local buffer */
			_sent_come_home_sub.copy(&raw);
			// orb_copy(ORB_ID(sent_come_home), sent_come_home_sub1, &raw);
			PX4_INFO("sent_come_home:\t%8.4f\t%8.4f\t%8.4f",
						(double)raw.direction,
						(double)raw.distance,
						(double)_yaw);
			_timestamp = raw.timestamp;
			float current_time_s = raw.timestamp / 1e6f;
			_time_in_seconds_to_stop = current_time_s + sqrt(2 * raw.distance / ACCELERATION);
			// set_acceleration_setpoint(raw.direction);
		}
	// }
}



void FlightTaskMyTask::stop()
{
    // set target acceleration (2 m/s^2) in a specific direction
    matrix::Vector3f acceleration_setpoint;
    acceleration_setpoint(0) = 0; // x-direction
    acceleration_setpoint(1) = 0; // y-direction
    acceleration_setpoint(2) = 0.0; // z-direction
    _acceleration_setpoint = acceleration_setpoint;
}

bool FlightTaskMyTask::set_acceleration_setpoint(float direction)
{
    // set target acceleration (2 m/s^2) in a specific direction
    matrix::Vector3f acceleration_setpoint;
    acceleration_setpoint(0) = cos(direction)*ACCELERATION; // x-direction
    acceleration_setpoint(1) = sin(direction)*ACCELERATION; // y-direction
    acceleration_setpoint(2) = 0.0; // z-direction
    _acceleration_setpoint = acceleration_setpoint;

//     float yaw = _yaw;
//     _yaw_setpoint = yaw;
//     _yawspeed_setpoint = 0.f;

//     // publish the setpoint
//     vehicle_local_position_setpoint_s setpoint{};
//     setpoint.timestamp = hrt_absolute_time();
//     setpoint.x = acceleration_setpoint(0);
//     setpoint.y = acceleration_setpoint(1);
//     setpoint.z = acceleration_setpoint(2);
//     setpoint.yaw = NAN;

//     orb_advert_t handle = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &setpoint);
// 	if (handle != OK)  {
// 		PX4_ERR("advertise failed");
// 		return false;
// 	}
	return true;
}
