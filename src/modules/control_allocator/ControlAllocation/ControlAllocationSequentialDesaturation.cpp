/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationSequentialDesaturation.cpp
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "ControlAllocationSequentialDesaturation.hpp"


void
ControlAllocationSequentialDesaturation::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	switch (_param_mc_airmode.get()) {
	case 1:
		mixAirmodeRP();
		break;

	case 2:
		mixAirmodeRPY();
		break;

	default:
		mixAirmodeDisabled();
		break;
	}
}

// void ControlAllocationSequentialDesaturation::desaturateActuators(
// 	ActuatorVector &actuator_sp,
// 	const ActuatorVector &desaturation_vector, bool increase_only)
// {
// 	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);

// 	if (increase_only && gain < 0.f) {
// 		return;
// 	}

// 	for (int i = 0; i < _num_actuators; i++) {
// 		actuator_sp(i) += gain * desaturation_vector(i);
// 	}

// 	gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);

// 	for (int i = 0; i < _num_actuators; i++) {
// 		actuator_sp(i) += gain * desaturation_vector(i);
// 	}
// }

// float ControlAllocationSequentialDesaturation::computeDesaturationGain(const ActuatorVector &desaturation_vector,
// 		const ActuatorVector &actuator_sp)
// {
// 	float k_min = 0.f;
// 	float k_max = 0.f;

// 	for (int i = 0; i < _num_actuators; i++) {
// 		// Do not use try to desaturate using an actuator with weak effectiveness to avoid large desaturation gains
// 		if (fabsf(desaturation_vector(i)) < 0.2f) {
// 			continue;
// 		}

// 		if (actuator_sp(i) < _actuator_min(i)) {
// 			float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);

// 			if (k < k_min) { k_min = k; }

// 			if (k > k_max) { k_max = k; }
// 		}

// 		if (actuator_sp(i) > _actuator_max(i)) {
// 			float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);

// 			if (k < k_min) { k_min = k; }

// 			if (k > k_max) { k_max = k; }
// 		}
// 	}

// 	// Reduce the saturation as much as possible
// 	return k_min + k_max;
// }



// Jan's version for debugging
void ControlAllocationSequentialDesaturation::desaturateActuators(
    ActuatorVector &actuator_sp,
    const ActuatorVector &desaturation_vector,
    bool increase_only)
{
    float gain = computeDesaturationGain(desaturation_vector, actuator_sp);
//     float _pusher_scale = _param_ca_pusher_scale.get();

    // Print initial gain
    // PX4_INFO("Desaturation gain (initial): %f", (double)gain);

    if (increase_only && gain < 0.f) {
        // PX4_INFO("Skipping desaturation as increase_only is true and gain is negative");
        return;
    }

    // Apply the initial gain to desaturate actuators
    for (int i = 0; i < _num_actuators; i++) {
        actuator_sp(i) += gain * desaturation_vector(i);
    }

    // Print actuator setpoints after first gain application
    // for (int i = 0; i < _num_actuators; i++) {
    //     PX4_INFO("Actuator %d - Setpoint after first gain application: %f", i, (double)actuator_sp(i));
    // }

    // Calculate and apply a reduced gain (0.5 * gain)
    gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);
    // PX4_INFO("Desaturation gain (reduced): %f", (double)gain);

    for (int i = 0; i < _num_actuators; i++) {
        actuator_sp(i) += gain * desaturation_vector(i);
    }
//     cap i=8 and i=9 between [-0.3,0.3]
//     for (int i = 0; i < _num_actuators; i++) {
//         if (i == 8 || i == 9) {
//             actuator_sp(i) = actuator_sp(i) * _pusher_scale;
//         }
//     }

    // Print actuator setpoints after reduced gain application
    // for (int i = 0; i < _num_actuators; i++) {
    //     PX4_INFO("Actuator %d - Final Setpoint after reduced gain application: %f", i, (double)actuator_sp(i));
    // }
}
float ControlAllocationSequentialDesaturation::computeDesaturationGain(
    const ActuatorVector &desaturation_vector,
    const ActuatorVector &actuator_sp)
{
    float k_min = 0.f;
    float k_max = 0.f;

    for (int i = 0; i < _num_actuators; i++) {
        // Skip actuators with weak effectiveness for desaturation
        if (fabsf(desaturation_vector(i)) < 0.2f) {
            // PX4_INFO("Skipping actuator %d for desaturation due to low effectiveness: %f", i, (double)desaturation_vector(i));
            continue;
        }

        if (actuator_sp(i) < _actuator_min(i)) {
            float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);

            // Print k_min and k_max after updating for current actuator
            // PX4_INFO("Actuator %d - k (below min): %f, k_min: %f, k_max: %f", i, (double)k, (double)k_min, (double)k_max);
        }

        if (actuator_sp(i) > _actuator_max(i)) {
            float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);

            // Print k_min and k_max after updating for current actuator
            // PX4_INFO("Actuator %d - k (above max): %f, k_min: %f, k_max: %f", i, (double)k, (double)k_min, (double)k_max);
        }
    }

    // Print the final computed gain
    float gain = k_min + k_max;
    // PX4_INFO("Final computed desaturation gain: %f", (double)gain);

    return gain;
}

// improved version of actuator desaturation by splitting the desaturation between hover and pusher motor groups, Jan
void ControlAllocationSequentialDesaturation::desaturatePusherActuatorsSep(
	ActuatorVector &actuator_sp,
	const ActuatorVector &desaturation_vector, bool increase_only)
{
	float gain_l = computeLeftPusherDesaturationGain(desaturation_vector, actuator_sp);
	float gain_r = computeRightPusherDesaturationGain(desaturation_vector, actuator_sp);
	// float _pusher_scale = _param_ca_pusher_scale.get();

	if (increase_only && gain_l < 0.f && gain_r < 0.f) {
		return;
	}

	for (int i = 0; i < _num_actuators; i++) {
		if (i == 8) {
            actuator_sp(i) += gain_l * desaturation_vector(i);
        }
	if (i == 9) {
	    actuator_sp(i) += gain_r * desaturation_vector(i);
	}
	}

	gain_l = 0.5f * computeLeftPusherDesaturationGain(desaturation_vector, actuator_sp);
	gain_r = 0.5f * computeRightPusherDesaturationGain(desaturation_vector, actuator_sp);

	for (int i = 0; i < _num_actuators; i++) {
        if (i == 8) {
            actuator_sp(i) += gain_l * desaturation_vector(i);
        }
	if (i == 9) {
	    actuator_sp(i) += gain_r * desaturation_vector(i);
	}
	}

	// actuator_sp(8) = actuator_sp(8) * _pusher_scale;
	// actuator_sp(9) = actuator_sp(9) * _pusher_scale;

}

void ControlAllocationSequentialDesaturation::desaturatePusherActuators(
	ActuatorVector &actuator_sp,
	const ActuatorVector &desaturation_vector, bool increase_only)
{
	float gain = computePusherDesaturationGain(desaturation_vector, actuator_sp);

	if (increase_only && gain < 0.f) {
		return;
	}

	for (int i = 0; i < _num_actuators; i++) {
		if (i == 8 || i == 9) {
            		actuator_sp(i) += gain * desaturation_vector(i);
        	}
	}

	gain = 0.5f * computePusherDesaturationGain(desaturation_vector, actuator_sp);

	for (int i = 0; i < _num_actuators; i++) {
        if (i == 8 || i == 9) {
            actuator_sp(i) += gain * desaturation_vector(i);
        }
	}
}

void ControlAllocationSequentialDesaturation::desaturateHoverActuators(
    ActuatorVector &actuator_sp,
    const ActuatorVector &desaturation_vector, bool increase_only)
{
    float gain = computeHoverDesaturationGain(desaturation_vector, actuator_sp);

    if (increase_only && gain < 0.f) {
        return;
    }

    for (int i = 0; i < _num_actuators; i++) {
        if (i != 8 && i != 9) {
            actuator_sp(i) += gain * desaturation_vector(i);
        }
    }

    gain = 0.5f * computeHoverDesaturationGain(desaturation_vector, actuator_sp);

    for (int i = 0; i < _num_actuators; i++) {
        if (i != 8 && i != 9) {
            actuator_sp(i) += gain * desaturation_vector(i);
        }
    }
}
float ControlAllocationSequentialDesaturation::computePusherDesaturationGain(
    const ActuatorVector &desaturation_vector,
    const ActuatorVector &actuator_sp)
{
    float k_min = 0.f;
    float k_max = 0.f;

    for (int i = 8; i < 10; i++) {
        if (fabsf(desaturation_vector(i)) < 0.2f) {
            continue;
        }

        if (actuator_sp(i) < _actuator_min(i)) {
            float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);
        }

        if (actuator_sp(i) > _actuator_max(i)) {
            float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);
        }
    }

    return k_min + k_max;
}
// compute desaturation gain left pusher index 8
float ControlAllocationSequentialDesaturation::computeLeftPusherDesaturationGain(
    const ActuatorVector &desaturation_vector,
    const ActuatorVector &actuator_sp)
{
    float k_min = 0.f;
    float k_max = 0.f;

    for (int i = 8; i < 9; i++) {
        if (fabsf(desaturation_vector(i)) < 0.2f) {
            continue;
        }

        if (actuator_sp(i) < _actuator_min(i)) {
            float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);
        }

        if (actuator_sp(i) > _actuator_max(i)) {
            float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);
        }
    }

    return k_min + k_max;
}
// compute desaturation gain left pusher index 8
float ControlAllocationSequentialDesaturation::computeRightPusherDesaturationGain(
    const ActuatorVector &desaturation_vector,
    const ActuatorVector &actuator_sp)
{
    float k_min = 0.f;
    float k_max = 0.f;

    for (int i = 9; i < 10; i++) {
        if (fabsf(desaturation_vector(i)) < 0.2f) {
            continue;
        }

        if (actuator_sp(i) < _actuator_min(i)) {
            float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);
        }

        if (actuator_sp(i) > _actuator_max(i)) {
            float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);
        }
    }

    return k_min + k_max;
}
float ControlAllocationSequentialDesaturation::computeHoverDesaturationGain(
    const ActuatorVector &desaturation_vector,
    const ActuatorVector &actuator_sp)
{
    float k_min = 0.f;
    float k_max = 0.f;

    for (int i = 0; i < 8; i++) {
        if (fabsf(desaturation_vector(i)) < 0.2f) {
            continue;
        }

        if (actuator_sp(i) < _actuator_min(i)) {
            float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);
        }

        if (actuator_sp(i) > _actuator_max(i)) {
            float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);
            k_min = fminf(k_min, k);
            k_max = fmaxf(k_max, k);
        }
    }

    return k_min + k_max;
}


void
ControlAllocationSequentialDesaturation::mixAirmodeRP()
{
	// Airmode for roll and pitch, but not yaw

	// Mix without yaw
	ActuatorVector thrust_z;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}

	desaturateActuators(_actuator_sp, thrust_z);

	// Mix yaw independently
	mixYaw();
}

void
ControlAllocationSequentialDesaturation::mixAirmodeRPY()
{
	// Airmode for roll, pitch and yaw

	// Do full mixing
	ActuatorVector thrust_z;
	ActuatorVector yaw;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		yaw(i) = _mix(i, ControlAxis::YAW);
	}

	desaturateActuators(_actuator_sp, thrust_z);

	// Unsaturate yaw (in case upper and lower bounds are exceeded)
	// to prioritize roll/pitch over yaw.
	desaturateActuators(_actuator_sp, yaw);
}

void
ControlAllocationSequentialDesaturation::mixAirmodeDisabled()
{
	// Airmode disabled: never allow to increase the thrust to unsaturate a motor

	// Mix without yaw
	ActuatorVector thrust_z;
	ActuatorVector roll;
	ActuatorVector pitch;

	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) = _actuator_trim(i) +
				  _mix(i, ControlAxis::ROLL) * (_control_sp(ControlAxis::ROLL) - _control_trim(ControlAxis::ROLL)) +
				  _mix(i, ControlAxis::PITCH) * (_control_sp(ControlAxis::PITCH) - _control_trim(ControlAxis::PITCH)) +
				  _mix(i, ControlAxis::THRUST_X) * (_control_sp(ControlAxis::THRUST_X) - _control_trim(ControlAxis::THRUST_X)) +
				  _mix(i, ControlAxis::THRUST_Y) * (_control_sp(ControlAxis::THRUST_Y) - _control_trim(ControlAxis::THRUST_Y)) +
				  _mix(i, ControlAxis::THRUST_Z) * (_control_sp(ControlAxis::THRUST_Z) - _control_trim(ControlAxis::THRUST_Z));
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
		roll(i) = _mix(i, ControlAxis::ROLL);
		pitch(i) = _mix(i, ControlAxis::PITCH);
	}

	// only reduce thrust
	desaturateActuators(_actuator_sp, thrust_z, true);

	// Reduce roll/pitch acceleration if needed to unsaturate
	desaturateActuators(_actuator_sp, roll);
	desaturateActuators(_actuator_sp, pitch);

	// Mix yaw independently
	mixYaw();
}

// void
// ControlAllocationSequentialDesaturation::mixYaw()
// {
// 	// Add yaw to outputs
// 	ActuatorVector yaw;
// 	ActuatorVector thrust_z;

// 	for (int i = 0; i < _num_actuators; i++) {
// 		_actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
// 		yaw(i) = _mix(i, ControlAxis::YAW);
// 		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
// 	}

// 	// Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
// 	// and allow some yaw response at maximum thrust
// 	ActuatorVector max_prev = _actuator_max;
// 	_actuator_max += (_actuator_max - _actuator_min) * 0.15f;
// 	desaturateActuators(_actuator_sp, yaw);
// 	_actuator_max = max_prev;

// 	// reduce thrust only
// 	desaturateActuators(_actuator_sp, thrust_z, true);
// }

// Jan's version with print statements for debugging
// void ControlAllocationSequentialDesaturation::mixYaw()
// {
//     // Add yaw to outputs
//     ActuatorVector yaw;
//     ActuatorVector thrust_z;

//     // Print yaw control setpoint and trim for debugging
//     PX4_INFO("Yaw Control Setpoint: %f, Trim: %f", (double)_control_sp(ControlAxis::YAW), (double)_control_trim(ControlAxis::YAW));

//     for (int i = 0; i < _num_actuators; i++) {
//         _actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
//         yaw(i) = _mix(i, ControlAxis::YAW);
//         thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);

//         // Print yaw effectiveness and resulting actuator setpoints
//         PX4_INFO("Actuator %d - Yaw Effectiveness: %f, Resulting Actuator Setpoint: %f", i, (double)yaw(i), (double)_actuator_sp(i));
//     }

//     // Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
//     // and allow some yaw response at maximum thrust
//     ActuatorVector max_prev = _actuator_max;
//     _actuator_max += (_actuator_max - _actuator_min) * 0.15f;
//     desaturateHoverActuators(_actuator_sp, yaw);
//     desaturatePusherActuators(_actuator_sp, yaw);
//     _actuator_max = max_prev;

//     // Print actuator setpoints after yaw desaturation
//     for (int i = 0; i < _num_actuators; i++) {
//         PX4_INFO("Actuator %d - Setpoint after yaw desaturation: %f", i, (double)_actuator_sp(i));
//     }

//     // reduce thrust only
//     desaturateActuators(_actuator_sp, thrust_z, true);

//     // Print final actuator setpoints after thrust reduction
//     for (int i = 0; i < _num_actuators; i++) {
//         PX4_INFO("Actuator %d - Final Setpoint after thrust reduction: %f", i, (double)_actuator_sp(i));
//     }
// }

void
ControlAllocationSequentialDesaturation::mixYaw()
{
	// Add yaw to outputs
	ActuatorVector yaw;
	ActuatorVector thrust_z;

	// Print yaw control setpoint and trim for debugging
//     PX4_INFO("Yaw Control Setpoint: %f, Trim: %f", (double)_control_sp(ControlAxis::YAW), (double)_control_trim(ControlAxis::YAW));


	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
		yaw(i) = _mix(i, ControlAxis::YAW);
		thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);

        // Print yaw effectiveness and resulting actuator setpoints
        // PX4_INFO("Actuator %d - Yaw Effectiveness: %f, Resulting Actuator Setpoint: %f", i, (double)yaw(i), (double)_actuator_sp(i));
	}

	// Get the pusher mode parameter value
    	int32_t pusher_mode = _param_ca_pusher_mode.get();

	// Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
	// and allow some yaw response at maximum thrust
	ActuatorVector max_prev = _actuator_max;
	_actuator_max += (_actuator_max - _actuator_min) * 0.15f;

	switch (pusher_mode) {
	case 0: // No yaw control in hover, pushers unidirectional
		desaturateHoverActuators(_actuator_sp, yaw);
		desaturatePusherActuators(_actuator_sp, yaw);
		break;

	case 1: // Yaw control in hover, pushers unidirectional
		desaturateHoverActuators(_actuator_sp, yaw);
		desaturatePusherActuatorsSep(_actuator_sp, yaw);
		break;

	case 2: // Yaw control in hover, pushers bidirectional
		desaturateActuators(_actuator_sp, yaw);
		break;

	default:
		// Handle invalid parameter values
		desaturateHoverActuators(_actuator_sp, yaw);
		desaturatePusherActuators(_actuator_sp, yaw);
		break;
    	}

	_actuator_max = max_prev;

    // Print actuator setpoints after yaw desaturation
//     for (int i = 0; i < _num_actuators; i++) {
//         PX4_INFO("Actuator %d - Setpoint after yaw desaturation: %f", i, (double)_actuator_sp(i));
//     }

	// reduce thrust only
	desaturateActuators(_actuator_sp, thrust_z, true);

    // Print final actuator setpoints after thrust reduction
//     for (int i = 0; i < _num_actuators; i++) {
//         PX4_INFO("Actuator %d - Final Setpoint after thrust reduction: %f", i, (double)_actuator_sp(i));
//     }
}

void
ControlAllocationSequentialDesaturation::updateParameters()
{
	updateParams();
}
