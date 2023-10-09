#include <cmath>
#include "Control.h"
#include "Types.h"

#define PI 3.1415926f
#define ROBOT_LOOP_MS 10

Control::Control() {
	target.command = ControlType::WHEEL_VELOCITY_CONTROL;
	target.wheel_velocity = WheelVelocity{0, 0};
}

WheelVelocity Control::control_loop(Pose pose) {
	this->pose = pose;
	switch (target.command) {
		case ControlType::VECTOR_CONTROL:
			return vector_control();
		case ControlType::POSITION_CONTROL:
			return position_control();
		case ControlType::ORIENTATION_CONTROL:
			return orientation_control();
		case ControlType::UVF_CONTROL:
			return uvf_control();
		case ControlType::WHEEL_VELOCITY_CONTROL:
			return target.wheel_velocity;
		default:
			return WheelVelocity{0, 0};
	}
}

WheelVelocity Control::uvf_control() {
	if(vel_acelerada < 0.3) vel_acelerada = 0.3;
	if(target.velocity == 0) {
		// stop_and_wait();
		return WheelVelocity{0, 0};
	}

//	Computes uni-vector field target theta
	float state_to_targ = std::atan2(target.y - pose.y, target.x - pose.x);
	float state_to_ref = std::atan2(target.ref_y - pose.y, target.ref_x - pose.x);
	float fi = round_angle(state_to_ref - state_to_targ);
	float uvf_target_theta = round_angle(state_to_targ - target.uvf_n * fi);

//	Activates backwards movement if theta_error > PI/2
	bool move_backwards = backwards_select(uvf_target_theta, pose.theta);
	if(move_backwards) uvf_target_theta = round_angle(uvf_target_theta + PI);

	float theta_error = round_angle(uvf_target_theta - pose.theta);

//	Decreases velocity for big errors
	if (std::abs(theta_error) > max_theta_error) {
		vel_acelerada = vel_acelerada - 2 * acc_rate * ROBOT_LOOP_MS/1000.0f;
	} else {
//		Applies acceleration until robot reaches target velocity
		if (vel_acelerada < target.velocity) {
			vel_acelerada = vel_acelerada + acc_rate * ROBOT_LOOP_MS/1000.0f;
		} else {
			vel_acelerada = target.velocity;
		}
	}

	return set_wheel_velocity_nonlinear_controller(theta_error, target.velocity, move_backwards);
}

WheelVelocity Control::vector_control() {
	if(vel_acelerada < 0.3) vel_acelerada = 0.3;
	if(target.velocity == 0) {
		// stop_and_wait();
		return WheelVelocity{0, 0};
	}
//	Computes target_theta in direction of {target.x, target.y} before each control loop
	float target_theta = std::atan2(target.y - pose.y, target.x - pose.x);

//	Activates backwards movement if theta_error > PI/2
	bool move_backwards = backwards_select(target_theta, pose.theta);
	if(move_backwards) target_theta = round_angle(target_theta + PI);

	float theta_error = round_angle(target_theta - pose.theta);

//	Decreases velocity for big errors
	if (std::abs(theta_error) > max_theta_error) {
		vel_acelerada = vel_acelerada - 2 * acc_rate * ROBOT_LOOP_MS/1000.0f;
	} else {
//		Applies acceleration until robot reaches target velocity
		if (vel_acelerada < target.velocity) {
			vel_acelerada = vel_acelerada + acc_rate * ROBOT_LOOP_MS/1000.0f;
		} else {
			vel_acelerada = target.velocity;
		}
	}

	return set_wheel_velocity_nonlinear_controller(theta_error, target.velocity, move_backwards);
}

WheelVelocity Control::position_control() {
//	Stops after arriving at destination
	float position_error = std::sqrt(std::pow(pose.x - target.x, 2.0f) + std::pow(pose.y - target.y, 2.0f));
	if(target.velocity == 0) {
		// stop_and_wait();
		return WheelVelocity{0, 0};
	}

	if(vel_acelerada < 0.3) vel_acelerada = 0.3;

//	Computes target_theta in direction of {target.x, target.y} before each control loop
	float target_theta = std::atan2(target.y - pose.y, target.x - pose.x);

//	Activates backwards movement if theta_error > PI/2
	bool move_backwards = backwards_select(target_theta, pose.theta);
	if(move_backwards) target_theta = round_angle(target_theta + PI);

	float theta_error = round_angle(target_theta - pose.theta);

//	Decreases velocity for big errors and limits maximum velocity
	if (std::abs(theta_error) > max_theta_error) {
		if(vel_acelerada > 0.8) {
			vel_acelerada = 0.8;
		} else if (vel_acelerada > 0.3) {
			vel_acelerada = vel_acelerada - 2 * acc_rate * ROBOT_LOOP_MS/1000.0f;
		}
	} else {
//		Applies acceleration until robot reaches target velocity
		float velocity_difference = target.velocity - vel_acelerada;
		if (velocity_difference > 0.2) {
			vel_acelerada = vel_acelerada + acc_rate * ROBOT_LOOP_MS/1000.0f;
		} else if(velocity_difference < 0) {
			vel_acelerada = target.velocity;
		}
	}

	return set_wheel_velocity_nonlinear_controller(theta_error, target.velocity * std::tanh(5 * position_error), move_backwards);
}

WheelVelocity scale_velocity(float left, float right, float scale) {
	return {left * scale, right * scale};
}

WheelVelocity Control::orientation_control() {
	float target_theta = target.theta;

//	Activates backwards movement if theta_error > PI/2
	if(backwards_select(target_theta, pose.theta))
		target_theta = round_angle(target_theta + PI);

	float theta_error = round_angle(target_theta - pose.theta);

//	Wheel velocities are always between 1 and -1
	float right_wheel_velocity = saturate(orientation_Kp * theta_error, 1);
	float left_wheel_velocity = saturate(-orientation_Kp * theta_error, 1);

	return scale_velocity(left_wheel_velocity, right_wheel_velocity, target.velocity);
}

WheelVelocity Control::set_wheel_velocity_nonlinear_controller(float theta_error, float velocity, bool backwards) {
	float m = 1;
	if(backwards) m = -1;

	float right_wheel_velocity = m + std::sin(theta_error) + m*kgz*std::tan(m*theta_error/2);
	right_wheel_velocity = saturate(right_wheel_velocity,1);

	float left_wheel_velocity = m - std::sin(theta_error) + m*kgz*std::tan(-m*theta_error/2);
	left_wheel_velocity = saturate(left_wheel_velocity,1);

	return scale_velocity(left_wheel_velocity, right_wheel_velocity, velocity);
}

// void Control::start_uvf_control(float x, float y, float x_ref, float y_ref, float n, float velocity) {
// 	target.x = x;
// 	target.y = y;
// 	target.ref_x = x_ref;
// 	target.ref_y = y_ref;
// 	uvf_n = n;
// 	target.velocity = velocity;
// 	target.command = UVF_CONTROL;
// 	continue_threads();
// }

// void Control::start_vector_control(float theta, float velocity) {
// //	target.theta is computed before each vector control loop, in direction of {target.x, target.y}
// 	target.x = 10*std::cos(theta * PI/180);
// 	target.y = 10*std::sin(theta * PI/180);
// 	target.velocity = velocity;
// 	target.command = VECTOR_CONTROL;
// 	continue_threads();
// }

// void Control::start_position_control(float x, float y, float velocity) {
// 	target.x = x;
// 	target.y = y;
// 	target.velocity = velocity;
// 	target.command = POSITION_CONTROL;
// 	continue_threads();
// }

// void Control::start_orientation_control(float theta, float velocity) {
// 	target.theta = round_angle(theta * PI/180);
// 	target.velocity = velocity;
// 	target.command = ORIENTATION_CONTROL;
// 	continue_threads();
// }

// void Control::start_velocity_control(float vel_left, float vel_right) {
// 	target.command = NO_CONTROL;
// 	controller.set_target_velocity(vel_left, vel_right, 1);
// 	continue_threads();
// }

// void Control::sensor_calibration() {
// 	#define sample_size_gyro_s 10000
// 	#define max_velocity 1.17f
// 	for (int i = 0; i < sample_size_gyro_s; ++i) {
// 		float v = max_velocity * i/sample_size_gyro_s;
// 		controller.set_target_velocity(-v, v, 1);
// //		controller.set_target_velocity(-calibration_velocity,
// //									   calibration_velocity, 1);
// 		Thread::wait(10);
// 	}
// 	sensors->wait = false;
// 	stop_and_wait();
// }

// void Control::start_calibration(float v) {
// 	calibration_velocity = v;
// 	sensors->wait = true;
// 	target.command = SENSOR_CALIBRATION;
// 	continue_threads();
// }


// void Control::stop_and_wait() {
// //	Flag tells Controller to stop robot and pause thread
// 	controller.stop = true;
// 	vel_acelerada = 0;
// 	target.command = NO_CONTROL;
// 	if(control_thread.get_state() != Thread::WaitingThreadFlag) {
// 		Thread::signal_wait(CONTINUE_SIGNAL);
// 		Thread::signal_clr(CONTINUE_SIGNAL);
// 	}
// }

float Control::round_angle(float angle) {
	float theta = std::fmod(angle, 2*PI);
	if(theta > PI) theta = theta - 2*PI;
	else if(theta < -PI) theta = theta + 2*PI;
	return theta;
}

float Control::saturate(float value, float limit) {
	if(value > limit) value = limit;
	if(value < -limit) value = -limit;
	return value;
}

void Control::set_max_theta_error(float error) {
	max_theta_error = round_angle(error * PI/180);
}

bool Control::backwards_select(float target, float orientation) {
	// return false;
	float now = time_now();
	float time_since_change = now - last_backwards_change;
	if(time_since_change > 25.0 * 0.001) {
		bool backwards = std::abs(round_angle(target - orientation)) > PI/2;
		if(previously_backwards != backwards) {
			last_backwards_change = now;
			vel_acelerada = 0.3;
		}
		previously_backwards = backwards;
		return backwards;
	} else {
		return previously_backwards;
	}
}
