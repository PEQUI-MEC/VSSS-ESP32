#include "EkfModel.h"

T::PoseVec EkfModel::prediction(const T::PoseVec &prev_x,
								  const T::ControlVec &controls, float time) {
	Pose pose(prev_x);
	Controls c(controls);
	Pose pred;

	float delta_theta = (pose.w * time) / 2;
	float x_direction = time * std::cos(pose.theta + delta_theta);
	float x_increment = pose.v * x_direction;
	float y_direction = time * std::sin(pose.theta + delta_theta);
	float y_increment = pose.v * y_direction;

	pred.x = pose.x + x_increment;
	pred.y = pose.y + y_increment;
//	pred.theta = wrap(pose.theta + pose.w * time);
	pred.theta = pose.theta + pose.w * time;
	pred.v = pose.v + c.lin_accel * time;
	pred.w = pose.w + c.ang_accel * time;

	process_noise(time);
	return pred.to_vec();
}

void EkfModel::process_noise(float time) {
	R(0, 0) = time * 0.0001f;
	R(1, 1) = time * 0.0001f;
	R(2, 2) = time * 0.00001f;
	R(3, 3) = time * 0.0001f;
	R(4, 4) = time * 0.0001f;
}

T::SensorVec EkfModel::sensor_measurement_error(const T::PoseVec &x, const T::SensorVec &z) {
	auto pred_z = sensor_measurement_model(x);
	T::SensorVec z_error = z - pred_z;
	return z_error;
}

T::SensorVec EkfModel::sensor_measurement_model(const T::PoseVec &x) {
	T::SensorVec z;
	z(0,0) = x(4,0);
	float v_increment = x(4,0) * ROBOT_SIZE/2;
	z(1,0) = x(3,0) - v_increment;
	z(2,0) = x(3,0) + v_increment;
	return z;
}

T::VisionVec EkfModel::vision_measurement_error(const T::PoseVec &x, const T::VisionVec &z) {
	auto pred_z = vision_measurement_model(x);
	T::VisionVec error = z - pred_z;
	error(2,0) = wrap(error(2,0));
	return error;
}

T::VisionVec EkfModel::vision_measurement_model(const T::PoseVec &x) {
	T::VisionVec z;
	for (int i = 0; i < 3; ++i) {
		z(i,0) = x(i,0);
	}
	return z;
}

void EkfModel::use_encoders(bool use) {
//	if(use) {
//		H(2,3) = 1;
//		H(3,3) = 1;
//		H(2,4) = -ROBOT_SIZE / 2;
//		H(3,4) = ROBOT_SIZE / 2;
//	} else {
//		H(2,3) = 0;
//		H(3,3) = 0;
//		H(2,4) = 0;
//		H(3,4) = 0;
//	}
}

EkfModel::EkfModel() {
	R.setZero();

	Q.setZero();
	Q(0,0) = 0.002857541f;
	Q(1,1) = 0.0022096f;
	Q(2,2) = 0.0022096f;

	Qv.setZero();
	Qv(0,0) = 3.44048681e-06f;
	Qv(1,1) = 2.82211659e-06f;
	Qv(2,2) = 9.75675349e-04f;
}