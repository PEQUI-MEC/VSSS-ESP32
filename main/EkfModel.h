#ifndef VSSS_EKFMODEL_H
#define VSSS_EKFMODEL_H

#include <Types.h>

const float ROBOT_SIZE = 0.0675f;

class EkfModel {
	private:
	void process_noise(float time);

	public:

//	EKF::PoseMat F;
	T::PoseMat R;

//	EKF::HSensorMat H;
	T::SensorMat Q;

//	EKF::HVisionMat Hv;
	T::VisionMat Qv;

	EkfModel();
	T::PoseVec prediction(const T::PoseVec &prev_x,
							const T::ControlVec &controls, float time);
	T::SensorVec sensor_measurement_error(const T::PoseVec &x, const T::SensorVec &z);
	T::SensorVec sensor_measurement_model(const T::PoseVec &x);
	T::VisionVec vision_measurement_error(const T::PoseVec &x, const T::VisionVec &z);
	T::VisionVec vision_measurement_model(const T::PoseVec &x);
	void use_encoders(bool use);
};

#endif //VSSS_EKFMODEL_H
