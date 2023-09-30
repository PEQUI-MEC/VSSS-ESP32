#include <UKF.h>
#include <MatrixFunctions>

UKF::UKF() {
	x.setZero();
	COV.setIdentity();
	COV = COV * 0.1;
	X.setZero();
}

float UKF::get_weight(int i, bool m) {
	if (i == 0 && m) return weight0_m;
	else if (i == 0) return weight0_c;
	else return weight;
}

void UKF::set_sigma_points(const T::PoseVec &x) {
	const T::PoseMat temp = (L + lambda) * COV;
	const T::PoseMat sqrt_mat = temp.sqrt();
//		const T::PoseMat &sqrt_mat = temp;
	X.col(0) = x;
	for (int i = 1; i <= L; i++)
		X.col(i) = x + sqrt_mat.row(i - 1).transpose();
	for (int i = L + 1; i <= 2 * L; i++)
		X.col(i) = x - sqrt_mat.row(i - L - 1).transpose();
}

void UKF::predict(const T::ControlVec &controls, float time) {
//		Predicted sigma points
	set_sigma_points(x);
	for (int i = 0; i <= 2 * L; i++)
		X.col(i) = model.prediction(X.col(i), controls, time);
//		Predicted pose
	T::PoseVec x_predicted;
	x_predicted.setZero();
	for (int i = 0; i <= 2 * L; i++)
		x_predicted += get_weight(i, true) * X.col(i);
//		Predicted covariance
	T::PoseMat COV_predicted;
	COV_predicted.setZero();
	for (int i = 0; i <= 2 * L; i++) {
		T::PoseVec error = X.col(i) - x_predicted;
		COV_predicted += get_weight(i, false) * error * error.transpose();
	}
//		Update pose and covariance
	x_predicted(2, 0) = wrap(x_predicted(2, 0));
	x = x_predicted;
	COV = COV_predicted + model.R;
}

void UKF::update_on_sensor_data(const T::SensorVec &data) {
//		Predicted measurement sigma points
	set_sigma_points(x);
	T::UKFSensorSigmaMat Y;
	for (int i = 0; i <= 2 * L; i++)
		Y.col(i) = model.sensor_measurement_model(X.col(i));
//		Predicted measurement
	T::SensorVec y_predicted;
	y_predicted.setZero();
	for (int i = 0; i <= 2 * L; i++)
		y_predicted += get_weight(i, true) * Y.col(i);
//		Predicted covariances
	T::SensorMat COV_YY;
	T::KSensorMat COV_XY;
	COV_YY.setZero();
	COV_XY.setZero();
	for (int i = 0; i <= 2 * L; i++) {
		T::SensorVec y_error = Y.col(i) - y_predicted;
		T::PoseVec x_error = X.col(i) - x;
		COV_YY += get_weight(i, false) * y_error * y_error.transpose();
		COV_XY += get_weight(i, false) * x_error * y_error.transpose();
	}
	COV_YY += model.Q;
//		Update pose and covariance
	T::KSensorMat K_GAIN = COV_XY * COV_YY.inverse();
	T::SensorVec error = data - y_predicted;
	// error(0, 0) = wrap(error(0, 0));
//		error(0, 0) = 0;
//		error(2, 0) = 0;
//		error(3, 0) = 0;
	x = x + K_GAIN * error;
	COV = COV - K_GAIN * COV_YY * K_GAIN.transpose();
}

void UKF::update_on_vision_data(const T::VisionVec &data) {
//		Predicted measurement sigma points
	set_sigma_points(x);
	T::UKFVisionSigmaMat Y;
	for (int i = 0; i <= 2 * L; i++)
		Y.col(i) = model.vision_measurement_model(X.col(i));
//		Predicted measurement
	T::VisionVec y_predicted;
	y_predicted.setZero();
	for (int i = 0; i <= 2 * L; i++)
		y_predicted += get_weight(i, true) * Y.col(i);
//		Predicted covariances
	T::VisionMat COV_YY;
	T::KVisionMat COV_XY;
	COV_YY.setZero();
	COV_XY.setZero();
	for (int i = 0; i <= 2 * L; i++) {
		T::VisionVec y_error = Y.col(i) - y_predicted;
		T::PoseVec x_error = X.col(i) - x;
		COV_YY += get_weight(i, false) * y_error * y_error.transpose();
		COV_XY += get_weight(i, false) * x_error * y_error.transpose();
	}
	COV_YY += model.Qv;
//		Update pose and covariance
	T::KVisionMat K_GAIN = COV_XY * COV_YY.inverse();
	T::VisionVec error = data - y_predicted;

	x_error = error(0, 0);
	y_error = error(1, 0);
	theta_error = wrap(error(2, 0));
	new_log = true;

	error(2, 0) = wrap(error(2, 0));
	error(3, 0) = wrap(error(3, 0));
	x = x + K_GAIN * error;
	COV = COV - K_GAIN * COV_YY * K_GAIN.transpose();
}
