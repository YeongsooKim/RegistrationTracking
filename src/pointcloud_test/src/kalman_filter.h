#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

class KalmanFilter {
	bool m_bIsInitialized = false;
	long long time_us_;
	double std_laspx_;
	double noise_ax;
	double noise_ay;

	public:

	// state vector
	Eigen::VectorXd x_;

	// state covariance matrix
	Eigen::MatrixXd P_;

	// state transition matrix
	Eigen::MatrixXd F_;

	// process covariance matrix
	Eigen::MatrixXd Q_;

	// measurement matrix
	Eigen::MatrixXd H_;

	// measurement covariance matrix
	Eigen::MatrixXd R_;

	// 4x4 identity matrix
	Eigen::MatrixXd I_;

	/**
	 * Constructor
	 */
	KalmanFilter();

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter();

	void ProcessMeasurement(MeasurementPackage meas_package);
	/**
	 * Prediction Predicts the state and the state covariance
	 * using the process model
	 * @param delta_T Time between k and k+1 in s
	 */
	void Predict(double delta_t);

	/**
	 * Updates the state by using standard Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void Update(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
