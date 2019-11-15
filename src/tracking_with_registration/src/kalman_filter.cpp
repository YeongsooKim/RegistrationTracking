#include "kalman_filter.h"
#define PI 3.14159265

#include <iostream>
using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() 
{
	time_us_ = 0.0;
	std_laspx_ = 0.15;
	noise_ax = 9.0;
	noise_ay = 9.0;

	x_ = VectorXd (4, 1);

	P_ = MatrixXd (4, 4);
	P_ << 1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1000, 0,
		  0, 0, 0, 1000;
//
	F_ = MatrixXd (4, 4);
	F_ << 1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1, 0,
		  0, 0, 0, 1;

	H_ = MatrixXd (2, 4);
	H_ << 1, 0, 0, 0,
		  0, 1, 0, 0;

	R_ = MatrixXd (2, 2);
	R_ << std_laspx_*std_laspx_, 0,
		  0, std_laspx_*std_laspx_;
	Q_ = MatrixXd (4, 4);
	I_ = Eigen::MatrixXd::Identity(4,4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::ProcessMeasurement(MeasurementPackage meas_package) 
{
	if(!m_bIsInitialized) 
	{
		x_ << meas_package.raw_measurements_[0],
			  meas_package.raw_measurements_[1], 
			  0, 
			  0;
		m_bIsInitialized = true;
		return;
	}

	//.. Calculate dt
	double dt = (meas_package.timestamp_ - time_us_)/1000000.0;
	time_us_ = meas_package.timestamp_;

	F_ (0, 2) = dt;
	F_ (1, 3) = dt;

    double dt2 = dt*dt;
    double dt3 = dt2*dt;
    double dt4 = dt3*dt;
    double dt4over4 = dt4/4.;
    double dt3over2 = dt3/2.;
    Q_ << dt4over4*noise_ax,   0, dt3over2*noise_ax,                 0,
		  0,	dt4over4*noise_ay,                 0, dt3over2*noise_ay,
		  dt3over2*noise_ax,   0,      dt2*noise_ax,                 0,
		  0,   dt3over2*noise_ay,                 0,      dt2*noise_ay;

	//.. Prediction step
	Predict (dt);

	//.. Update step
	Update (meas_package.raw_measurements_);
}

void KalmanFilter::Predict(double delta_t) 
{
	// predict the state
	x_ = F_*x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
	// Update the state using Kalman Filter equations
	VectorXd y = z - H_*x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_*P_*Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_*Ht*Si;

	// New state
	x_ = x_ + ( K*y );
	P_ = ( I_ - K*H_ )*P_;
}
