#include "myTools.hpp"

MyTools::MyTools() { }

void MyTools::setTime (long long time)
{
	m_times_s.push_back (time);
}

void MyTools::setOnlyBoundingBoxRMSE (VectorXd inputVectorXd)
{
	m_vecVecXdOnlyBoundingBoxRMSE.push_back (inputVectorXd);
}

void MyTools::setAccumulationRMSE (VectorXd inputVectorXd)
{
	m_vecVecXdAccumulationRMSE.push_back (inputVectorXd);  
}

void MyTools::setKalmanFilterRMSE (VectorXd inputVectorXd)
{
	m_vecVecXdKalmanFilterRMSE.push_back (inputVectorXd);
}

void MyTools::plotting()
{
	std::vector<double> times_s (m_times_s.size());
	std::vector<double> vecD_OnlyBoundingBoxRMSE_x (m_vecVecXdOnlyBoundingBoxRMSE.size());
	std::vector<double> vecD_OnlyBoundingBoxRMSE_y (m_vecVecXdOnlyBoundingBoxRMSE.size());
	std::vector<double> vecD_AccumulationRMSE_x (m_vecVecXdAccumulationRMSE.size());
	std::vector<double> vecD_AccumulationRMSE_y (m_vecVecXdAccumulationRMSE.size());
	std::vector<double> vecD_KalmanFilterRMSE_x (m_vecVecXdKalmanFilterRMSE.size());
	std::vector<double> vecD_KalmanFilterRMSE_y (m_vecVecXdKalmanFilterRMSE.size());
	std::vector<double> vecD_KalmanFilterRMSE_vx (m_vecVecXdKalmanFilterRMSE.size());
	std::vector<double> vecD_KalmanFilterRMSE_vy (m_vecVecXdKalmanFilterRMSE.size());

	for (unsigned int stepIndex = 0; stepIndex < m_times_s.size(); stepIndex++)
	{
		times_s.at(stepIndex) = m_times_s.at(stepIndex)/1e6;
		vecD_OnlyBoundingBoxRMSE_x.at (stepIndex) = m_vecVecXdOnlyBoundingBoxRMSE.at(stepIndex)[0];
		vecD_OnlyBoundingBoxRMSE_y.at (stepIndex) = m_vecVecXdOnlyBoundingBoxRMSE.at(stepIndex)[1];
		vecD_AccumulationRMSE_x.at (stepIndex) = m_vecVecXdAccumulationRMSE.at(stepIndex)[0];
		vecD_AccumulationRMSE_y.at (stepIndex) = m_vecVecXdAccumulationRMSE.at(stepIndex)[1];

		vecD_KalmanFilterRMSE_x.at (stepIndex) = m_vecVecXdKalmanFilterRMSE.at(stepIndex)[0];
		vecD_KalmanFilterRMSE_y.at (stepIndex) = m_vecVecXdKalmanFilterRMSE.at(stepIndex)[1];
		vecD_KalmanFilterRMSE_vx.at (stepIndex) = m_vecVecXdKalmanFilterRMSE.at(stepIndex)[2];
		vecD_KalmanFilterRMSE_vy.at (stepIndex) = m_vecVecXdKalmanFilterRMSE.at(stepIndex)[3];
	}


	plt::figure_size (1200, 780);
    plt::suptitle("RMSE over time");
    // Set the "super title"
    plt::subplot(2, 1, 1);
    plt::title("x RMSE over time");
	plt::plot(times_s, vecD_OnlyBoundingBoxRMSE_x, "r-");
	plt::plot(times_s, vecD_AccumulationRMSE_x, "b-");
	plt::plot(times_s, vecD_KalmanFilterRMSE_x, "k");
    plt::named_plot("OnlyBox", times_s, vecD_OnlyBoundingBoxRMSE_x);
    plt::named_plot("Accumulation", times_s, vecD_AccumulationRMSE_x);
    plt::named_plot("Kalman filter", times_s, vecD_KalmanFilterRMSE_x);
	plt::legend();
	plt::ylabel ("RMSE (m)");

    plt::subplot(2, 1, 2);
    plt::title("y RMSE over time");
	plt::plot(times_s, vecD_OnlyBoundingBoxRMSE_y, "r-");
	plt::plot(times_s, vecD_AccumulationRMSE_y, "b-");
	plt::plot(times_s, vecD_KalmanFilterRMSE_y, "k");
    plt::named_plot("OnlyBox", times_s, vecD_OnlyBoundingBoxRMSE_y);
    plt::named_plot("Accumulation", times_s, vecD_AccumulationRMSE_y);
    plt::named_plot("Kalman filter", times_s, vecD_KalmanFilterRMSE_y);
	plt::legend();
	plt::xlabel ("time (s)");
	plt::ylabel ("RMSE (m)");


	// Show plots
	plt::show();
}
