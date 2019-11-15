#ifndef __MYTOOLS_HPP__
#define __MYTOOLS_HPP__

#define _USE_MATH_DEFINES
#include <vector>
#include <cmath>
#include <iostream>
#include "matplotlibcpp.h"
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace plt = matplotlibcpp;

class MyTools
{
	std::vector<long long> m_times_s;
	std::vector<VectorXd> m_vecVecXdOnlyBoundingBoxRMSE;
	std::vector<VectorXd> m_vecVecXdAccumulationRMSE;
	std::vector<VectorXd> m_vecVecXdKalmanFilterRMSE;
	std::vector<double> m_vecD_OnlyBoxDistanceRMSE;
	std::vector<double> m_vecD_AccumulationDistanceRMSE;
	std::vector<double> m_vecD_KalmanFilterDistanceRMSE;

	public:
	MyTools();
	void setTime(long long time);
	void setOnlyBoundingBoxRMSE (VectorXd inputVectorXd);
	void setAccumulationRMSE (VectorXd inputVectorXd);
	void setKalmanFilterRMSE (VectorXd inputVectorXd);
	void setOnlyBoxDistanceRMSE (double inputDistance);
	void setAccumulationDistanceRMSE (double inputDistance);
	void setKalmanFilterDistanceRMSE (double inputDistance);
	void plotting();
};


#endif // __MYTOOLS_HPP__
