/* \author Aaron Brown */
// Handle logic for creating traffic on highway and animating it

#include <ros/ros.h>
#include "sensors/lidar.h"
#include "tools.h"

class Highway
{
public:

	std::vector<Car> traffic;
	Car egoCar;
	Tools tools;
	std::vector<std::ofstream> vecOf_refCSV;
	std::vector<VectorXd> m_vGroundTruth;

	// Parameters 
	// --------------------------------
	// Set which cars to track with UKF
	std::vector<bool> trackCars = {true,true,true};
	// Visualize sensor measurements
	// Predict path in the future using UKF
	double projectedTime = 0;
	int projectedSteps = 0;
	// --------------------------------

	Highway()
	{
		tools = Tools();
	
		egoCar = Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");
		
		Car car1(Vect3(-10, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");
		
		std::vector<accuation> car1_instructions;
		accuation a = accuation(0.5*1e6, 0.5, 0.0);
		car1_instructions.push_back(a);
		a = accuation(2.2*1e6, 0.0, -0.2);
		car1_instructions.push_back(a);
		a = accuation(3.3*1e6, 0.0, 0.2);
		car1_instructions.push_back(a);
		a = accuation(4.4*1e6, -2.0, 0.0);
		car1_instructions.push_back(a);
	
		car1.setInstructions(car1_instructions);
		if( trackCars[0] )
		{
			UKF ukf1;
			car1.setUKF(ukf1);
		}
		traffic.push_back(car1);
		
		Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");
		std::vector<accuation> car2_instructions;
		a = accuation(4.0*1e6, 3.0, 0.0);
		car2_instructions.push_back(a);
		a = accuation(8.0*1e6, 0.0, 0.0);
		car2_instructions.push_back(a);
		car2.setInstructions(car2_instructions);
		if( trackCars[1] )
		{
			UKF ukf2;
			car2.setUKF(ukf2);
		}
		traffic.push_back(car2);
	
		Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");
		std::vector<accuation> car3_instructions;
		a = accuation(0.5*1e6, 2.0, 1.0);
		car3_instructions.push_back(a);
		a = accuation(1.0*1e6, 2.5, 0.0);
		car3_instructions.push_back(a);
		a = accuation(3.2*1e6, 0.0, -1.0);
		car3_instructions.push_back(a);
		a = accuation(3.3*1e6, 2.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(4.5*1e6, 0.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(5.5*1e6, -2.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(7.5*1e6, 0.0, 0.0);
		car3_instructions.push_back(a);
		car3.setInstructions(car3_instructions);
		if( trackCars[2] )
		{
			UKF ukf3;
			car3.setUKF(ukf3);
		}
		traffic.push_back(car3);

		vecOf_refCSV.resize(traffic.size());

		for (unsigned int trafficIndex = 0; trafficIndex < traffic.size(); trafficIndex++)
		{
			string num = std::to_string(trafficIndex);
			vecOf_refCSV[trafficIndex].open ("reference_state_" + num + ".csv");

			if (vecOf_refCSV[trafficIndex].is_open()){
				vecOf_refCSV[trafficIndex] << "timestamp, pose_x, pose_y, velocity_x, velocity_y, angle" << std::endl;
			}
		}
	}

	~Highway()
	{
	}

	
	void stepHighway(double egoVelocity, long long timestamp, int frame_per_sec)
	{

		for (int i = 0; i < traffic.size(); i++)
		{
			traffic[i].move((double)1/frame_per_sec, timestamp);
			// Sense surrounding cars with lidar and radar
			if(trackCars[i])
			{
				vecOf_refCSV[i] << timestamp/1e6 << "," << traffic[i].position.x << "," << traffic[i].position.y << "," << traffic[i].velocity*cos(traffic[i].angle) << "," << traffic[i].velocity*sin(traffic[i].angle) << "," << traffic[i].angle << std::endl;


				VectorXd gt(4);
				gt << traffic[i].position.x, traffic[i].position.y, traffic[i].velocity*cos(traffic[i].angle), traffic[i].velocity*sin(traffic[i].angle);
				tools.ground_truth.push_back(gt);
				tools.lidarSense(traffic[i], timestamp);
				tools.radarSense(traffic[i], egoCar, timestamp);
				tools.ukfResults(traffic[i], projectedTime, projectedSteps);
				VectorXd estimate(4);
				double v  = traffic[i].ukf.x_(2);
    			double yaw = traffic[i].ukf.x_(3);
    			double v1 = cos(yaw)*v;
    			double v2 = sin(yaw)*v;
				estimate << traffic[i].ukf.x_[0], traffic[i].ukf.x_[1], v1, v2;
				tools.estimations.push_back(estimate);
	
			}
			if (i == 0)
			{
				VectorXd gt(4);
				gt << traffic[i].position.x, traffic[i].position.y, traffic[i].velocity*cos(traffic[i].angle), traffic[i].velocity*sin(traffic[i].angle);
				m_vGroundTruth.push_back(gt);
			}
		}
		VectorXd rmse = tools.CalculateRMSE(tools.estimations, tools.ground_truth);
	}

	std::vector<VectorXd> getGroundTruth()
	{
		return m_vGroundTruth;
	}
};
