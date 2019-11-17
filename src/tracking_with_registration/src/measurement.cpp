/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors


#include "highway.h"
#include "extractMeasurement.h"
#include <fstream>
#include <string>
#include <iostream>

int main(int argc, char** argv)
{
	ros::init (argc, argv, "extract_measurement");

	// set camera position and angle
	Highway highway;
	ExtractMeasurement measurement(highway.traffic.size());

	measurement.setParam();

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;
	
	char d;
	d = getchar();

	while (frame_count < (frame_per_sec*sec_interval))
	{
		highway.stepHighway(egoVelocity,time_us, frame_per_sec);

		measurement.setData (highway.getGroundTruth(), time_us);
		measurement.process ();

		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;

		char c;
		while ((c = getchar()) != '\n' && c != EOF) { }
		ROS_INFO_STREAM ("end iteration");
	}

	measurement.m_myTools.plotting();

	for (unsigned int trafficIndex = 0; trafficIndex < highway.traffic.size(); trafficIndex++)
	{
		highway.vecOf_refCSV[trafficIndex].close();
		measurement.vecOf_measurementCSV[trafficIndex].close();
		measurement.vecOf_accumMeasurementCSV[trafficIndex].close();
	}

	return 0;
}
