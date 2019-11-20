#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "extractMeasurement.h"

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pointcloud_test");

	ExtractMeasurement measurement;

	while (ros::ok())
	{
		ros::spin();
	}
}
