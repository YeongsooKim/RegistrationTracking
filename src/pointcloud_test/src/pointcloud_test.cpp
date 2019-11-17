#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

ros::Publisher result;

void threshold (const pcl::PointCloud<pcl::PointXYZI> &inputCloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &pOutput)
{
	// Thresholding 
	pcl::PointCloud<pcl::PointXYZI> tmpCloud;
	for (const auto& point : inputCloud.points)
	{
		pcl::PointXYZI p;
		p.x = (double)point.x;
		p.y = (double)point.y;
		p.z = (double)point.z;
		p.intensity = (double)point.intensity;

		double r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
		if ((p.y < 5.5 && p.y >1.5) && p.z > -1.25 && r < 17.0)
		{
			tmpCloud.push_back(p);
		}
	}
	*pOutput = tmpCloud;
}

void velodyneCallback (const sensor_msgs::PointCloud2::ConstPtr& pInput)
{
	// Container for input data 
	pcl::PointCloud<pcl::PointXYZI> tmpCloud;
	pcl::fromROSMsg(*pInput, tmpCloud);

	pcl::PointCloud<pcl::PointXYZI>::Ptr pThresholdedCloud (new pcl::PointCloud<pcl::PointXYZI>);
	threshold (tmpCloud, pThresholdedCloud);
	//////////////////////////////////////////////////////////////////
	
	pThresholdedCloud->header.frame_id = "velodyne";
	result.publish (*pThresholdedCloud);
}

	//	// Thresholding 
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr pThresholdedCloud (new pcl::PointCloud<pcl::PointXYZI>);
	//	threshold (pTmpCloud, pThresholdedCloud, m_dMaxScanRange_m, m_dHighThreshold_m, m_dLowThreshold_m);
	//
	//	double thresholdTime = ros::Time::now().toSec();
	//	double intervalTime = thresholdTime - iterationStartTime;	
	//	m_file << intervalTime << ";";
	//
	//	// Transform global to local and accumulate mapCloud to transformed input pointcloud only first time
	//	// Set input target only first time
	//	if (!m_bIsFirstMap)
	//	{
	//		// downsampling
	//		pcl::PointCloud<pcl::PointXYZI>::Ptr pDownsampledCloud (new pcl::PointCloud<pcl::PointXYZI>);
	//		downsample (pThresholdedCloud, pDownsampledCloud, m_dLeafSize_m);
	//
	//		double downsampleTime = ros::Time::now().toSec();
	//		intervalTime = downsampleTime - thresholdTime;
	//		m_file << intervalTime << ";";
	//
	//		// set NDT parameter
	//		setNDTParam(pDownsampledCloud);
	//
	//		double setNDTParamTime = ros::Time::now().toSec();
	//		intervalTime = setNDTParamTime - downsampleTime;
	//		m_file << intervalTime << ";";
	//
	//		// ndt align and get result
	//		getResultParam();
	//
	//		double getResultTime = ros::Time::now().toSec();
	//		intervalTime = getResultTime - setNDTParamTime;
	//		m_file << intervalTime << ";";
	//
	//		// update previous pose, current pose, added pose, map, ndt target
	//		update (pThresholdedCloud);
	//	}
	//	else
	//	{
	//		// Transform thresholded cloud from base to local
	//		pcl::PointCloud<pcl::PointXYZI>::Ptr pTransformedInputCloud (new pcl::PointCloud<pcl::PointXYZI>);
	//		pcl::transformPointCloud (*pThresholdedCloud, *pTransformedInputCloud, m_tfBase2Local);
	//
	//		// Set map
	//		m_mapCloud += *pTransformedInputCloud;
	//		pcl::PointCloud<pcl::PointXYZI>::Ptr pMapCloud (new pcl::PointCloud<pcl::PointXYZI>(m_mapCloud));
	//
	//		// set NDT input target
	//		m_ndt.setInputTarget (pMapCloud);
	//
	//		m_bIsFirstMap = false;
	//	}
	//
	//	// check time interval
	//	double iterationEndTime = ros::Time::now().toSec();
	//
	//	pcl::PointCloud<pcl::PointXYZI>::Ptr pMapCloud (new pcl::PointCloud<pcl::PointXYZI>(m_mapCloud));
	//	m_pubAccumulationPointCloud.publish (pMapCloud);
	//
	//	double publishTime = ros::Time::now().toSec();
	//	intervalTime = publishTime - iterationEndTime;	
	//	m_file << intervalTime << std::endl;

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "pointcloud_test");

	// NodeHandle
	ros::NodeHandle nh;

	// subscriber
	ros::Subscriber subVelodyne = nh.subscribe ("velodyne_points", 1000, velodyneCallback);

	// publisher
	result = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud_test/modified_cloud", 1000);

	while (ros::ok())
	{
		ros::spin();
	}
}
