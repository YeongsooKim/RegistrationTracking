#ifndef EXTRACTMEASUREMENT
#define EXTRACTMEASUREMENT

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>
#include "tools.h"

class ExtractMeasurement
{
	private:
	// Declare nodehandler
	ros::NodeHandle nh;

	// Declare publisher
	ros::Publisher m_pub_result;
//	ros::Publisher pub_shape;
//	ros::Publisher pub_detectedObject;
//	ros::Publisher pub_Origin;
//
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pResultCloud;

	public:
	Tools m_tools;
	ExtractMeasurement();
	void loadPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic, long long timestamp, bool doVisualize);
//		void dbscan (const PointCloudXYZ::ConstPtr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices);
//		void setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, std::vector<clusterPtr>& pOriginalClusters, const PointCloudXYZ::Ptr pInputCloud);
//		void generateColor(size_t indexNumber);
//		void displayShape (const std::vector<clusterPtr> pVecClusters);
//		void setDetectedObject (const std::vector<clusterPtr>& pVecClusters);
	void publish();
};


#endif //EXTRACTMEASUREMENT
