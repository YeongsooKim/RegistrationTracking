#ifndef EXTRACTMEASUREMENT
#define EXTRACTMEASUREMENT

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
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
	pcl::PointCloud<pcl::PointXYZ> m_resultCloud;

	// param
	double m_fMarkerDuration;
	double m_dClusterErrRadius;
	double m_dClusterMinSize;
	double m_dClusterMaxSize; 	

	public:
	Tools m_tools;

	ExtractMeasurement();
	void setParam();
	void downsample (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& pDownsampledCloud, float f_paramLeafSize_m);

	void loadPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic, long long timestamp, bool doVisualize);
	void dbscan (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices);
	//		void setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, std::vector<clusterPtr>& pOriginalClusters, const PointCloudXYZ::Ptr pInputCloud);
	//		void generateColor(size_t indexNumber);
	//		void displayShape (const std::vector<clusterPtr> pVecClusters);
	//		void setDetectedObject (const std::vector<clusterPtr>& pVecClusters);
	void publish();
};


#endif //EXTRACTMEASUREMENT
