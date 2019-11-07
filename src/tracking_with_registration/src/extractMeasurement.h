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
#include "cluster.hpp"
#include "obstacle_tracking.hpp"
#include "tools.h"

typedef struct _rgb RGB;
struct _rgb
{
        uint8_t m_r;
        uint8_t m_g;
        uint8_t m_b;

        _rgb ()
        { }

        _rgb (uint8_t r, uint8_t g, uint8_t b)
        {
                m_r = r;
                m_g = g;
                m_b = b;
        }
};


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

	std::vector<RGB> m_globalRGB;
	int m_maxIndexNumber = 0;

	std::vector<clusterPtr> m_OriginalClusters;
	ObstacleTracking m_ObstacleTracking;

	public:
	Tools m_tools;

	ExtractMeasurement();
	void setParam();
	void downsample (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& pDownsampledCloud, float f_paramLeafSize_m);
	void loadPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic, long long timestamp, bool doVisualize);
	void dbscan (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices);
	void generateColor(size_t indexNumber);
	void setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, const pcl::PointCloud<pcl::PointXYZ>::Ptr pInputCloud);

	//		void generateColor(size_t indexNumber);
	//		void displayShape (const std::vector<clusterPtr> pVecClusters);
	//		void setDetectedObject (const std::vector<clusterPtr>& pVecClusters);
	void publish();
};


#endif //EXTRACTMEASUREMENT
