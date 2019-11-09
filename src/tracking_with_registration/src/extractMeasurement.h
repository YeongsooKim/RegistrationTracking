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
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <sensor_msgs/PointCloud2.h>
#include "cluster.hpp"
#include "obstacle_tracking.hpp"
#include "tools.h"
#include "Eigen/Dense"


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
	ros::Publisher m_pub_shape;
	ros::Publisher m_pub_resultICP;
	ros::Publisher m_pub_shapeICP;
	ros::Publisher m_pub_Origin;

	// param
	double m_fMarkerDuration;
	double m_dClusterErrRadius;
	double m_dClusterMinSize;
	double m_dClusterMaxSize; 	

	unsigned int m_measurementN;
	unsigned int m_maxIndexNumber;

	bool m_bDoICP;

	std::vector<VectorXd> m_rgvOnlyBoundingbox;
	std::vector<VectorXd> m_rgvRegistrationAccum;
	std::vector<VectorXd> m_vecResultRMSE;

	std::vector<RGB> m_globalRGB;
	std::vector<clusterPtr> m_OriginalClusters;
	std::vector<clusterPtr> m_vecVehicleAccumulatedCloud;
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> m_vecVehicleTrackingClouds;

	ObstacleTracking m_ObstacleTracking;

	public:
	Tools m_tools;

	visualization_msgs::Marker m_Origin;
	visualization_msgs::MarkerArray m_arrShapes;
	visualization_msgs::MarkerArray m_arrShapesICP;

	std::vector<std::ofstream> vecOf_measurementCSV;
	std::vector<std::ofstream> vecOf_accumMeasurementCSV;

	ExtractMeasurement (unsigned int size);
	void setParam ();
	void downsample (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& pDownsampledCloud, float f_paramLeafSize_m);
	void downsample (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pDownsampledCloud, float f_paramLeafSize_m);
	void loadPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic, long long timestamp, bool doVisualize);
	void dbscan (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices);
	void generateColor(size_t indexNumber);
	void setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, const pcl::PointCloud<pcl::PointXYZ>::Ptr pInputCloud, long long timestamp);
	void association (long long timestamp);
	void point2pointICP (long long timestamp);
	void displayShape ();
	void publish ();
	void savePCD (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud);
	void calculateRMSE (const std::vector<VectorXd>& reference);
};


#endif //EXTRACTMEASUREMENT
