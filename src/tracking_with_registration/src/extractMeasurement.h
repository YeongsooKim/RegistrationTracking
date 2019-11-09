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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <vector>
#include "cluster.hpp"
#include "obstacle_tracking.hpp"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
	ros::Publisher m_pub_resultICP;
	ros::Publisher m_pub_shape;
	ros::Publisher m_pub_shapeICP;
	ros::Publisher m_pub_shapeReference;

	// param
	double m_fMarkerDuration;
	double m_dClusterErrRadius;
	double m_dClusterMinSize;
	double m_dClusterMaxSize; 	

	unsigned int m_measurementN;
	unsigned int m_maxIndexNumber;

	bool m_bDoICP;
	bool m_bDoVisualize;
	long long m_llTimestamp_s;

	std::vector<VectorXd> m_vecVecXdOnlyBoundingbox;
	std::vector<VectorXd> m_vecVecXdRegistrationAccum;
	std::vector<VectorXd> m_vecVecXdResultRMSE;
	std::vector<VectorXd> m_vecVecXdRef;

	std::vector<RGB> m_globalRGB;
	std::vector<clusterPtr> m_OriginalClusters;
	std::vector<clusterPtr> m_vecVehicleAccumulatedCloud;
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> m_vecVehicleTrackingClouds;

	ObstacleTracking m_ObstacleTracking;
	geometry_msgs::PoseArray m_geomsgReferences;

	public:
	visualization_msgs::MarkerArray m_arrShapes;
	visualization_msgs::MarkerArray m_arrShapesICP;
	visualization_msgs::MarkerArray m_arrShapesReference;

	std::vector<std::ofstream> vecOf_measurementCSV;
	std::vector<std::ofstream> vecOf_accumMeasurementCSV;

	ExtractMeasurement (unsigned int size, bool bDoVisualizePCD);
	void setParam ();
	void setData (const std::vector<VectorXd>& vecVecXdRef, long long timestamp);
	void process ();
	void downsample (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& pDownsampledCloud, float f_paramLeafSize_m);
	void downsample (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pDownsampledCloud, float f_paramLeafSize_m);
	void getPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic);
	void dbscan (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices);
	void generateColor(size_t indexNumber);
	void setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, const pcl::PointCloud<pcl::PointXYZ>::Ptr pInputCloud);
	void association ();
	void point2pointICP ();
	void savePCD (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCD (std::string file);
	void calculateRMSE ();
	void displayShape ();
	void publish ();
};


#endif //EXTRACTMEASUREMENT
