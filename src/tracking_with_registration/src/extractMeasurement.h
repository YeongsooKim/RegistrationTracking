#ifndef EXTRACTMEASUREMENT


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
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <vector>
#include <algorithm>
#include "cluster.hpp"
#include "obstacle_tracking.hpp"
#include "myTools.hpp"
#include "Eigen/Dense"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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


struct pose
{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
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
		ros::Publisher m_pub_shapeKalman;
		ros::Publisher m_pub_shapeReference;

		
		// Check for visualization cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOutputCloud;
		ros::Publisher m_pub_source;
		ros::Publisher m_pub_target;
		ros::Publisher m_pub_final;
		ros::Publisher m_pub_output;

		// param
		double m_fMarkerDuration;
		double m_dClusterErrRadius;
		double m_dClusterMinSize;
		double m_dClusterMaxSize; 	

		unsigned int m_measurementN;
		unsigned int m_maxIndexNumber;

		bool m_bDoICP = false;
		bool m_bDoNDT = true;
		bool m_bDoVisualize;
		long long m_llTimestamp_s;

		std::vector<VectorXd> m_vecVecXdOnlyBoundingbox;
		std::vector<VectorXd> m_vecVecXdRegistrationAccum;
		std::vector<VectorXd> m_vecVecXdKalmanFilter;
		std::vector<VectorXd> m_vecVecXdResultRMSE;
		std::vector<VectorXd> m_vecVecXdRef;
		std::vector<VectorXd> m_vecVecXdRefwithVelo;

		std::vector<double> m_vecD_ResultDistanceRMSE;

		std::vector<RGB> m_globalRGB;
		std::vector<clusterPtr> m_OriginalClusters;
		std::vector<clusterPtr> m_vecVehicleAccumulatedCloud;

		ObstacleTracking m_ObstacleTracking;
		geometry_msgs::PoseArray m_geomsgReferences;

		// about ndt variable
		pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> m_ndt; 

		Eigen::Matrix4f m_mat4fBase2Local;
		Eigen::Matrix4f m_mat4fLocal2Base;

		pose m_previousPose;
		pose m_currentPose;
		pose m_ndtPose;
		pose m_addedPose;
		pose m_localizerPose;
		
		double m_diff_x = 0.0; 
		double m_diff_y = 0.0; 
		double m_diff_z = 0.0; 
		double m_diff_yaw;  // current_pose - previous_pose


	public:
		MyTools m_myTools;

		visualization_msgs::MarkerArray m_arrShapes;
		visualization_msgs::MarkerArray m_arrShapesICP;
		visualization_msgs::MarkerArray m_arrShapesKalman;
		visualization_msgs::MarkerArray m_arrShapesReference;

		std::vector<std::ofstream> vecOf_measurementCSV;
		std::vector<std::ofstream> vecOf_accumMeasurementCSV;
		std::vector<std::ofstream> vecOf_KalmanFilterCSV;

		ExtractMeasurement (unsigned int size);
		void setParam ();
		void setData (const std::vector<VectorXd>& vecVecXdRef, long long timestamp);
		void process ();
		void threshold (const pcl::PointCloud<pcl::PointXYZ> &inputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &pOutput);
		void downsample (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& pDownsampledCloud, float f_paramLeafSize_m);
		void downsample (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pDownsampledCloud, float f_paramLeafSize_m);
		void getPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic);
		void dbscan (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices);
		void generateColor(size_t indexNumber);
		void setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, const pcl::PointCloud<pcl::PointXYZ>::Ptr pInputCloud);
		void association ();
		//void point2pointICPwithAccumulation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputSourceCloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputTargetCloud, bool bIsFirst);
		void point2pointICPwithAccumulation (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputSourceCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputTargetCloud, bool& bIsFirst);
		void savePCD (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCD (std::string file);
		void calculateRMSE ();
		void displayShape ();
		void publish ();
		void NDT (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputSourceCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputTargetCloud, bool& bIsInitSource);
		double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
};


#endif //EXTRACTMEASUREMENT
