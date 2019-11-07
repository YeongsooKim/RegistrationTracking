#include "extractMeasurement.h"

ExtractMeasurement::ExtractMeasurement()
{
	// define publisher
	m_pub_result = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("output", 100);
	m_resultCloud.header.frame_id = "map";
	
}

void ExtractMeasurement::setParam()
{
	// set parameter
	nh.param ("extractMeasurement/Marker_duration", m_fMarkerDuration, 0.1);
	nh.param ("extractMeasurement/cluster_err_range", m_dClusterErrRadius, 0.5);
	nh.param ("extractMeasurement/cluster_min_size", m_dClusterMinSize, 15.0);
	nh.param ("extractMeasurement/cluster_max_size", m_dClusterMaxSize, 50.0);
}


void ExtractMeasurement::downsample (const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& pDownsampledCloud, float f_paramLeafSize_m)
{
	pDownsampledCloud->clear();

	// Voxel length of the corner : fLeafSize
	pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
	voxelFilter.setInputCloud (pInputCloud);
	voxelFilter.setLeafSize(f_paramLeafSize_m, f_paramLeafSize_m, f_paramLeafSize_m);
	voxelFilter.filter (*pDownsampledCloud);
}



void ExtractMeasurement::loadPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic, long long timestamp, bool doVisualize)
{
	if(doVisualize)
	{
		pCloudTraffic = m_tools.loadPcd("/workspace/TrackingWithRegistration/src/tracking_with_registration/src/sensors/data/pcd/highway_"+std::to_string(timestamp)+".pcd");
	}

	m_resultCloud.clear();
	m_resultCloud.points = pCloudTraffic->points;
}


void ExtractMeasurement::dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices)
{
	//////////////////////////////////////////////////////////////////
	// DBSCAN

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr pKdtreeDownsampledCloud (new pcl::search::KdTree<pcl::PointXYZ>);
	pKdtreeDownsampledCloud->setInputCloud (pInputCloud);

	// DBSCAN object and parameter setting
	// Tolerance is the length from core point to query point
	// Min cluster size is the minimum number of points in the circle with the tolerance as radius
	// Max cluster size is the maximum number of points in the circle with the tolerance as radius
	// extract the index of each cluster to vecClusterIndices
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanCluster;
	euclideanCluster.setClusterTolerance (0.70);
	euclideanCluster.setMinClusterSize (50);
	euclideanCluster.setMaxClusterSize (45000);
	//	euclideanCluster.setClusterTolerance (m_dClusterErrRadius);
	//	euclideanCluster.setMinClusterSize (m_dClusterMinSize);
	//	euclideanCluster.setMaxClusterSize (m_dClusterMaxSize);
	euclideanCluster.setSearchMethod (pKdtreeDownsampledCloud);
	euclideanCluster.setInputCloud (pInputCloud);
	euclideanCluster.extract (vecClusterIndices);
}


void ExtractMeasurement::publish ()
{
	m_pub_result.publish (m_resultCloud);
}


//
//	void dbscan (long long timestamp)
//
//		if(visualize_pcd)
//		{
//			pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = tools.loadPcd("../src/sensors/data/pcd/highway_"+std::to_string(timestamp)+".pcd");
//			renderPointCloud(viewer, trafficCloud, "trafficCloud", Color((float)184/256,(float)223/256,(float)252/256));
//		}






