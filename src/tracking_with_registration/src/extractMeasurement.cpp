#include "extractMeasurement.h"


void ExtractMeasurement::loadPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic, long long timestamp, bool doVisualize)
{
	if(doVisualize)
	{
		pCloudTraffic = m_tools.loadPcd("../src/sensors/data/pcd/highway_"+std::to_string(timestamp)+".pcd");
	}
}



//
//	void dbscan (long long timestamp)
//
//		if(visualize_pcd)
//		{
//			pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = tools.loadPcd("../src/sensors/data/pcd/highway_"+std::to_string(timestamp)+".pcd");
//			renderPointCloud(viewer, trafficCloud, "trafficCloud", Color((float)184/256,(float)223/256,(float)252/256));
//		}






//void Tracker::dbscan(const PointCloudXYZI::ConstPtr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices)
//{
//	//////////////////////////////////////////////////////////////////
//	// DBSCAN
//
//	// Creating the KdTree object for the search method of the extraction
//	pcl::search::KdTree<pcl::PointXYZI>::Ptr pKdtreeDownsampledCloud (new pcl::search::KdTree<pcl::PointXYZI>);
//	pKdtreeDownsampledCloud->setInputCloud (pInputCloud);
//
//	// DBSCAN object and parameter setting
//	// Tolerance is the length from core point to query point
//	// Min cluster size is the minimum number of points in the circle with the tolerance as radius
//	// Max cluster size is the maximum number of points in the circle with the tolerance as radius
//	// extract the index of each cluster to vecClusterIndices
//	pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclideanCluster;
//	euclideanCluster.setClusterTolerance (m_dClusterErrRadius);
//	euclideanCluster.setMinClusterSize (m_dClusterMinSize);
//	euclideanCluster.setMaxClusterSize (m_dClusterMaxSize);
//	euclideanCluster.setSearchMethod (pKdtreeDownsampledCloud);
//	euclideanCluster.setInputCloud (pInputCloud);
//	euclideanCluster.extract (vecClusterIndices);
//}
//
//
