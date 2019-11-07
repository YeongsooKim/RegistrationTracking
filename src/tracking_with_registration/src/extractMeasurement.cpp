#include "extractMeasurement.h"

ExtractMeasurement::ExtractMeasurement()
{
	// define publisher
	m_pub_result = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("output", 100);
	m_maxIndexNumber = 0;	
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
	euclideanCluster.setClusterTolerance (1.0);
	euclideanCluster.setMinClusterSize (50);
	euclideanCluster.setMaxClusterSize (45000);
	//	euclideanCluster.setClusterTolerance (m_dClusterErrRadius);
	//	euclideanCluster.setMinClusterSize (m_dClusterMinSize);
	//	euclideanCluster.setMaxClusterSize (m_dClusterMaxSize);
	euclideanCluster.setSearchMethod (pKdtreeDownsampledCloud);
	euclideanCluster.setInputCloud (pInputCloud);
	euclideanCluster.extract (vecClusterIndices);
}

void ExtractMeasurement::setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, const pcl::PointCloud<pcl::PointXYZ>::Ptr pInputCloud)
{
	m_OriginalClusters.clear();

	int objectNumber = 0;
	for (const auto& clusterIndice : vecClusterIndices)
	{
		std::string label_ = "vehicle";
		std::string label = label_;
		label.append (std::to_string(objectNumber));

		// generate random color to globalRGB variable
		generateColor(vecClusterIndices.size());

		// pCluster is a local cluster
		clusterPtr pCluster (new Cluster());

		std_msgs::Header dummy;
		dummy.frame_id = "map";
		// Cloring and calculate the cluster center point and quaternion
		pCluster->SetCloud(pInputCloud, clusterIndice.indices, dummy, objectNumber, m_globalRGB[objectNumber].m_r, m_globalRGB[objectNumber].m_g, m_globalRGB[objectNumber].m_b, label, true);

		m_OriginalClusters.push_back(pCluster);

		objectNumber++;
	}
}


// generate random color
void ExtractMeasurement::generateColor(size_t indexNumber)
{
	if (indexNumber > m_maxIndexNumber)
	{
		int addRGB = indexNumber - m_maxIndexNumber;
		m_maxIndexNumber = indexNumber;

		for (size_t i = 0; i < addRGB; i++)
		{
			uint8_t r = 1024 * rand () % 255;
			uint8_t g = 1024 * rand () % 255;
			uint8_t b = 1024 * rand () % 255;
			m_globalRGB.push_back(RGB(r, g, b));
		}
	}
}

void ExtractMeasurement::associate()
{
	m_ObstacleTracking.associate(m_OriginalClusters);
}



void ExtractMeasurement::publish ()
{
	// Accumulate all cluster to pAccumulationCloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pAccumulationCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pAccumulationCloud->header.frame_id = "map";

//	// accumulation for publish
//	for (const auto& pCluster : m_OriginalClusters)
//		*pAccumulationCloud += *(pCluster->GetCloud());

	*pAccumulationCloud += *(m_OriginalClusters[0]->GetCloud());


	// publish
	m_pub_result.publish (*pAccumulationCloud);
}
