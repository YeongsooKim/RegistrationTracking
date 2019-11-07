#include "extractMeasurement.h"

ExtractMeasurement::ExtractMeasurement()
{
	// define publisher
	m_pub_result = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("output", 100);
	m_pub_shape = nh.advertise<visualization_msgs::MarkerArray>("Shape", 1);
	//m_pub_Origin = nh.advertise<visualization_msgs::Marker> ("Origin", 1);

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
	euclideanCluster.setClusterTolerance (1.5);
	euclideanCluster.setMinClusterSize (50);
	euclideanCluster.setMaxClusterSize (23000);
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

void ExtractMeasurement::association()
{
	m_ObstacleTracking.association(m_OriginalClusters);
}


void ExtractMeasurement::displayShape ()//const std::vector<clusterPtr> pVecClusters)
{
//	// origin
//	m_Origin.header.frame_id = "map";
//	m_Origin.header.stamp = ros::Time::now();
//
//	m_Origin.ns = "/origin";
//	m_Origin.id = 0;
//
//	m_Origin.type = visualization_msgs::Marker::SPHERE;
//
//	m_Origin.action = visualization_msgs::Marker::ADD;
//
//	m_Origin.pose.position.x = 0;
//	m_Origin.pose.position.y = 0;
//	m_Origin.pose.position.z = 0;
//	m_Origin.pose.orientation.x = 0.0;
//	m_Origin.pose.orientation.y = 0.0;
//	m_Origin.pose.orientation.z = 0.0;
//	m_Origin.pose.orientation.w = 1.0;
//
//	m_Origin.scale.x = 0.5;
//	m_Origin.scale.y = 0.5;
//	m_Origin.scale.z = 0.5;
//
//	m_Origin.color.r = 0.0f;
//	m_Origin.color.g = 1.0f;
//	m_Origin.color.b = 0.0f;
//	m_Origin.color.a = 1.0;
//
//	m_Origin.lifetime = ros::Duration();

	// tracking objects
	m_arrShapes.markers.clear();

	for (auto pCluster : m_ObstacleTracking.m_TrackingObjects)
	{
		visualization_msgs::Marker shape;

		shape.lifetime = ros::Duration(0.5);
		shape.header.frame_id = "map";
		shape.id = pCluster->m_id;

		// bounding box
		shape.type = visualization_msgs::Marker::CUBE;
		shape.action = visualization_msgs::Marker::ADD;
		shape.ns = "/BoundingBox";

		shape.pose.position = pCluster->m_center.position;
		shape.pose.orientation = pCluster->m_center.orientation;

		shape.scale.x = pCluster->m_dimensions.x;
		shape.scale.y = pCluster->m_dimensions.y;
		shape.scale.z = pCluster->m_dimensions.z;

		shape.color.r = pCluster->m_r/255.0f;
		shape.color.g = pCluster->m_g/255.0f;
		shape.color.b = pCluster->m_b/255.0f;
		shape.color.a = 0.5;

		m_arrShapes.markers.push_back(shape);

		// text
		shape.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		shape.ns = "/Text";

		shape.points.clear();
		shape.pose.position = pCluster->m_center.position;
		shape.pose.orientation = pCluster->m_center.orientation;

		shape.scale.x = 1.0;
		shape.scale.y = 1.0;
		shape.scale.z = 1.0;

		shape.color.r = shape.color.g = shape.color.b = 1.0;
		shape.color.a = 1.0;

		shape.text = std::to_string(pCluster->m_id);

		m_arrShapes.markers.push_back (shape);
	}
}

void ExtractMeasurement::publish ()
{
	// Accumulate all cluster to pAccumulationCloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pAccumulationCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pAccumulationCloud->header.frame_id = "map";

	// accumulation for publish
	for (const auto& pCluster : m_ObstacleTracking.m_TrackingObjects)
		*pAccumulationCloud += *(pCluster->GetCloud());

	// publish
	m_pub_result.publish (*pAccumulationCloud);
	m_pub_shape.publish (m_arrShapes);
}
