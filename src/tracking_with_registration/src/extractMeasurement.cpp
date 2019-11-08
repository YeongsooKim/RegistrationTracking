#include "extractMeasurement.h"

ExtractMeasurement::ExtractMeasurement(unsigned int size) : m_measurementN (size)
{
	// define publisher
	m_pub_result = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("output", 100);
	m_pub_shape = nh.advertise<visualization_msgs::MarkerArray>("Shape", 1);
	//m_pub_Origin = nh.advertise<visualization_msgs::Marker> ("Origin", 1);

	m_maxIndexNumber = 0;	

	vecOf_measurementCSV.resize (m_measurementN);
	vecOf_accumMeasurementCSV.resize (m_measurementN);

	for (unsigned int measurementIndex = 0; measurementIndex < m_measurementN; measurementIndex++)
	{
		string num = std::to_string(measurementIndex);

		vecOf_measurementCSV[measurementIndex].open ("measurement_" + num + ".csv");
		if (vecOf_measurementCSV[measurementIndex].is_open()){
			vecOf_measurementCSV[measurementIndex] << "timestamp, pose_x, pose_y" << std::endl;
		}

		vecOf_accumMeasurementCSV[measurementIndex].open ("accumulation_measurement_" + num + ".csv");
		if (vecOf_accumMeasurementCSV[measurementIndex].is_open()){
			vecOf_accumMeasurementCSV[measurementIndex] << "timestamp, pose_x, pose_y" << std::endl;
		}
	}

	m_maxIndexNumber = 0;
	m_bDoICP = true;

	m_vecVehicleTrackingClouds.resize(m_measurementN);
	for (unsigned int measurementIndex = 0; measurementIndex < m_measurementN; measurementIndex++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		m_vecVehicleAccumulatedCloud.push_back(pPointCloud);
	}
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
	// Voxel length of the corner : fLeafSize
	pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
	voxelFilter.setInputCloud (pInputCloud);
	voxelFilter.setLeafSize(f_paramLeafSize_m, f_paramLeafSize_m, f_paramLeafSize_m);
	voxelFilter.filter (*pDownsampledCloud);
}

void ExtractMeasurement::downsample (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pDownsampledCloud, float f_paramLeafSize_m)
{
	// Voxel length of the corner : fLeafSize
	pcl::VoxelGrid<pcl::PointXYZRGB> voxelFilter;
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

void ExtractMeasurement::setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, const pcl::PointCloud<pcl::PointXYZ>::Ptr pInputCloud, long long timestamp)
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
		dummy.stamp = ros::Time(timestamp/1e6);

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

void ExtractMeasurement::association(long long timestamp)
{
	static bool bIsFirst = true;
	m_ObstacleTracking.association(m_OriginalClusters);

	// store data into vector of vehicles tracking cloud
	for (unsigned int vehicleIndex = 0; vehicleIndex < m_measurementN; vehicleIndex++)
	{
		for (unsigned int objectIndex = 0; objectIndex < m_measurementN; objectIndex++)
		{
			if ((m_ObstacleTracking.m_TrackingObjects[objectIndex]->m_id-1) == vehicleIndex)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTrackingCloud (m_ObstacleTracking.m_TrackingObjects[objectIndex]->GetCloud());
				m_vecVehicleTrackingClouds[vehicleIndex].push_back (pTrackingCloud);
				
				break;
			}
		}
	}

	if (m_bDoICP)
	{
		if (bIsFirst)
		{
			for (unsigned int vehicleIndex = 0; vehicleIndex < m_measurementN; vehicleIndex++)
			{
				*m_vecVehicleAccumulatedCloud[vehicleIndex] += *m_vecVehicleTrackingClouds[vehicleIndex].back();
				extractCenterPoint (timestamp, m_vecVehicleAccumulatedCloud[vehicleIndex], vehicleIndex);
			}

			bIsFirst = false;
		}
		else
		{
			point2pointICP(timestamp);

		}
	}
}

void ExtractMeasurement::point2pointICP(long long timestamp)
{
	// Accumulate all cluster to pAccumulationCloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pAccumulationCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pAccumulationCloud->header.frame_id = "map";

	unsigned int vehicleN = 0;
	for (const auto& vecVehicleTrackingCloud : m_vecVehicleTrackingClouds)
	{
		ROS_INFO_STREAM ("-------------------------------------");
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pSourceCloud (m_vecVehicleAccumulatedCloud[vehicleN]);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTargetCloud (vecVehicleTrackingCloud.back());

		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		icp.setInputSource (pSourceCloud);
		icp.setInputTarget (pTargetCloud);
		pcl::PointCloud<pcl::PointXYZRGB> Final;
		icp.align (Final);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pSourceCloudChangeRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTargetCloudChangeRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB> FinalChangeRGB;

		for (auto point : pSourceCloud->points)
		{
			pcl::PointXYZRGB tmp;
			tmp.x = point.x;
			tmp.y = point.y;
			tmp.z = point.z;
			
			tmp.r = 255;
			tmp.g = 0;
			tmp.b = 0;
			pSourceCloudChangeRGB->points.push_back(tmp);
		}
		for (auto point : pTargetCloud->points)
		{
			pcl::PointXYZRGB tmp;
			tmp.x = point.x;
			tmp.y = point.y;
			tmp.z = point.z;
			
			tmp.r = 0;
			tmp.g = 255;
			tmp.b = 0;
			pTargetCloudChangeRGB->points.push_back(tmp);
		}
		for (auto point : Final.points)
		{
			pcl::PointXYZRGB tmp;
			tmp.x = point.x;
			tmp.y = point.y;
			tmp.z = point.z;
			
			tmp.r = 0;
			tmp.g = 0;
			tmp.b = 255;
			FinalChangeRGB.points.push_back(tmp);
		}

		*pAccumulationCloud += *pSourceCloudChangeRGB;
		*pAccumulationCloud += *pTargetCloudChangeRGB;
		*pAccumulationCloud += FinalChangeRGB;


		if (icp.getFitnessScore() < 0.02)
		{
			ROS_INFO_STREAM (m_vecVehicleAccumulatedCloud[vehicleN]->size());
			ROS_INFO_STREAM (icp.getFitnessScore());

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTmpPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			*pTmpPointCloud += Final;
			*pTmpPointCloud += *pTargetCloud;

			m_vecVehicleAccumulatedCloud[vehicleN]->clear();
			downsample (pTmpPointCloud, m_vecVehicleAccumulatedCloud[vehicleN], 0.1);
		}
		else
		{
			ROS_ERROR_STREAM (m_vecVehicleAccumulatedCloud[vehicleN]->size());
			ROS_ERROR_STREAM (icp.getFitnessScore());
			*m_vecVehicleAccumulatedCloud[vehicleN] = *pTargetCloud;
		}

		extractCenterPoint (timestamp, m_vecVehicleAccumulatedCloud[vehicleN], vehicleN);
		vehicleN++;

		m_pub_result.publish (*pAccumulationCloud);
	}
}

void ExtractMeasurement::displayShape ()
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
		shape.header.stamp = ros::Time(pCluster->m_timestamp);
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
		vecOf_measurementCSV[shape.id-1] << shape.header.stamp << "," << shape.pose.position.x << "," << shape.pose.position.y << std::endl;

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

	//	// accumulation for publish
	//	for (const auto& pCluster : m_ObstacleTracking.m_TrackingObjects)
	//		*pAccumulationCloud += *(pCluster->GetCloud());

	// accumulation for publish
	for (const auto& pVehicleTrackingCloud : m_vecVehicleAccumulatedCloud)
		*pAccumulationCloud += *pVehicleTrackingCloud;

	// publish
	m_pub_result.publish (*pAccumulationCloud);
	m_pub_shape.publish (m_arrShapes);
}

void ExtractMeasurement::savePCD (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud)
{
	pcl::io::savePCDFile ("ICP_test.pcd", *pInputCloud);
}


void ExtractMeasurement::extractCenterPoint (long long timestamp, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud, unsigned int vehicleIndex)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pAccumulatedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	for (const auto& point : pInputCloud->points)
	{
		// fill new colored cluster point by point
		pcl::PointXYZRGB tmpPoint;
		tmpPoint.x = point.x;
		tmpPoint.y = point.y;
		tmpPoint.z = point.z;

		pAccumulatedCloud->points.push_back(tmpPoint);
	}

	std::vector<cv::Point2f> points;
	for (const auto& point : pAccumulatedCloud->points)
	{
		cv::Point2f pt;
		pt.x = point.x;
		pt.y = point.y;
		points.push_back(pt);
	}

	std::vector<cv::Point2f> vecHull;
	cv::convexHull(points, vecHull);


	cv::RotatedRect box = minAreaRect(vecHull);
//	m_center.position.x = box.center.x;
//	m_center.position.y = box.center.y;
//	m_dDistanceToCenter = hypot (m_center.position.y, m_center.position.x);
//	m_dimensions.x = box.size.width;
//	m_dimensions.y = box.size.height;
	vecOf_accumMeasurementCSV[vehicleIndex] << timestamp/1e6 << "," << box.center.x << "," << box.center.y << std::endl;
}
