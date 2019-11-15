#include "extractMeasurement.h"


bool ID_sort (const clusterPtr& ID1, const clusterPtr& ID2)
{
	return ID1->m_id < ID2->m_id;
}

ExtractMeasurement::ExtractMeasurement(unsigned int size, bool bDoVisualizePCD) : m_measurementN (size), m_bDoVisualize(bDoVisualizePCD)
{
	// define publisher
	m_pub_result = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("OnlyBoundingBox", 100);
	m_pub_resultICP = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("ICP", 100);
	m_pub_shape = nh.advertise<visualization_msgs::MarkerArray>("Shape", 100);
	m_pub_shapeICP = nh.advertise<visualization_msgs::MarkerArray>("ShapeICP", 100);
	m_pub_shapeKalman = nh.advertise<visualization_msgs::MarkerArray>("ShapeKalman", 100);
	m_pub_shapeReference = nh.advertise<visualization_msgs::MarkerArray>("ShapeReference", 100);
	//m_pub_Origin = nh.advertise<visualization_msgs::Marker> ("Origin", 1);

	m_maxIndexNumber = 0;	

	vecOf_measurementCSV.resize (m_measurementN);
	vecOf_accumMeasurementCSV.resize (m_measurementN);
	vecOf_KalmanFilterCSV.resize (m_measurementN);

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

		vecOf_KalmanFilterCSV[measurementIndex].open ("Kalman_filter_" + num + ".csv");
		if (vecOf_KalmanFilterCSV[measurementIndex].is_open()){
			vecOf_KalmanFilterCSV[measurementIndex] << "timestamp, pose_x, pose_y, velocity_x, velocity_y" << std::endl;
		}
	}

	m_maxIndexNumber = 0;

	m_vecVehicleTrackingClouds.resize(m_measurementN);

	Eigen::Translation3f tl_btol(0.0, 0.0, 0.0);                 // tl: translation
	Eigen::AngleAxisf rot_x_btol(0.0, Eigen::Vector3f::UnitX());  // rot: rotation
	Eigen::AngleAxisf rot_y_btol(0.0, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf rot_z_btol(0.0, Eigen::Vector3f::UnitZ());
	m_mat4fBase2Local = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
	m_mat4fLocal2Base = m_mat4fBase2Local.inverse();

	m_previousPose.x = 0.0;
	m_previousPose.y = 0.0;
	m_previousPose.z = 0.0;
	m_previousPose.roll = 0.0;
	m_previousPose.pitch = 0.0;
	m_previousPose.yaw = 0.0;

	m_ndtPose.x = 0.0;
	m_ndtPose.y = 0.0;
	m_ndtPose.z = 0.0;
	m_ndtPose.roll = 0.0;
	m_ndtPose.pitch = 0.0;
	m_ndtPose.yaw = 0.0;

	m_currentPose.x = 0.0;
	m_currentPose.y = 0.0;
	m_currentPose.z = 0.0;
	m_currentPose.roll = 0.0;
	m_currentPose.pitch = 0.0;
	m_currentPose.yaw = 0.0;

	m_diff_x = 0.0;
	m_diff_y = 0.0;
	m_diff_z = 0.0;
	m_diff_yaw = 0.0;
}

void ExtractMeasurement::setParam()
{
	// set parameter
	nh.param ("extractMeasurement/Marker_duration", m_fMarkerDuration, 0.1);
	nh.param ("extractMeasurement/cluster_err_range", m_dClusterErrRadius, 0.5);
	nh.param ("extractMeasurement/cluster_min_size", m_dClusterMinSize, 15.0);
	nh.param ("extractMeasurement/cluster_max_size", m_dClusterMaxSize, 50.0);
}

void ExtractMeasurement::setData (const std::vector<VectorXd>& vecVecXdRef, long long timestamp)
{
	m_vecVecXdRefwithVelo = vecVecXdRef;

	m_vecVecXdRef.clear();
	for (const auto& ref : m_vecVecXdRefwithVelo)
	{
		VectorXd tmp(2);
		tmp[0] = ref[0];
		tmp[1] = ref[1];
		m_vecVecXdRef.push_back (tmp);
	}
	m_llTimestamp_s = timestamp;

	VectorXd vecXdRef = m_vecVecXdRef.back();
	geometry_msgs::Pose geomsgRefPose;
	geomsgRefPose.position.x = vecXdRef[0];
	geomsgRefPose.position.y = vecXdRef[1];
	m_geomsgReferences.poses.push_back (geomsgRefPose);

	m_myTools.setTime (timestamp);
}

void ExtractMeasurement::process()
{
	// get pcd 
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudTraffic (new pcl::PointCloud<pcl::PointXYZ>);
	getPCD(pCloudTraffic);

	// downsample
	pcl::PointCloud<pcl::PointXYZ>::Ptr pDownsampledCloud (new pcl::PointCloud<pcl::PointXYZ>);
	downsample(pCloudTraffic, pDownsampledCloud, 0.1);

	// dbscan
	std::vector<pcl::PointIndices> vecClusterIndices;
	dbscan (pDownsampledCloud, vecClusterIndices);

	// Set cluster pointcloud from clusterIndices and coloring
	setCluster (vecClusterIndices, pDownsampledCloud);

	// Associate 
	association ();

	// calculate RMSE
	calculateRMSE ();

	// display shape
	displayShape ();

	// publish	
	publish ();
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


void ExtractMeasurement::getPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic)
{
	if(m_bDoVisualize)
	{
		pCloudTraffic = loadPCD("/workspace/TrackingWithRegistration/src/tracking_with_registration/src/sensors/data/pcd/highway_"+std::to_string(m_llTimestamp_s)+".pcd");
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
		dummy.stamp = ros::Time(m_llTimestamp_s/1e6);

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

	// Sort the vector using predicate and std::sort
	std::sort(m_ObstacleTracking.m_TrackingObjects.begin(), m_ObstacleTracking.m_TrackingObjects.end(), ID_sort);

	// To store data in csv file, put the center point of OnlyBoundingBox tracking object into member variable with a data type of VectorXd
	for (auto pCluster : m_ObstacleTracking.m_TrackingObjects)
	{
		vecOf_measurementCSV[(pCluster->m_id)-1] << pCluster->m_timestamp << "," 
			<< pCluster->m_center.position.x << "," << pCluster->m_center.position.y << std::endl;
	}

	// To calculate RMSE, store the center point of OnlyBoundingBox tracking object into member variable with a data type of vector<VectorXd> 
	// which store the same object in same vector
	VectorXd meas(2);
	meas << m_ObstacleTracking.m_TrackingObjects[0]->m_center.position.x, m_ObstacleTracking.m_TrackingObjects[0]->m_center.position.y;
	m_vecVecXdOnlyBoundingbox.push_back (meas);

	
	// Store the each pointcloud of same obstacle over timestamp in same member variable 
	unsigned int vehicleN = 0;
	for (const auto& pCluster : m_ObstacleTracking.m_TrackingObjects)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTrackingCloud (pCluster->GetCloud());
		m_vecVehicleTrackingClouds[vehicleN].push_back (pTrackingCloud);
		vehicleN++;
	}


	static bool bIsFirst = true;
	// ICP
	if (m_bDoICP)
	{
		unsigned int vehicleN = 0;
		for (const auto& vecVehicleTrackingCloud : m_vecVehicleTrackingClouds)
		{
			if (bIsFirst)
			{
				clusterPtr pCluster (new Cluster());
				m_vecVehicleAccumulatedCloud.push_back(pCluster);
			}

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pAccumulatedCloud (m_vecVehicleAccumulatedCloud[vehicleN]->GetCloud());
			point2pointICPwithAccumulation (pAccumulatedCloud, vecVehicleTrackingCloud.back(), bIsFirst);

			unsigned int r; unsigned int g; unsigned int b;
			if (vehicleN == 0) {
				r = 255; g = 0; b = 0;
			}
			else if (vehicleN == 1) {
				r = 0; g = 255; b = 0;
			}
			else if (vehicleN == 2) {
				r = 0; g = 0; b = 255;
			}

			m_vecVehicleAccumulatedCloud[vehicleN]->SetCluster (m_llTimestamp_s, vehicleN, r, g, b);
			vehicleN++;
		}
	}
	// NDT
	else if (m_bDoNDT)
	{
		unsigned int vehicleN = 0;
		for (const auto& vecVehicleTrackingCloud : m_vecVehicleTrackingClouds)
		{
			if (bIsFirst)
			{
				clusterPtr pCluster (new Cluster());
				m_vecVehicleAccumulatedCloud.push_back(pCluster);
			}

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pAccumulatedCloud (m_vecVehicleAccumulatedCloud[vehicleN]->GetCloud());
			ExtractMeasurement::NDT (pAccumulatedCloud, vecVehicleTrackingCloud.back(), bIsFirst);

			unsigned int r; unsigned int g; unsigned int b;
			if (vehicleN == 0) {
				r = 255; g = 0; b = 0;
			}
			else if (vehicleN == 1) {
				r = 0; g = 255; b = 0;
			}
			else if (vehicleN == 2) {
				r = 0; g = 0; b = 255;
			}

			m_vecVehicleAccumulatedCloud[vehicleN]->SetCluster (m_llTimestamp_s, vehicleN, r, g, b);
			vehicleN++;
		}
	}

	// To store data in csv file, put the center point of registrated tracking object into member variable with a data type of VectorXd
	for (const auto& pCluster : m_vecVehicleAccumulatedCloud)
	{
		vecOf_accumMeasurementCSV[pCluster->m_id] << pCluster->m_timestamp << "," 
			<< pCluster->m_center.position.x << "," << pCluster->m_center.position.y << std::endl;
	}

	// To calculate RMSE, store the center point of registrated tracking object into member variable with a data type of vector<VectorXd> 
	// which store the same object in same vector
	VectorXd meas2 (2);
	meas2 << m_vecVehicleAccumulatedCloud[0]->m_center.position.x,  m_vecVehicleAccumulatedCloud[0]->m_center.position.y;
	m_vecVecXdRegistrationAccum.push_back (meas2);

	for (const auto& pCluster : m_vecVehicleAccumulatedCloud)
	{
		MeasurementPackage meas_package;
		meas_package.raw_measurements_ = VectorXd(2);

		meas_package.raw_measurements_ << pCluster->m_center.position.x, 
			pCluster->m_center.position.y;
		meas_package.timestamp_ = m_llTimestamp_s;

		pCluster->KF.ProcessMeasurement(meas_package);
	}

	// To store data in csv file, put the center point of Kalman filter tracking object into member variable with a data type of VectorXd
	for (const auto& pCluster : m_vecVehicleAccumulatedCloud)
	{
		vecOf_KalmanFilterCSV[pCluster->m_id] << pCluster->m_timestamp << "," 
											  << pCluster->KF.x_[0] << ","
											  << pCluster->KF.x_[1] << ","
											  << pCluster->KF.x_[2] << ","
											  << pCluster->KF.x_[3] << ","
											  << std::endl;
	}

	// To calculate RMSE, store the center point of kalman filter tracking object into member variable with a data type of vector<VectorXd> 
	// which store the same object in same vector
	VectorXd meas3 (4);
	meas3 << m_vecVehicleAccumulatedCloud[0]->KF.x_[0],
			 m_vecVehicleAccumulatedCloud[0]->KF.x_[1],
			 m_vecVehicleAccumulatedCloud[0]->KF.x_[2],
			 m_vecVehicleAccumulatedCloud[0]->KF.x_[3],
	m_vecVecXdKalmanFilter.push_back (meas3);

	bIsFirst = false;
}

void ExtractMeasurement::point2pointICPwithAccumulation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputSourceCloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputTargetCloud, bool bIsFirst)
{
	if (bIsFirst)
	{
		pInputSourceCloud->swap (*pInputTargetCloud);
		bIsFirst = false;
	}
	else if(!bIsFirst)
	{
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		icp.setInputSource (pInputSourceCloud);
		icp.setInputTarget (pInputTargetCloud);
		pcl::PointCloud<pcl::PointXYZRGB> Final;
		icp.align (Final);

		if (icp.getFitnessScore() < 0.02)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTmpPointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			*pTmpPointCloud += Final;
			*pTmpPointCloud += *pInputTargetCloud;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTmpPointCloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
			downsample (pTmpPointCloud, pTmpPointCloud2, 0.09);
			pInputSourceCloud->clear ();
			pInputSourceCloud->swap (*pTmpPointCloud2);
		}

		else {
			pInputSourceCloud->clear ();
			pInputSourceCloud->swap (*pInputTargetCloud);
		}
	}
}

void ExtractMeasurement::NDT (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputTargetCloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputSourceCloud, bool bIsInitSource)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTransformedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Add initial point cloud to 
	if (bIsInitSource)
	{
		pcl::transformPointCloud (*pInputSourceCloud, *pTransformedCloud, m_mat4fBase2Local);
		pInputTargetCloud->swap (*pTransformedCloud);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pDownsampledCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	downsample(pInputSourceCloud, pDownsampledCloud, 0.9);


	m_ndt.setTransformationEpsilon(0.01);
	m_ndt.setStepSize(0.1);
	m_ndt.setResolution(1.0);
	m_ndt.setMaximumIterations(30);
	m_ndt.setInputSource (pDownsampledCloud);

	if (bIsInitSource)
	{
		m_ndt.setInputTarget (pInputTargetCloud);
		bIsInitSource = false;
	}

	pose guess_pose;

	guess_pose.x = m_previousPose.x + m_diff_x;	// what is the coordinate of guess_pose
	guess_pose.y = m_previousPose.y + m_diff_y;
	guess_pose.z = m_previousPose.z + m_diff_z;
	guess_pose.roll = m_previousPose.roll;
	guess_pose.pitch = m_previousPose.pitch;
	guess_pose.yaw = m_previousPose.yaw + m_diff_yaw;

	Eigen::AngleAxisf init_rotation_x (guess_pose.roll, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y (guess_pose.pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z (guess_pose.yaw, Eigen::Vector3f::UnitZ());

	Eigen::Translation3f init_translation(guess_pose.x, guess_pose.y, guess_pose.z);
	Eigen::Matrix4f init_guess =
		(init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * m_mat4fBase2Local;


	Eigen::Matrix4f mat4fLocalizer (Eigen::Matrix4f::Identity());
	Eigen::Matrix4f mat4fLocalizerInverse (Eigen::Matrix4f::Identity());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pOutputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	m_ndt.align(*pOutputCloud, init_guess);
	mat4fLocalizer = m_ndt.getFinalTransformation();
	mat4fLocalizerInverse = mat4fLocalizer.inverse();

	Eigen::Matrix4f mat4fBaseLink(Eigen::Matrix4f::Identity());
	mat4fBaseLink = mat4fLocalizer * m_mat4fLocal2Base;

	pcl::transformPointCloud(*pInputSourceCloud, *pTransformedCloud, mat4fLocalizer);
	//pInputTargetCloud->swap (*pTransformedCloud);

	tf::Matrix3x3 mat_b;

	mat_b.setValue(static_cast<double>(mat4fBaseLink(0, 0)), static_cast<double>(mat4fBaseLink(0, 1)),
			static_cast<double>(mat4fBaseLink(0, 2)), static_cast<double>(mat4fBaseLink(1, 0)),
			static_cast<double>(mat4fBaseLink(1, 1)), static_cast<double>(mat4fBaseLink(1, 2)),
			static_cast<double>(mat4fBaseLink(2, 0)), static_cast<double>(mat4fBaseLink(2, 1)),
			static_cast<double>(mat4fBaseLink(2, 2)));

	// Update m_ndtPose.
	m_ndtPose.x = mat4fBaseLink(0, 3);
	m_ndtPose.y = mat4fBaseLink(1, 3);
	m_ndtPose.z = mat4fBaseLink(2, 3);
	mat_b.getRPY(m_ndtPose.roll, m_ndtPose.pitch, m_ndtPose.yaw, 1);

	m_currentPose.x = m_ndtPose.x;
	m_currentPose.y = m_ndtPose.y;
	m_currentPose.z = m_ndtPose.z;
	m_currentPose.roll = m_ndtPose.roll;
	m_currentPose.pitch = m_ndtPose.pitch;
	m_currentPose.yaw = m_ndtPose.yaw;

	// Calculate the offset (curren_pos - previous_pos)
	m_diff_x = m_currentPose.x - m_previousPose.x;
	m_diff_y = m_currentPose.y - m_previousPose.y;
	m_diff_z = m_currentPose.z - m_previousPose.z;
	m_diff_yaw = calcDiffForRadian(m_currentPose.yaw, m_previousPose.yaw);

	// Update position and posture. current_pos -> previous_pos
	m_previousPose.x = m_currentPose.x;
	m_previousPose.y = m_currentPose.y;
	m_previousPose.z = m_currentPose.z;
	m_previousPose.roll = m_currentPose.roll;
	m_previousPose.pitch = m_currentPose.pitch;
	m_previousPose.yaw = m_currentPose.yaw;

	//*pInputTargetCloud += *pDownsampledCloud;
	*pInputTargetCloud += *pTransformedCloud;
	m_ndt.setInputTarget(pInputTargetCloud);
}

double ExtractMeasurement::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
	double diff_rad = lhs_rad - rhs_rad;
	if (diff_rad >= M_PI)
		diff_rad = diff_rad - 2 * M_PI;
	else if (diff_rad < -M_PI)
		diff_rad = diff_rad + 2 * M_PI;
	return diff_rad;
}

void ExtractMeasurement::displayShape ()
{
	// Tracking objects
	m_arrShapes.markers.clear();
	m_arrShapesICP.markers.clear();
	m_arrShapesKalman.markers.clear();
	m_arrShapesReference.markers.clear();

	// For OnlyBoundingBox
	for (auto pCluster : m_ObstacleTracking.m_TrackingObjects)
	{
		visualization_msgs::Marker shape;

		shape.lifetime = ros::Duration();
		shape.header.frame_id = "map";
		shape.header.stamp = ros::Time(pCluster->m_timestamp);
		shape.id = pCluster->m_id;

		// bounding box
		shape.type = visualization_msgs::Marker::CUBE;
		shape.action = visualization_msgs::Marker::ADD;
		shape.ns = "/OnlyBoundingBox";

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
		shape.ns = "/vehicle number";

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

		// text
		string s_x_RMSE = std::to_string(m_vecVecXdResultRMSE[0][0]);
		string s_y_RMSE = std::to_string(m_vecVecXdResultRMSE[0][1]);
		string sWholeText = "OnlyBoundingBox RMSE\r\nx: " + s_x_RMSE + "\r\ny: " + s_y_RMSE;

		shape.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		shape.ns = "/RMSE";

		shape.points.clear();
		shape.pose.position.x = 0;
		shape.pose.position.y = 30;
		shape.pose.position.z = 0;
		shape.pose.orientation.x = 0.0;
		shape.pose.orientation.y = 0.0;
		shape.pose.orientation.z = 0.0;
		shape.pose.orientation.w = 1.0;

		shape.scale.x = 2.0;
		shape.scale.y = 2.0;
		shape.scale.z = 2.0;

		shape.color.r = shape.color.g = shape.color.b = 1.0;
		shape.color.a = 1.0;

		shape.text = sWholeText;

		m_arrShapes.markers.push_back (shape);

		// center point using sphere
		shape.type = visualization_msgs::Marker::SPHERE;
		shape.action = visualization_msgs::Marker::ADD;
		shape.ns = "/center point";

		shape.points.clear();
		shape.pose.position = pCluster->m_center.position;
		shape.pose.orientation = pCluster->m_center.orientation;

		shape.scale.x = 0.5;
		shape.scale.y = 0.5;
		shape.scale.z = 0.5;

		shape.color.r = 1.0;
		shape.color.g = 1.0;
		shape.color.b = 0.0;
		shape.color.a = 0.5;

		m_arrShapes.markers.push_back (shape);
	}

	// For registration and accumulation
	for (auto pCluster : m_vecVehicleAccumulatedCloud)
	{
		visualization_msgs::Marker shape;

		shape.lifetime = ros::Duration();
		shape.header.frame_id = "map";
		shape.header.stamp = ros::Time(pCluster->m_timestamp);
		shape.id = (pCluster->m_id)+1;

		// bounding box
		shape.type = visualization_msgs::Marker::CUBE;
		shape.action = visualization_msgs::Marker::ADD;
		shape.ns = "/Registration&accumulation";

		shape.pose.position = pCluster->m_center.position;
		shape.pose.orientation = pCluster->m_center.orientation;

		shape.scale.x = pCluster->m_dimensions.x;
		shape.scale.y = pCluster->m_dimensions.y;
		shape.scale.z = pCluster->m_dimensions.z;

		shape.color.r = pCluster->m_r/254.0f;
		shape.color.g = pCluster->m_g/254.0f;
		shape.color.b = pCluster->m_b/254.0f;
		shape.color.a = 0.5;

		m_arrShapesICP.markers.push_back(shape);

		// text
		shape.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		shape.ns = "/vehicle number";

		shape.points.clear();
		shape.pose.position = pCluster->m_center.position;
		shape.pose.orientation = pCluster->m_center.orientation;

		shape.scale.x = 1.0;
		shape.scale.y = 1.0;
		shape.scale.z = 1.0;

		shape.color.r = shape.color.g = shape.color.b = 1.0;
		shape.color.a = 1.0;

		shape.text = std::to_string(shape.id);

		m_arrShapesICP.markers.push_back (shape);

		// text
		string s_x_RMSE = std::to_string(m_vecVecXdResultRMSE[1][0]);
		string s_y_RMSE = std::to_string(m_vecVecXdResultRMSE[1][1]);
		string sWholeText = "Registraton and Accumulation RMSE\r\nx: " + s_x_RMSE + "\r\ny: " + s_y_RMSE;

		shape.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		shape.ns = "/RMSE";

		shape.points.clear();
		shape.pose.position.x = 0;
		shape.pose.position.y = 20;
		shape.pose.position.z = 0;
		shape.pose.orientation.x = 0.0;
		shape.pose.orientation.y = 0.0;
		shape.pose.orientation.z = 0.0;
		shape.pose.orientation.w = 1.0;

		shape.scale.x = 2.0;
		shape.scale.y = 2.0;
		shape.scale.z = 2.0;

		shape.color.r = shape.color.g = shape.color.b = 1.0;
		shape.color.a = 1.0;

		shape.text = sWholeText;

		m_arrShapesICP.markers.push_back (shape);

		// center point using sphere
		shape.type = visualization_msgs::Marker::SPHERE;
		shape.action = visualization_msgs::Marker::ADD;
		shape.ns = "/center point";

		shape.points.clear();
		shape.pose.position = pCluster->m_center.position;
		shape.pose.orientation = pCluster->m_center.orientation;

		shape.scale.x = 0.5;
		shape.scale.y = 0.5;
		shape.scale.z = 0.5;

		shape.color.r = 0.0;
		shape.color.g = 1.0;
		shape.color.b = 1.0;
		shape.color.a = 0.5;

		m_arrShapesICP.markers.push_back (shape);

		visualization_msgs::Marker shapeForKalman;

		shapeForKalman = shape;

		shapeForKalman.ns = "/kalman center point";

		shapeForKalman.points.clear();
		shapeForKalman.pose.position.x = pCluster->KF.x_[0];
		shapeForKalman.pose.position.y = pCluster->KF.x_[1];
		shapeForKalman.pose.position.z = 0.0;
		shapeForKalman.pose.orientation.x = 0.0;
		shapeForKalman.pose.orientation.y = 0.0;
		shapeForKalman.pose.orientation.z = 0.0;
		shapeForKalman.pose.orientation.w = 1.0;

		shapeForKalman.color.r = 1.0;
		shapeForKalman.color.g = 0.0;
		shapeForKalman.color.b = 1.0;
		shapeForKalman.color.a = 0.5;
		m_arrShapesKalman.markers.push_back (shapeForKalman);

		// text
		string s_px_RMSE = std::to_string(m_vecVecXdResultRMSE[2][0]);
		string s_py_RMSE = std::to_string(m_vecVecXdResultRMSE[2][1]);
		string s_vx_RMSE = std::to_string(m_vecVecXdResultRMSE[2][2]);
		string s_vy_RMSE = std::to_string(m_vecVecXdResultRMSE[2][3]);
		sWholeText = "Kalman filter RMSE\r\npx: " + s_px_RMSE
				   + "\r\npy: " + s_py_RMSE
				   + "\r\nvx: " + s_vx_RMSE
				   + "\r\nvy: " + s_vy_RMSE;

		shapeForKalman.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		shapeForKalman.ns = "/RMSE";

		shapeForKalman.points.clear();
		shapeForKalman.pose.position.x = 0;
		shapeForKalman.pose.position.y = 10;
		shapeForKalman.pose.position.z = 0;
		shapeForKalman.pose.orientation.x = 0.0;
		shapeForKalman.pose.orientation.y = 0.0;
		shapeForKalman.pose.orientation.z = 0.0;
		shapeForKalman.pose.orientation.w = 1.0;

		shapeForKalman.scale.x = 2.0;
		shapeForKalman.scale.y = 2.0;
		shapeForKalman.scale.z = 2.0;

		shapeForKalman.color.r = shapeForKalman.color.g = shapeForKalman.color.b = 1.0;
		shapeForKalman.color.a = 1.0;

		shapeForKalman.text = sWholeText;

		m_arrShapesKalman.markers.push_back (shapeForKalman);
	}

	// For reference
	{
		visualization_msgs::Marker shape;

		// Line stript
		shape.lifetime = ros::Duration();
		shape.header.frame_id = "map";
		shape.header.stamp = ros::Time::now();
		shape.id = 0;

		shape.type = visualization_msgs::Marker::LINE_STRIP;
		shape.action = visualization_msgs::Marker::ADD;
		shape.ns = "/Trajectory";

		shape.pose.orientation.x = 0.0;
		shape.pose.orientation.y = 0.0;
		shape.pose.orientation.z = 0.0;
		shape.pose.orientation.w = 1.0;

		shape.scale.x = 0.09; 
		shape.scale.y = 0.09; 
		shape.scale.z = 0.09;

		shape.color.r = 1.0;
		shape.color.g = 0.0;
		shape.color.b = 0.0;
		shape.color.a = 1;

		for (const auto& pose : m_geomsgReferences.poses)
		{
			geometry_msgs::Point tmp;
			tmp.x = pose.position.x;
			tmp.y = pose.position.y;
			tmp.z = pose.position.z;
			shape.points.push_back(tmp);
		}

		m_arrShapesReference.markers.push_back (shape);

		// End point
		geometry_msgs::Pose geomsgEndPoint(m_geomsgReferences.poses.back());

		shape.type = visualization_msgs::Marker::SPHERE;
		shape.action = visualization_msgs::Marker::ADD;
		shape.ns = "/End point";

		shape.pose.position = geomsgEndPoint.position;
		shape.pose.orientation.x = 0.0;
		shape.pose.orientation.y = 0.0;
		shape.pose.orientation.z = 0.0;
		shape.pose.orientation.w = 1.0;

		shape.scale.x = 0.5; 
		shape.scale.y = 0.5; 
		shape.scale.z = 0.5;

		shape.color.r = 1.0;
		shape.color.g = 0.0;
		shape.color.b = 0.0;
		shape.color.a = 1;

		m_arrShapesReference.markers.push_back(shape);
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

	// Accumulate all cluster to pAccumCloudForICP
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pAccumCloudForICP (new pcl::PointCloud<pcl::PointXYZRGB>);
	pAccumCloudForICP->header.frame_id = "map";

	// accumulation for publish
	for (const auto& pVehicleTrackingCloud : m_vecVehicleAccumulatedCloud)
		*pAccumCloudForICP += *(pVehicleTrackingCloud->GetCloud());


	// publish
	m_pub_resultICP.publish (*pAccumCloudForICP);
	m_pub_result.publish (*pAccumulationCloud);
	m_pub_shapeReference.publish (m_arrShapesReference);
	m_pub_shapeKalman.publish (m_arrShapesKalman);
	m_pub_shapeICP.publish (m_arrShapesICP);
	m_pub_shape.publish (m_arrShapes);
}

void ExtractMeasurement::savePCD (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pInputCloud)
{
	pcl::io::savePCDFile ("ICP_test.pcd", *pInputCloud);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractMeasurement::loadPCD (std::string file)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file \n");
	}
	//std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

	return cloud;
}


void ExtractMeasurement::calculateRMSE ()
{
	m_vecVecXdResultRMSE.clear();

	VectorXd vOnlyBoundingBoxRMSE(2);
	vOnlyBoundingBoxRMSE << 0,0;

	for (unsigned int measIndex = 0; measIndex < m_vecVecXdRef.size(); measIndex++)
	{
		VectorXd residual = m_vecVecXdRef[measIndex] - m_vecVecXdOnlyBoundingbox[measIndex];
		residual = residual.array() * residual.array();
		vOnlyBoundingBoxRMSE += residual;
	}
	vOnlyBoundingBoxRMSE = vOnlyBoundingBoxRMSE/m_vecVecXdRef.size();
	vOnlyBoundingBoxRMSE = vOnlyBoundingBoxRMSE.array().sqrt();

	m_vecVecXdResultRMSE.push_back (vOnlyBoundingBoxRMSE);
	m_myTools.setOnlyBoundingBoxRMSE (vOnlyBoundingBoxRMSE);

	VectorXd vRegistrationAccumRMSE(2);
	vRegistrationAccumRMSE << 0,0;

	for (unsigned int measIndex = 0; measIndex < m_vecVecXdRef.size(); measIndex++)
	{
		VectorXd residual = m_vecVecXdRef[measIndex] - m_vecVecXdRegistrationAccum[measIndex];
		residual = residual.array() * residual.array();
		vRegistrationAccumRMSE += residual;
	}
	vRegistrationAccumRMSE = vRegistrationAccumRMSE/m_vecVecXdRef.size();
	vRegistrationAccumRMSE = vRegistrationAccumRMSE.array().sqrt();

	m_vecVecXdResultRMSE.push_back (vRegistrationAccumRMSE);
	m_myTools.setAccumulationRMSE (vRegistrationAccumRMSE);

	VectorXd vKalmanRMSE (4);
	vKalmanRMSE << 0,0,0,0;

	for (unsigned int measIndex = 0; measIndex < m_vecVecXdRefwithVelo.size(); measIndex++)
	{
		VectorXd residual = m_vecVecXdRefwithVelo[measIndex] - m_vecVecXdKalmanFilter[measIndex];
		residual = residual.array() * residual.array();
		vKalmanRMSE += residual;
	}
	vKalmanRMSE = vKalmanRMSE/m_vecVecXdRefwithVelo.size();
	vKalmanRMSE = vKalmanRMSE.array().sqrt();

	m_vecVecXdResultRMSE.push_back (vKalmanRMSE);
	m_myTools.setKalmanFilterRMSE (vKalmanRMSE);
}
