#include "obstacle_tracking.hpp"


ObstacleTracking::ObstacleTracking()
{
	m_MAX_ASSOCIATION_DISTANCE = 2.0;
	iTracksNumber = 0;
}


ObstacleTracking::~ObstacleTracking() {}


void ObstacleTracking::association (const std::vector<clusterPtr>& objList)
{
	m_DetectedObjects = objList;

	matchWithDistanceOnly();
	m_DetectedObjects.clear();
}


void ObstacleTracking::matchWithDistanceOnly()
{
	m_TmpTrackingObjects.clear();
	double d_y = 0, d_x = 0, d = 0;


	while (m_DetectedObjects.size() > 0)
	{
		int iClosest_track = -1;
		int iClosest_obj = -1;
		double dClosest = m_MAX_ASSOCIATION_DISTANCE;

		// foreach the detected objects 
		unsigned int objectNum = 0;
		for (const auto& pDetectedObject : m_DetectedObjects)
		{
			// foreach the tracking objects
			unsigned int trackNum = 0;
			for (const auto& pTrackingObject : m_TrackingObjects)
			{
				d_y = pDetectedObject->m_center.position.y - pTrackingObject->m_center.position.y;
				d_x = pDetectedObject->m_center.position.x - pTrackingObject->m_center.position.x;
				//
				// calculate distance
				d = hypot(d_y, d_x);

				// find the detected object closest to the tracking object
				if(d < dClosest)
				{
					dClosest = d;
					iClosest_obj = objectNum;
					iClosest_track = trackNum;
				}
				trackNum++;
			}
			objectNum++;
		}

		// if a detected object mathcing with a tracking object is more than one
		if(iClosest_obj != -1 && iClosest_track != -1 && dClosest < m_MAX_ASSOCIATION_DISTANCE)
		{
			// replace the new detected-object's id to the old detected-object's id
			// and insert the new detected-object to the old detected-object =>
			// change all information except id about old detected-object
			MergeObjectAndTrack(m_TrackingObjects.at(iClosest_track), m_DetectedObjects.at(iClosest_obj));

			// push old detected-object that changed all information except id
			m_TmpTrackingObjects.push_back(m_TrackingObjects.at(iClosest_track));

			// erase pair of matching object
			m_TrackingObjects.erase(m_TrackingObjects.begin()+iClosest_track);
			m_DetectedObjects.erase(m_DetectedObjects.begin()+iClosest_obj);
		}
		// detected object matching with a tracking object is zero 
		else
		{
			iTracksNumber += 1;
			m_DetectedObjects.at(0)->m_id = iTracksNumber;
			m_TmpTrackingObjects.push_back(m_DetectedObjects.at(0));
			m_DetectedObjects.erase (m_DetectedObjects.begin() + 0);

		}
	}

	m_TrackingObjects = m_TmpTrackingObjects;
}


void ObstacleTracking::MergeObjectAndTrack (clusterPtr& track, clusterPtr& obj)
{
	obj->m_id = track->m_id;
	obj->m_originalID = track->m_originalID;

	obj->m_label = track->m_label;
	obj->m_r = track->m_r;
	obj->m_g = track->m_g;
	obj->m_b = track->m_b;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pTmpCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (auto point : obj->GetCloud()->points)
	{
		pcl::PointXYZRGB tmpPoint;
		tmpPoint.x = point.x;
		tmpPoint.y = point.y;
		tmpPoint.z = point.z;

		tmpPoint.r = track->m_r;
		tmpPoint.g = track->m_g;
		tmpPoint.b = track->m_b;
		pTmpCloud->points.push_back(tmpPoint);
	}

	obj->setPointCloud (pTmpCloud);

	track = obj;
}
