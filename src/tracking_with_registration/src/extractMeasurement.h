#ifndef EXTRACTMEASUREMENT
#define EXTRACTMEASUREMENT

#include "tools.h"

class ExtractMeasurement
{
	public:
		Tools m_tools;
		void loadPCD (pcl::PointCloud<pcl::PointXYZ>::Ptr& pCloudTraffic, long long timestamp, bool doVisualize);
//		void dbscan (const PointCloudXYZ::ConstPtr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices);
//		void setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, std::vector<clusterPtr>& pOriginalClusters, const PointCloudXYZ::Ptr pInputCloud);
//		void generateColor(size_t indexNumber);
//		void displayShape (const std::vector<clusterPtr> pVecClusters);
//		void setDetectedObject (const std::vector<clusterPtr>& pVecClusters);
};


#endif //EXTRACTMEASUREMENT
