// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

#include <unordered_set>

#include "quiz/cluster/kdtree.h"
#include "quiz/cluster/cluster.h"


// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
	std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();

	// TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	pcl::VoxelGrid<PointT> vg;
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

	vg.setInputCloud(cloud);
	vg.setLeafSize(filterRes, filterRes, filterRes);
	vg.filter(*cloudFiltered);

	typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

	typename pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloudFiltered);
	region.filter(*cloudRegion);

	std::vector<int> indices;

	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
	roof.setInputCloud(cloudRegion);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
	for (int point : indices)
		inliers->indices.push_back(point);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloudRegion);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloudRegion);

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

	return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
	// TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

	for (int index : inliers->indices)
		planeCloud->points.push_back(cloud->points[index]);

	// Create the filtering object
	typename pcl::ExtractIndices<PointT> extract;
	// Extract the inliers
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstCloud);

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
	return segResult;
}

template <typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersBestFitted;
	pcl::PointIndices::Ptr inliersResult(new pcl::PointIndices());

	srand(time(NULL));

	int i = 0;

	// For max iterations
	while (maxIterations--)
	{
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size()));

		// Measure distance between every point and fitted line
		float x1, x2, z1, y1, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

		float a = i;
		float b = j;
		float c = k;
		float d = -(i * x1 + j * y1 + k * z1);

		// If distance is smaller than threshold count it as inlier
		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0)
			{
				continue;
			}

			typename PointT point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
			float z3 = point.z;

			float dist = fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c);

			if (dist <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult->indices.size())
		{
			inliersBestFitted = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	for(auto it = inliersBestFitted.begin(); it != inliersBestFitted.end(); ++it)
		inliersResult->indices.push_back(*it);
	return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneV2(
	typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	
	pcl::PointIndices::Ptr inliers = this->Ransac(cloud, maxIterations, distanceThreshold);
	if (inliers->indices.size() == 0)
	{
		std::cout << "could not estimate a planar model for the given dataset." << std::endl;
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
	return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
	//  TODO:: Fill in this function to find inliers for the cloud.

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	typename pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIterations);
	seg.setDistanceThreshold(distanceThreshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		std::cout << "could not estimate a planar model for the given dataset." << std::endl;
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
	return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

	// Time clustering process
	auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	// Creating the KdTree object for the search method of the extraction
	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	typename pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(clusterTolerance); // 2cm
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(maxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	for (pcl::PointIndices indices : cluster_indices)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		for (int idx : indices.indices)
			cloud_cluster->push_back(cloud->points[idx]); //*
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
		clusters.push_back(cloud_cluster);
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringV2(
	typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

	// Time clustering process
	auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	
	// Creating the KdTree object for the search method of the extraction
	KdTree* tree = new KdTree;
	std::vector<std::vector<float>> points = std::vector<std::vector<float>>();

	for(int i = 0; i < cloud->size(); ++i)
	{
		PointT& p = cloud->at(i);
		std::vector<float> point = {p.x, p.y, p.z};
		tree->insert(point, i);
		points.push_back(point);
	}

	// std::vector<std::vector<int>> clustersV2 = euclideanCluster(points, tree, clusterTolerance);

	// if(auto cluster : clusters)
	// {
	// 	std::cout << "PointCloud representing the Cluster: " << cluster.size() << " data points." << std::endl;
	// }

	// for (pcl::PointIndices indices : cluster_indices)e
	// {
	// 	typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
	// 	for (int idx : indices.indices)
	// 		cloud_cluster->push_back(cloud->points[idx]); //*
	// 	cloud_cluster->width = cloud_cluster->size();
	// 	cloud_cluster->height = 1;
	// 	cloud_cluster->is_dense = true;

	// 	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
	// 	clusters.push_back(cloud_cluster);
	// }

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

	// Find bounding box for one of the clusters
	PointT minPoint, maxPoint;
	pcl::getMinMax3D(*cluster, minPoint, maxPoint);

	Box box;
	box.x_min = minPoint.x;
	box.y_min = minPoint.y;
	box.z_min = minPoint.z;
	box.x_max = maxPoint.x;
	box.y_max = maxPoint.y;
	box.z_max = maxPoint.z;

	return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
	pcl::io::savePCDFileASCII(file, *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

	typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
	}
	std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

	return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

	std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

	// sort files in accending order so playback is chronological
	sort(paths.begin(), paths.end());

	return paths;
}