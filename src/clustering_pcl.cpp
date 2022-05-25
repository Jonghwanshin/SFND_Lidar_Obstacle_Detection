/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "cluster.h"

#include <chrono>

#ifndef _POSIX_SOURCE
typedef unsigned int uint;
#endif

void proximity(KdTree* tree, 
				const std::vector<std::vector<float>>& points,
				int id,
				std::vector<bool>& visited, 
				std::vector<int>& cluster,
				float distanceTol)
{
	if(!visited[id]) 
	{
		//mark point as processed
		//add point to cluster
		cluster.push_back(id);
		visited[id] = true;
	}
	
	std::vector<int> cluster_temp = tree->search(points[id], distanceTol);
	for(auto idx : cluster_temp) //iterate through each nearby point
	{
		if(!visited[idx])
			proximity(tree, points, idx, visited, cluster, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(
	const std::vector<std::vector<float>>& points, 
    KdTree* tree, 
    float distanceTol, 
    int minSize, int maxSize)
{

	std::vector<std::vector<int>> clusters;
	std::vector<bool> visited = std::vector<bool>(points.size(), false);

	//iterate through each points
	for(int id = 0; id < points.size(); id++)
	{
		if(!visited[id]) //If points has not been processed
		{
            std::vector<int> cluster;
			proximity(tree, points, id, visited, cluster, distanceTol); 
            if(cluster.size() > minSize)
			    clusters.push_back(cluster); //cluster add clusters
		}
	}
	return clusters;

}