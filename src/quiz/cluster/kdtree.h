/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef KDTREE_H_
#define KDTREE_H_

#include "../../render/render.h"

#include <queue>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}
	void insert(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root 
		int level = 0;
		if(root == NULL)
		{
			root = new Node(point, id);	
			return;
		}
		
		Node** prev = &root;
		Node** next = &root;
		int dims = point.size();
		while((*next) != NULL)
		{
			int idx = level++ % dims;
			if((*prev)->point[idx] < point[idx])
			{
				prev = next;
				next = &((*next)->right);
			}
			else
			{
				prev = next;
				next = &((*next)->left);
			}
		}
		*next = new Node(point, id);
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node == NULL)
			return;
		
		if (node->point.size() != target.size())
			throw std::invalid_argument("Dimension doesn't match to search");
		
		int dims = target.size();
		std::vector<float> distances = std::vector<float>(dims, 0.0f);
		bool is_too_far = false;

		for(int i = 0; i < dims; i++)
		{
			float dist = abs(target[i] - node->point[i]);
			distances[i] = dist;
			if(dist > distanceTol)
				is_too_far = true;
		}

		if(!is_too_far)
		{
			float distance = 0.0f;
			for(int i = 0; i < dims; i++)
			{
				distance +=	pow(distances[i], 2);
			}
			distance = sqrt(distance);
			if(distance <= distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		int idx = depth % dims;
		if((target[idx] - distanceTol) < node->point[idx])
		{	
			searchHelper(target, node->left, depth+1, distanceTol, ids);	
		}
		if((target[idx] + distanceTol) > node->point[idx])
		{	
			searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// x and y
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};

#endif