/* \author Aaron Brown */
// Quiz on implementing kd tree

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
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		int level = 0;
		if(root == NULL)
		{
			root = new Node(point, id);	
			return;
		}
		
		Node** prev = &root;
		Node** next = &root;
		while((*next) != NULL)
		{
			int idx = level++ % 2;
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

		float x_target = target[0], y_target = target[1];
		float x_cmp = node->point[0], y_cmp = node->point[1];

		if((abs(x_target - x_cmp)< distanceTol) && (abs(y_target - y_cmp)< distanceTol))
		{
			float distance = sqrt(pow(x_cmp - x_target, 2) + pow(y_cmp - y_target, 2));
			if(distance <= distanceTol) //in distance
			{
				ids.push_back(node->id);
			}
		}

		int idx = depth % 2;
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




