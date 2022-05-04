/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
		float x = point[0], y = point[1];
		while((*next) != NULL)
		{
			float x_cmp = (*prev)->point[0], y_cmp = (*prev)->point[1];
			if((level++) % 2 == 0)
			{
				if(x > x_cmp)
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
			else
			{
				if(y > y_cmp)
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
		}
		*next = new Node(point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		
		return ids;
	}
	

};




