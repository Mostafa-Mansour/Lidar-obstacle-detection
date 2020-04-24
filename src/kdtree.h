/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "./render/render.h"


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
	uint treeDim;

	KdTree()
	: root(NULL), treeDim(2)
	{}
	KdTree(int dim): root(NULL), treeDim(3){}

	void insertPntKDTree(Node** ptr,int depth,std::vector<float> point,int id){

		if(*ptr==NULL){
			*ptr=new Node(point, id);
			return;
		}

		uint idx=depth%treeDim;

		if(point[idx]>(*ptr)->point[idx])
			insertPntKDTree(&((*ptr)->right),depth+1,point,id);
		else
		{
			insertPntKDTree(&((*ptr)->left),depth+1,point,id);
		}
		

	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertPntKDTree(&root,0,point,id);

	}

	float distance(std::vector<float> pnt,std::vector<float> target){
		float normVal=0;
		for(int i=0;i<target.size();i++)
			normVal+=std::pow((pnt[i]-target[i]),2);
		
		return std::sqrt(normVal);
	}

	void searchPntKDTree(Node** ptr,int depth,std::vector<float>& target, float distanceTol,std::vector<int>& ids){
		bool pntInBox=true;
		
		if(*ptr!=NULL){
			
			for(int i=0;i<target.size();i++){
				if((*ptr)->point[i]>=target[i]-distanceTol && (*ptr)->point[i]<=target[i]+distanceTol)
					pntInBox=pntInBox && true;
				else
				{
					pntInBox=pntInBox && false;
				}
				
			}
			if(pntInBox){

				float pntDistance=distance((*ptr)->point,target);
				if(pntDistance<=distanceTol)
					ids.push_back((*ptr)->id);
				}
		}
		else{
			return;
		}

		

		if(target[depth%treeDim]-distanceTol <= (*ptr)->point[depth%treeDim])
			searchPntKDTree(&((*ptr)->left),depth+1,target,distanceTol,ids);
		
		if(target[depth%treeDim]+distanceTol >= (*ptr)->point[depth%treeDim])
			searchPntKDTree(&((*ptr)->right),depth+1,target,distanceTol,ids);
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
				
		searchPntKDTree(&root,0,target,distanceTol,ids);
		return ids;
	}
	

};




