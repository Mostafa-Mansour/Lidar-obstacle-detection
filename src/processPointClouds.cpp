// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // DONE:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr regionedCloud(new pcl::PointCloud<PointT>());    
    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(filteredCloud);
    roi.filter(*regionedCloud);

    // removing car roof
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1)); // min car dimensions
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1)); // max car dimensions
    roof.setInputCloud(regionedCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(auto& idx:indices)
        inliers->indices.push_back(idx);
    
    // extract inliers to get a cloud without roof points
    typename pcl::PointCloud<PointT>::Ptr outCloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(regionedCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*outCloud);







    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return outCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // Done: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Define two ptrs for two cloud, plane cloud and obstacle cloud
    typename pcl::PointCloud<PointT>::Ptr planePtr(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstaclePtr(new pcl::PointCloud<PointT>);

    // extract plane point cloud
    for(auto idx:inliers->indices){
        planePtr->points.push_back(cloud->points[idx]);
    }

    // subtract the plane point cloud from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstaclePtr);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclePtr, planePtr);
    return segResult;
}


// PCL implementation for plane segmentation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // DONE:: Fill in this function to find inliers for the cloud.
    // plane coefficients
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //creat a segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);

    // Define the model
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coefficients);

    if(inliers->indices.size()==0){
        std::cout<<"The model can not be fitted"<<std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    
    return segResult;
}

// OWN implementation for plane segmentation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool flag)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr inliersRes(new pcl::PointIndices);
    inliersRes->header=cloud->header;
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

	float a,b,c,d ; //plane coefficients
	float x1,y1,z1,x2,y2,z2,x3,y3,z3; // points coordinates
	int numElements; // number of fitted points
	int maxNumberOfPoints=0;
	
	// DONE: Fill in this function
	while(maxIterations--){
		numElements=0;
		std::unordered_set<int> inliers;
		while(inliers.size()<=3)
			inliers.insert(std::rand()%(cloud->points.size()));
		
		auto itr=inliers.begin();
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
        z1=cloud->points[*itr].z;
		++itr;
		x2=cloud->points[*itr].x;
		y2=cloud->points[*itr].y;
        z2=cloud->points[*itr].z;
        ++itr;
		x3=cloud->points[*itr].x;
		y3=cloud->points[*itr].y;
        z3=cloud->points[*itr].z;

        /* Step 1: find the normal vector to the plane

            Use point1 as a reference and define two vectors on the plane vec_1_2 and vec_1_3 as follows:

            Vector vec_1_2 travels from point1 to point2.
            Vector vec_1_3 travels from point1 to point3
            vec_1_2 = < x2 - x1, y2 - y1, z2 - z1 >
            vec_1_3 = < x3 - x1, y3 - y1, z3 - z1 >

            Find normal vector to the plane by taking cross product of vec_1_2 and vec_1_3:
            vecNormal=vec_1_2 x vec_1_3 = <(y2-y1)(z3-z1)-(z2-z1)(y3-y1),
            (z2-z1)(x3-x1)-(x2-x1)(z3-z1),(z2−z1)(x3−x1)−(x2−x1)(z3−z1),
            (x2-x1)(y3-y1)-(y2-y1)(x3-x1)>(x2−x1)(y3−y1)−(y2−y1)(x3−x1)>

            To simplify notation we can write it in the form
            vecNormal = < i, j, k >

            A=i,

            B = j,

            C = k,

            D = -( ix1 + jy1 + kz1)



        */
        auto i= (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        auto j= (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        auto k= (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		
        a=i;
		b=j;
		c=k;
        d=-(i*x1 + j*y1 + k*z1);

        /*  Step2: claculate the distance of each point to the fitted plane

            d=∣A∗x+B∗y+C∗z+D∣/sqrt(A*A +B*B + C*C )

        */

		for(int idx=0; idx<cloud->points.size();++idx){
			float xPnt=cloud->points[idx].x;
			float yPnt=cloud->points[idx].y;
            float zPnt=cloud->points[idx].z;

			float distance=std::abs(a*xPnt + b*yPnt +c*zPnt +d);
			distance/=std::sqrt(a*a + b*b + c*c);
			if(distance<=distanceThreshold)
				inliers.insert(idx);

		}
		numElements=inliers.size();
		if(numElements > maxNumberOfPoints){
			maxNumberOfPoints=numElements;
			inliersResult=inliers;
		}

        inliersRes->indices=std::vector<std::int32_t>(inliersResult.begin(),inliersResult.end());
	

	}
		
	
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersRes,cloud);
    
    return segResult;
}

// PCL implementation for clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // DONE:: Fill in the function to perform euclidean clustering to group detected obstacles

    // making KDtree to search for the neighbors
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for(auto& item:cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr ptrCloud(new pcl::PointCloud<PointT>());
        for(auto idx:item.indices){
            ptrCloud->points.push_back(cloud->points[idx]);
        }
        ptrCloud->is_dense=true;
        ptrCloud->width=ptrCloud->size();
        ptrCloud->height=1;

        clusters.push_back(ptrCloud);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// OWN implementation for clustering
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize,bool flag){
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree* tree = new KdTree(3);
    std::vector<std::vector<float>> vecCloud;
    for(int i=0;i<cloud->points.size();i++){

        std::vector<float> pnt{cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
        tree->insert(pnt,i);
        vecCloud.push_back(pnt);

    }

    std::vector<std::vector<int>> cluster_indices=euclideanCluster(vecCloud,  tree, clusterTolerance);
    
    for(auto& item:cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr ptrCloud(new pcl::PointCloud<PointT>());
        for(auto idx:item){
            ptrCloud->points.push_back(cloud->points[idx]);
        }
        ptrCloud->is_dense=true;
        ptrCloud->width=ptrCloud->size();
        ptrCloud->height=1;

        clusters.push_back(ptrCloud);
    }




    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}