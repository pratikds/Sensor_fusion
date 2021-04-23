// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
/*  adding additional headerfiles */
#include "quiz/cluster/cluster.h"
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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Experiment with the ? values and find what works best
  
    typename pcl::PointCloud<PointT>::Ptr voxel_grid_filtered{
      new pcl::PointCloud<PointT>};
  
    typename pcl::VoxelGrid<PointT> vg;
    //typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*voxel_grid_filtered);
  
    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};
  
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(voxel_grid_filtered);
    region.filter(*cloudRegion);
  
  
    // to remove roof top pixels
    std::vector<int> indices;
    
    typename pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.8,1.7,-.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
  
     //removing the roof points but first need the indices that is to be removed
  
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
         inliers->indices.push_back(point);
    
    // extraction pool to remove the indices
  
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*cloudRegion);
  
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_UD(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // kd tree
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    for (int i=0; i<cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        
        std::vector<float> point_vector;
        point_vector.push_back(point.x);
        point_vector.push_back(point.y);
        point_vector.push_back(point.z);

        tree->insert(point_vector, i); 
        points.push_back(point_vector);
    }

    // cluster
    std::vector<std::vector<int>> clusters_idx = euclideanCluster(points, tree, clusterTolerance);

    for(std::vector<int> cluster_idx : clusters_idx)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for (int indice: cluster_idx)
        {
            clusterCloud->points.push_back(cloud->points[indice]);
        }
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        if ((clusterCloud->width >= minSize) and (clusterCloud->width <= maxSize))
            clusters.push_back(clusterCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
      
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacle (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

    // get cloud_plane
    for (int index: inliers->indices) {
        cloud_plane->points.push_back(cloud->points[index]);
    }

    // get cloud_obstacle
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_obstacle);
     
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}




template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); 
    
    // creating segmentation object
  
    pcl::SACSegmentation<PointT> seg;

    // Set up the options
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);  // Set up the point cloud

    // Perform RANSAC to find the plane for the point cloud (i.e. the road)
    seg.segment(*inliers, *coefficients);
  
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
  
      std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  
    /* using PCL memthod */
  
    //if (!customclustering){
  
      typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
      tree->setInputCloud(cloud);
      std::vector<pcl::PointIndices> clusterIndices;
      pcl::EuclideanClusterExtraction<PointT> ec;
      ec.setClusterTolerance(clusterTolerance);
      ec.setMinClusterSize(minSize);
      ec.setMaxClusterSize(maxSize);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud);
      ec.extract(clusterIndices);
   
      for (pcl::PointIndices getIndices: clusterIndices)
      {
    
        typename pcl::PointCloud<PointT>::Ptr cloudCluster {new pcl::PointCloud<PointT>};
        for (int i : getIndices.indices)
        {
          cloudCluster->points.push_back(cloud->points[i]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1 ;
        cloudCluster->is_dense = true; 
      
        clusters.push_back(cloudCluster);  
     
      } 
    //}
 /* 
     else {
 
  
    // kdTree object
  
     KdTree* tree = new KdTree;
    
     // vector for storage to be compatible with custom Eucleadian Clustering Method
   
     std::vector<std::vector<float>> vector_points;
    
  
     // inserting points inside the Kd Tree
  
  
     for ( int i =0; i < cloud->points.size() ; i++)
     {
        const std::vector<float> pt = { cloud->points[i].x , cloud->points[i].y, cloud->points[i].z};
        tree->insert(pt,i);
        vector_points.push_back(pt);
      
    
     }
     // get the cluster Ids
  
     std::vector<std::vector<int>> clusterskdtree = euclideanCluster(vector_points, tree, clusterTolerance);
  
     delete tree;
     tree = nullptr;
  
      // ensure the size of the cluster within min max range
  
      for (const auto& clust : clusterskdtree) {
        if (clust.size() >= minSize && clust.size() <= maxSize) {
          // Create new pointer to point cloud object of type PointT
          typename pcl::PointCloud<PointT>::Ptr cloud_cluster{new pcl::PointCloud<PointT>};

          // For each index, access the point in the point cloud and
          // and add to the list of points
          for (const auto index : clust) {
            cloud_cluster->points.push_back(cloud->points[index]);
          }
          cloud_cluster->width = cloud_cluster->points.size();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

           // Add to the final output
          clusters.push_back(cloud_cluster);
        }
      }
    } 
*/
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>  
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{

    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;

    // ransac
    while (maxIterations--) 
    {
        // Randomly sample subset and fit line
        std::unordered_set<int> curr_inliers; 
        while (curr_inliers.size() != 3) {
          curr_inliers.insert(rand() % cloud->points.size());
        }

        auto itr = curr_inliers.begin();
        float x1 = cloud->points[*itr].x;
        float y1 = cloud->points[*itr].y;
        float z1 = cloud->points[*itr].z;
        itr++;
        float x2 = cloud->points[*itr].x;
        float y2 = cloud->points[*itr].y;
        float z2 = cloud->points[*itr].z;
        itr++;
        float x3 = cloud->points[*itr].x;
        float y3 = cloud->points[*itr].y;
        float z3 = cloud->points[*itr].z;

        // plane params
        float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        float d = - (a*x1 + b*y1 + c*z1);

        // Measure distance between every point and fitted line
        for (int i = 0; i < cloud->points.size(); ++i)
        {
          if (curr_inliers.count(i) > 0)
            continue;

          float nx = cloud->points[i].x;
          float ny = cloud->points[i].y;
          float nz = cloud->points[i].z;
          float dist = fabs(a * nx + b * ny + c * nz + d) / sqrt(a * a + b * b + c * c);

          if (dist < distanceTol)
            curr_inliers.insert(i);
        }

        if (curr_inliers.size() > inliersResult.size())
          inliersResult = curr_inliers;
    }

    // inliersResult
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;

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