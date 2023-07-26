// PCL library functions for point cloud processing
#include "processPointClouds.h"

// Constructor
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// Destructor
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  std::cout << cloud->points.size() << std::endl;
}

/* Reduce point cloud size.
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, 
  float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
  // Time the segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  // https://pcl.readthedocs.io/en/latest/voxel_grid.html
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT> ());

  std::cout << "Point Cloud size before filtering: " << cloud->points.size() << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;

  sor.setInputCloud(cloud);

  // Each voxel is a cube with side filterRes (0.01f = 1 cm long)
  sor.setLeafSize(filterRes, filterRes, filterRes);

  // Set filter limits for the points coordinates x, y, z [suggested by Udacity GPT]
  sor.setFilterLimits(minPoint[0], maxPoint[0]);
  sor.setFilterLimits(minPoint[1], maxPoint[1]);
  sor.setFilterLimits(minPoint[2], maxPoint[2]);

  sor.filter(*cloudFiltered);

  std::cout << "Point Cloud size after filtering: " << cloudFiltered->points.size() << std::endl;

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  std::cout << "Filtering took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;

  return cloudFiltered;
}

/* Separate inliers (points on a plane, or road) from obstacles.
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
  ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT> ());

  for (int index : inliers->indices)
    // Add inlier indices to segmented plane vector
  	plane_cloud->points.push_back(cloud->points[index]);

  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacles);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane_cloud);
  return segResult;
}

/* Perform RANSAC 3D segmentation.
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
  ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  while (maxIterations--) {

    // Randomly select three distinct points from the cloud to define a plane
    std::unordered_set<int> inliers;

    while (inliers.size() < 3)
      inliers.insert(rand() % cloud->points.size());

    // Store the inliers in a vector
    std::vector<PointT> p {3};

    auto itr = inliers.begin();

    for (int i=0; i < p.size(); i++)
    {
      p[i] = cloud->points[*itr];
      itr++;
    }

    // Calculate the coefficients of plane equation Ax + By + Cz + D = 0 via cross product
    float a = (p[1].y-p[0].y) * (p[2].z-p[0].z) - (p[1].z-p[0].z)*(p[2].y-p[0].y);
    float b = (p[1].z-p[0].z) * (p[2].x-p[0].x) - (p[1].x-p[0].x)*(p[2].z-p[0].z);
    float c = (p[1].x-p[0].x) * (p[2].y-p[0].y) - (p[1].y-p[0].y)*(p[2].x-p[0].x);
    float d = -(a*p[0].x + b*p[0].y + c*p[0].z);

    for (int i=0; i < cloud->points.size(); i++) {

      // Ignore points which are already inliers to the plane
      if (inliers.count(i) > 0)
        continue;

      // Check if current point is inlier to the plane
      PointT point = cloud->points[i];

      float dist = fabs(a*point.x + b*point.y + c*point.z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));

      if (dist <= distanceThreshold)
        inliers.insert(i);
    }

    // The plane with the most inliers is the best model
    if (inliers.size() > inliersResult.size())
      inliersResult = inliers;
  }

  if (inliersResult.size() == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // Cast inliersResult to suitable input for SeparateClouds [method suggested by Udacity GPT]
  pcl::PointIndices::Ptr inliersIndices {new pcl::PointIndices};
  inliersIndices->indices.reserve(inliersResult.size());

  for (const auto& inlier : inliersResult)
    inliersIndices->indices.push_back(inlier);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  std::cout << "Plane segmentation took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersIndices, cloud);
  return segResult;
}

/* Create a 3-dimensional KD-Tree.
   This method encapsulates logic from 'Clustering' to ease 3D-rendering of the object.
 */
template<typename PointT>
KdTree<PointT>* ProcessPointClouds<PointT>::CreateKdTree(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // Time the KD-Tree generation process
  auto startTime = std::chrono::steady_clock::now();

  // Initialize 3-dimensional KD-Tree on the heap
  KdTree<PointT>* tree = new KdTree<PointT>;

  for (int i=0; i < cloud->points.size(); i++)
    tree->insert(cloud->points[i], i);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  std::cout << "KD-Tree generation took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;

  return tree;
}

/* Perform Euclidean clustering on a point cloud object with the help of a 3-dimensional KD-Tree.
   Differently from the starting code version, this method also requires a KD-Tree as argument.
 */
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, 
  KdTree<PointT>* tree, float clusterTolerance, int minSize, int maxSize)
{
  // Time the clustering process
  auto startTime = std::chrono::steady_clock::now();

  // Euclidean clustering
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  std::cout << "Clustering took " << elapsedTime.count() / 1000. << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
  std::cerr << "Saved " << cloud->points.size () << " data points to " + file << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size () << " data points from " + file << std::endl;

  return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

  // Sort files in ascending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}