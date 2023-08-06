/* PCL library functions for point cloud processing.
 */
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

/* Reduce point cloud size using Voxel Grid and Region of Interest filtering.
 * Remove LiDAR static points on egoCar roof.
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, 
  float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
  // Time the segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::cout << "Point Cloud size before filtering: " << cloud->points.size() << std::endl;

  // Voxel grid point reduction [https://pcl.readthedocs.io/en/latest/voxel_grid.html]
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

  // Create the filtering object
  pcl::VoxelGrid<PointT> voxelGrid;
  voxelGrid.setInputCloud(cloud);

  // Each voxel (volumetric pixel) is a cube with side length filterRes (.01f = 1cm)
  voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
  voxelGrid.filter(*cloudFiltered);

  // Region of interest filtering [https://pointclouds.org/documentation/classpcl_1_1_crop_box.html]
  typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

  pcl::CropBox<PointT> regionOfInterest (true);  // true to extract indices of removed points

  regionOfInterest.setMin(minPoint);
  regionOfInterest.setMax(maxPoint);
  regionOfInterest.setInputCloud(cloudFiltered);
  regionOfInterest.filter(*cloudRegion);

  // Remove LiDAR static points on egoCar roof
  std::vector<int> indices;

  pcl::CropBox<PointT> roof (true);

  // Using default XYZI min/max values provided in the Nanodegree
  roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

  for (int point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);  // To actually remove the roof points
  extract.filter(*cloudRegion);

  std::cout << "Point Cloud size after filtering: " << cloudRegion->points.size() << std::endl;

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  std::cout << "Filtering took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;

  return cloudRegion;
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

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult (obstacles, plane_cloud);
  return segResult;
}

/* Perform RANSAC 3D segmentation.
 * Extended from Aaron Brown's solution to RANSAC 2D problem.
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
  ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  while (maxIterations--)
  {
    // Randomly select three distinct point indices from the cloud to define a plane
    std::unordered_set<int> inliers;

    while (inliers.size() < 3)
      inliers.insert(rand() % cloud->points.size());

    auto itr = inliers.begin();

    // Store the plane inlier points based on extracted indices
    std::vector<PointT> p (3);

    for (int i=0; i < p.size(); i++)
    {
      p[i] = cloud->points[*itr];
      itr++;
    }

    // Calculate the coefficients of plane equation Ax + By + Cz + D = 0 via cross product
    float A, B, C, D;
    A = (p[1].y - p[0].y) * (p[2].z - p[0].z) - (p[1].z - p[0].z) * (p[2].y - p[0].y);
    B = (p[1].z - p[0].z) * (p[2].x - p[0].x) - (p[1].x - p[0].x) * (p[2].z - p[0].z);
    C = (p[1].x - p[0].x) * (p[2].y - p[0].y) - (p[1].y - p[0].y) * (p[2].x - p[0].x);
    D = -(A*p[0].x + B*p[0].y + C*p[0].z);

    for (int i=0; i < cloud->points.size(); i++) {

      // Ignore points which are already inliers to the plane
      if (inliers.count(i) > 0)
        continue;

      // Check if current point is inlier to the plane
      PointT point = cloud->points[i];

      float distance = fabs(A*point.x + B*point.y + C*point.z + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));

      // If so, add current point to the list of inliers
      if (distance <= distanceThreshold)
        inliers.insert(i);
    }

    // The plane with the most inliers is the best model
    if (inliers.size() > inliersResult.size())
      inliersResult = inliers;
  }

  if (inliersResult.size() == 0) {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // Cast inliersResult to suitable input for SeparateClouds [below 4 lines suggested by Udacity GPT]
  pcl::PointIndices::Ptr inliersIndices {new pcl::PointIndices};
  inliersIndices->indices.reserve(inliersResult.size());

  for (const auto& inlier : inliersResult)
    inliersIndices->indices.push_back(inlier);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  std::cout << "Plane segmentation took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    segResult = SeparateClouds(inliersIndices, cloud);

  return segResult;
}

/* Create a 3-dimensional KD-Tree.
 * This method encapsulates logic from 'Clustering' to ease 3D-rendering of the object.
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
 * In contrast to the starting code version, this method also requires a KD-Tree as argument.
 */
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
  ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, 
    float clusterTolerance, int minSize, int maxSize)
{
  // Time the clustering process
  auto startTime = std::chrono::steady_clock::now();

  // Euclidean clustering
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = euclideanCluster(cloud, tree, clusterTolerance, 
    minSize, maxSize);

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

/* Find the minimum bounding box for a 3D point cloud
 * From: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
 */
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::MinimumBoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
  BoxQ boxQ;

  // Center data around the origin to ensure first principal component captures direction of maximum variance
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);

  // Compute variance-covariance matrix of mean deviations
  Eigen::Matrix3f varcov;
  computeCovarianceMatrixNormalized(*cluster, centroid, varcov);

  // Eigendecomposition
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(varcov, Eigen::ComputeEigenvectors);

  Eigen::Matrix3f eigenVectorsPCA = eigenSolver.eigenvectors();

  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  Eigen::Matrix4f projectionTransform (Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());

  typename pcl::PointCloud<PointT>::Ptr pointCloudProjected (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cluster, *pointCloudProjected, projectionTransform);

  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*pointCloudProjected, minPoint, maxPoint);

  const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  const Eigen::Quaternionf bboxQuaternion (eigenVectorsPCA);
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + centroid.head<3>();

  boxQ.bboxQuaternion = bboxQuaternion;
  boxQ.bboxTransform = bboxTransform;

  boxQ.cube_length = maxPoint.x - minPoint.x;
  boxQ.cube_width = maxPoint.y - minPoint.y;
  boxQ.cube_height = maxPoint.z - minPoint.z;

  return boxQ;
}

/* Find the smallest obstacle-fitting bounding box oriented flat within the XY plane.
 * Modified from: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
 */
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::MinimumXyAlignedBoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
  BoxQ boxQ;

  // TODO


  return boxQ;
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
  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, 
    boost::filesystem::directory_iterator{});

  // Sort files in ascending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}