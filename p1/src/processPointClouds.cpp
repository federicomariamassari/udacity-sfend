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
  extract.setNegative(true);  // true: discard roof points; false: keep only roof points
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

    // Find coefficients of general equation of a plane Ax + By + Cz + D = 0 via cross-product
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

  // Cast inliersResult to suitable input for SeparateClouds [below 4 lines were suggested by Udacity GPT]
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
  std::cout << "Clustering took " << elapsedTime.count() / 1000. << " milliseconds and found " << clusters.size() 
    << " clusters" << std::endl;

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

/* Compute unit quaternions scaled by desired Euler angle [1].
 * 
 * Resources:
 *  [1] - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 */
template<typename PointT>
Eigen::Quaternionf ProcessPointClouds<PointT>::axisRotate(float angle, char axis)
{
  Eigen::Quaternionf quaternion;

  switch(axis)
  {
    case 'x':
      quaternion = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX());
      break;

    case 'y':
      quaternion = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY());
      break;

    case 'z':
      quaternion = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ());
  }

  return quaternion;
}

/* Find the smallest obstacle-fitting bounding boxes aligned with the XY plane.
 * 
 * PCA minimum bounding boxes are fitted around clusters of points using Ryan McCormick's methodology,
 * [1] [2], then flattened (roll, pitch = 0) keeping their original Z-axis orientation (yaw).
 * Some correction is applied if required.
 * 
 * Resources:
 * [1] - http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
 * [2] - https://github.com/Frogee/SorghumReconstructionAndPhenotyping/blob/master/boundingBox.h
 * [3] - https://en.wikipedia.org/wiki/Principal_component_analysis
 * [4] - https://en.wikipedia.org/wiki/Singular_value_decomposition
 * [5] - https://en.wikipedia.org/wiki/Rotation_matrix
 * [6] - https://en.wikipedia.org/wiki/Affine_transformation
 * [7] - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 * [8] - https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
 *
 * In addition, all Udacity Sensor Fusion posts mentioning PCA boxes were consulted.
 * A special mention to Udacity GPT who greatly helped with suggestions along the process.
 */
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::MinimumXyAlignedBoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
  BoxQ boxQ;

  // Center data around the origin to ensure first principal component captures the direction of maximum
  // variance instead of the mean [3]
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);

  // Compute normalized variance-covariance matrix of mean deviations
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cluster, centroid, covariance);

  // Use Singular Value Decomposition (SVD) instead of the eigendecomposition suggested by [1]
  Eigen::JacobiSVD<Eigen::MatrixXf> svd (covariance, Eigen::ComputeThinV);
  
  // Extract matrix of right-singular vectors of the normalized VarCov matrix [4]
  // In Eigen, the columns of V are sorted descendingly by corresponding singular value, so the first
  // column is associated to the dimension with most significant variation in the data
  Eigen::Matrix3f V = svd.matrixV();

  // Also extract singular values to test for almost meaningless dimensions
  Eigen::Vector3f S = svd.singularValues();

  // V satisfies the orthogonality property A * A^T = I(3), but sometimes has determinant -1 (reflection) [5].
  // Convert to a valid rotation matrix by flipping the sign of the column associated to the smallest singular
  // value--the one related to the least significant axis of the transformation (source: Udacity GPT)

  if (V.determinant() < 0)
    // Flip the sign of the last column, since Eigen::JacobiSVD sorts columns descendingly by singular value
    V.col(2) *= -1;

  // Store rotation matrix and translation vector in an affine transformation matrix (augmented) [6]
  Eigen::Matrix4f affineTransformationMatrix (Eigen::Matrix4f::Identity());

  // Transpose V so that its most significant eigenvector becomes the top row in the rotation matrix.
  affineTransformationMatrix.block<3,3>(0,0) = V.transpose();

  // Invert the sign of the translation vector [1] when storing in the transformation matrix
  affineTransformationMatrix.block<3,1>(0,3) = -1.f * (affineTransformationMatrix.block<3,3>(0,0) * centroid.head<3>());

  // Project the original cluster onto the new basis
  typename pcl::PointCloud<PointT>::Ptr pointCloudProjected (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cluster, *pointCloudProjected, affineTransformationMatrix);

  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*pointCloudProjected, minPoint, maxPoint);

  const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  const Eigen::Quaternionf bboxQuaternion (V);
  const Eigen::Vector3f bboxTransform = V * meanDiagonal + centroid.head<3>();

  boxQ.bboxTransform = bboxTransform;
  boxQ.bboxQuaternion = bboxQuaternion;

  boxQ.cube_length = maxPoint.x - minPoint.x;
  boxQ.cube_width = maxPoint.y - minPoint.y;
  boxQ.cube_height = maxPoint.z - minPoint.z;

  // Do not flatten bi-dimensional vertical objects (e.g., poles) to avoid "cross" effect (vertical
  // point cloud, horizontal bounding box)
  int irrelevantDimensions = 0;
  float eps = 5e-3;

  if (S.x() > eps && S.y() > eps && S.z() > eps)
  {
    // All dimensions are relevant; align the minimum bounding boxes to the XY-plane

    // Extract rotation matrix from quaternion [7]
    Eigen::Matrix3f rotationMatrix = boxQ.bboxQuaternion.toRotationMatrix();

    // Extract Euler angles in ZYX order: yaw, pitch, roll [7]
    Eigen::Vector3f euler = rotationMatrix.eulerAngles(2, 1, 0);

    // Set target roll (X) and pitch (Y) to 0 to align the boxes on the XY-plane  
    float yaw = euler[0];
    float pitch = 0.0;
    float roll = 0.0;

    // Generate basic 3D rotation matrices using axis-angle representation [8]
    Eigen::Matrix3f rotateZ, rotateY, rotateX;

    rotateZ = axisRotate(yaw, 'z');
    rotateY = axisRotate(pitch, 'y');
    rotateX = axisRotate(roll, 'x');

    // Reconstruct the original ZYX rotation as a quaternion product, but without pitch and roll
    Eigen::Matrix3f rotatedMatrix = rotateZ * rotateY * rotateX;
    Eigen::Quaternionf bboxQuaternionXyAligned (rotatedMatrix);

    boxQ.bboxQuaternion = bboxQuaternionXyAligned;
  }

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

  if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) // Load the file
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