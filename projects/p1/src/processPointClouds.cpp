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

/* Reduce point cloud size using Voxel Grid and Region of Interest filtering and remove LiDAR
 * static points on egoCar roof.
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

  // Using default XYZI min/max values provided in the Nanodegree (these values are slightly
  // narrowed down when displaying egoCar box in custom/options.h)
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
  extract.setNegative(true);  // true: discard roof points; false: keep only roof points and discard the rest
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
 * Extended from Aaron Brown's solution to RANSAC 2D problem [1].
 *
 * Resources:
 * [1] - Implementing RANSAC for Lines (Point Cloud Segmentation: Lesson 7), Udacity Sensor Fusion Nanodegree
 * [2] - Extending RANSAC to Planes (Point Cloud Segmentation: Lesson 8), Udacity Sensor Fusion Nanodegree
 * [3] - https://en.wikipedia.org/wiki/Random_sample_consensus
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

    // Find coefficients of general equation of a plane Ax + By + Cz + D = 0 via cross-product [2]
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
 * Also see: "custom/kdtree3d.h::render3DTree"
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

/* Perform Euclidean clustering on a point cloud object with the help of a 3-dimensional KD-Tree [1] [2] [3].
 * In contrast with the starting code version, this method also requires a KD-Tree as argument (see "CreateKdTree").
 *
 * Resources:
 * [1] - Insert Points (Clustering Obstacles: Lesson 6), Udacity Sensor Fusion Nanodegree
 * [2] - Searching Points in a KD-Tree (Clustering Obstacles: Lesson 7), Udacity Sensor Fusion Nanodegree
 * [3] - Euclidean Clustering (Clustering Obstacles: Lesson 8), Udacity Sensor Fusion Nanodegree
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

/* Sort Eigenvector columns descendingly based on range magnitude across the three dimensions.
 */
template<typename PointT>
void ProcessPointClouds<PointT>::sortEigenvectors(typename pcl::PointCloud<PointT>::Ptr cluster, Eigen::Matrix3f &eigenvectors)
{
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Eigen::Vector3f ranges {abs(maxPoint.x - minPoint.x), abs(maxPoint.y - minPoint.y), abs(maxPoint.z - minPoint.z)};

  std::vector<int> index {0, 1, 2};

  // Sorting and permutation logic suggested by Udacity GPT
  std::sort(index.begin(), index.end(), [&](int a, int b) { return ranges[a] > ranges[b]; });

  Eigen::Vector3i sortedIndex = Eigen::Map<Eigen::Vector3i> (index.data());

  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> permutationMatrix (eigenvectors.cols());
  permutationMatrix.setIdentity();
  permutationMatrix.indices() = sortedIndex;

  eigenvectors = eigenvectors * permutationMatrix;
}

/* Find the smallest obstacle-fitting bounding boxes aligned with the XY plane.
 * 
 * PCA minimum bounding boxes are fitted around clusters of points using Ryan McCormick's methodology,
 * [1] [2], then flattened (roll, pitch = 0) keeping their original Z-axis orientation (yaw).
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
  // column is associated with the dimension showing largest variation in the data
  Eigen::Matrix3f V = svd.matrixV();

  // Also extract singular values to test for almost meaningless dimensions
  Eigen::Vector3f S = svd.singularValues();

  // Largely uni-dimensional clusters such as the side pole in City Block tend to display "cross" patterns
  // (vertical point cloud, horizontal bounding box) because ranges for X and Y vary (affecting the sorting
  // of the associated eigenvectors) and related singular values are scarcely significant. A correction is
  // therefore applied in this case, so that eigenvectors are sorted in ZYX order (2, 1, 0).
  // Note that trimming the range (e.g., removing outliers or considering data within n standard deviations)
  // does not always help obtain the correct order for the eigenvectors.

  float eps = 0.01;
  int negligibleDimensions = (S.array() < eps).count();

  if (negligibleDimensions < 2)
    // Largely tri-dimensional objects: sort eigenvectors according to spread of data across dimensions
    sortEigenvectors(cluster, V);

  else
    // Apply correction: ZYX
    V = V.rowwise().reverse().eval();

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

  rotateZ = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  rotateY = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
  rotateX = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

  // Reconstruct the original ZYX rotation as a quaternion composition, without pitch and roll
  Eigen::Matrix3f rotatedMatrix = rotateZ * rotateY * rotateX;
  Eigen::Quaternionf bboxQuaternionXyAligned (rotatedMatrix);

  boxQ.bboxQuaternion = bboxQuaternionXyAligned;

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
  std::cerr << "\nLoaded " << cloud->points.size () << " data points from " + file << std::endl;

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