/* PCL library functions for point cloud processing.
 */
#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <algorithm>
#include <ctime>
#include <chrono>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include "render/box.h"

// Custom implementations of KD-Tree 3D and Euclidean clustering (extended from Aaron Brown's 2D solutions)
#include "custom/kdtree3d.h"
#include "custom/clustering.h"

template<typename PointT>
class ProcessPointClouds {

  public:

    // Constructor
    ProcessPointClouds();

    // Deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, 
      float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
      SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
      SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    // Separate KD-Tree creation logic from method 'Clustering' to enable 3D rendering of the tree
    KdTree<PointT>* CreateKdTree(typename pcl::PointCloud<PointT>::Ptr cloud);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, 
      KdTree<PointT>* tree, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    // Basic 3D quaternion rotations (to align object to XY-plane preserving Z rotation)
    Eigen::Quaternionf axisRotate(float angle, char axis);

    BoxQ MinimumXyAlignedBoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};

#endif /* PROCESSPOINTCLOUDS_H_ */