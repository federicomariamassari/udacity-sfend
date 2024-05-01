#ifndef PROCESS_POINT_CLOUD_H
#define PROCESS_POINT_CLOUD_H

#include <algorithm>  // To sort bounding boxes based on associated cars' positions (x, y)
#include <cmath>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "../render/box.h"

/**
 * @brief Perform voxel (volumetric pixel) grid filtering on an input point cloud [1] [2].
 * 
 * @param cloud The input cloud.
 * @param filtered The output filtered cloud, to populate.
 * @param filterRes The voxel cube side (0.01f = 1 cm).
 * 
 * Resources:
 * 
 * [1] - https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html
 * [2] - https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/src/processPointClouds.cpp#L23
 */
template<typename PointT>
void filterCloud(const typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr filtered, 
  float filterRes);

/**
 * @brief Cluster an input point cloud using Euclidean clustering [1] [2].
 * 
 * @param cloud The input cloud, likely the outcome of filterCloud.
 * @param clusters Structure containing the output clusters, to populate.
 * @param clusterTol Maximum distance between two points for them to be considered part of the same cluster.
 * @param minSize Minimum size of each cluster.
 * @param maxSize Maximum size of each cluster.
 * 
 * Resources:
 * 
 * [1] - https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html
 * [2] - Lesson 3: Euclidean Clustering with PCL, Clustering Obstacle, LiDAR course, Udacity Sensor Fusion Nanodegree
 */
template<typename PointT>
void euclideanClustering(const typename pcl::PointCloud<PointT>::Ptr cloud, 
  std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters, float clusterTol, int minSize, int maxSize);

/**
 * @brief Render point cloud clusters cycling through three different colors [1].
 * 
 * @param viewer The PCL visualizer object.
 * @param clusters Structure containing point cloud clusters.
 * 
 * Resources:
 * 
 * [1] - https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/src/environment.cpp#L238
 */
template <typename PointT>
void renderClusters(pcl::visualization::PCLVisualizer::Ptr& viewer,
  const std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters);

/**
 * @brief Helper function to compute the norm of a vector using (x, y) coordinates.
 * 
 * @param car A car object.
 * @param box A bounding box framing a point cloud cluster.
 */
float norm(const Car& car, const Box& box);

/**
 * @brief Match bounding boxes with the right car, to avoid LiDAR markers switching from box to box and confusing
 *   predictions. This is a necessary step since Euclidean clustering does not preserve box-to-car associations when 
 *   generating clusters and pushing them into a vector.
 * 
 * @param traffic The structure containing car objects.
 * @param boxes The structure containing (unsorted) bounding boxes.
 */
void sortBoundingBoxes(const std::vector<Car>& traffic, std::vector<Box>& boxes);

#endif /* PROCESS_POINT_CLOUD_H */