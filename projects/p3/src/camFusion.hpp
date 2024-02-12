#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
#include <vector>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>  // FP.2: To compute Euclidean clustering with FLANN
#include <opencv2/features2d.hpp>  // FP.4: To visualize keypoints overlay (camera TTC visualization)
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz.hpp>  // FP.2: To visualize LiDAR point cloud clusters in 3D

#include "dataStructures.h"


/**
 * @brief Create groups of LiDAR points whose projection into the camera falls into the same bounding box.
 * 
 * @param boundingBoxes Set of bounding boxes detected in a particular image.
 * @param lidarPoints Detected 3D LiDAR points for a particular image.
 * @param shrinkFactor Bounding box shrink factor to avoid 3D-object merging at the edges of an ROI.
 * @param P_rect_xx Intrinsic parameter matrix K (3x4 projection matrix after rectification).
 * @param R_rect_xx 3x3 rectifying rotation matrix to make image planes co-planar.
 * @param RT Extrinsic parameters (rotation matrix and translation vector).
 */
void clusterLidarWithROI(std::vector<BoundingBox>& boundingBoxes, std::vector<LidarPoint>& lidarPoints, 
  float shrinkFactor, cv::Mat& P_rect_xx, cv::Mat& R_rect_xx, cv::Mat& RT);

/**
 * @brief
 * 
 * @param 
 * @param 
 * @param 
 * @param 
 */
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait=true);

/**
 * @brief FP.1: Match consecutive bounding box pairs by the largest number of keypoint correspondences [1].
 * 
 * @param matches The keypoint matches between previous (query) and current (train) frames.
 * @param bbBestMatches Container of previous-current bounding box IDs that result in the best matches, to populate.
 * @param prevFrame Sensor information at previous (query) time.
 * @param currFrame Sensor information at current (train) time.
 * 
 * Resources:
 * 
 * [1] - https://knowledge.udacity.com/questions/570553 (additional refactoring suggested by Udacity GPT)
 * [2] - https://docs.opencv.org/4.2.0/d4/de0/classcv_1_1DMatch.html
 * [3] - Estimating TTC with a Camera (Lesson 3: Engineering a Collision Detection System), Udacity Sensor Fusion
 */
void matchBoundingBoxes(std::vector<cv::DMatch>& matches, std::map<int, int>& bbBestMatches, DataFrame& prevFrame, 
  DataFrame& currFrame);

/**
 * @brief FP.2, FP.4: Linearly interpolate percentile q on a sorted container, using method R7 [1].
 * 
 * @param The input container.
 * @param q The desired percentile in [0; 1].
 * 
 * Resources:
 * 
 * [1] - https://axibase.com/use-cases/workshop/percentiles.html#r7-linear-interpolation
 * [2] - https://en.wikipedia.org/wiki/Percentile
 * [3] - https://en.wikipedia.org/wiki/Median
 */
double percentile(const std::vector<double>& vec, const float q);

/**
 * 
 */
double mean(const std::vector<double>& vec);

/**
 * FP.2: List of filtering methods to remove outliers from a cloud of 3-dimensional points.
 * 
 * TUKEY: Applies Tukey's fences [1] to the x-coordinates of the input data.
 * EUCLIDEAN_CLUSTERING: Applies Euclidean clustering (L2-norm) on the input data removing isolated points [2].
 * 
 * Resources:
 * 
 * [1] - https://en.wikipedia.org/wiki/Outlier
 * [2] - https://knowledge.udacity.com/questions/296395
 */
enum class FilteringMethod
{
  TUKEY,
  EUCLIDEAN_CLUSTERING
};

/**
 * @brief FP.2: Render a 3-dimensional LiDAR point cloud in OpenCV (code suggested by Udacity GPT).
 * 
 * @param src The input LiDAR point cloud data.
 * @param clusters The populated Euclidean clusters container.
 * @param bShowRemoved Whether to also render discarded clusters (in white). 
 * 
 * Resources:
 * 
 * [1] - https://docs.opencv.org/4.2.0/d4/dba/classcv_1_1viz_1_1Color.html
 */
void renderClusters(const std::vector<LidarPoint>& src, const std::vector<std::set<int>>& clusters, 
  bool bShowRemoved=true);


void printStatistics(const std::vector<LidarPoint>& src, const std::vector<std::set<int>>& clusters, 
  const std::vector<std::set<int>>& removed, float radius, int minSize, int maxSize);

/**
 * @brief FP.2: Recursive function to compute the nearest neighbors of a query point based on input L2-norm [1] [2].
 * 
 * @param index Id of the query point.
 * @param cloud The input LiDAR point cloud data.
 * @param cluster The cluster instance, to populate.
 * @param processed Container to mark visited query points.
 * @param tree FLANN KD-Tree structure.
 * @param radius Distance tolerance to query point for the neighborhood search; will be squared (L2-norm).
 * @param knn No. of points to include at each radius search. Set k > 3 to ensure a least one new point is included
 *            at each radius iteration, but k too large will greatly increase computational time.
 * @param minSize Minimum acceptable size for a cluster; smaller ones will be discarded.
 * @param maxSize Maximum acceptable size for a cluster; larger ones will be split.
 * 
 * Resources:
 * 
 * [1] - Lidar Obstacle Detection: Euclidean Clustering (Clustering Obstacles: Lesson 8), Udacity Sensor Fusion
 * [2] - https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/src/custom/clustering.h
 */
template<typename T>
void clusterHelper(int index, const cv::Mat& cloud, std::set<int>& cluster, std::vector<bool>& processed, 
  cv::flann::GenericIndex<T>& tree, float radius, int knn, int minSize, int maxSize);

/**
 * @brief FP.2: Cluster input data based on the Euclidean distance (L2-norm) from a query point [1].
 * 
 * @param src The input LiDAR point cloud data.
 * @param clusters The cluster container, to populate.
 * @param removed The removed clusters container, to also populate.
 * @param radius Distance tolerance to query point for the neighborhood search; will be squared (L2-norm).
 * @param knn No. of points to include at each radius search (see clusterHelper for additional info).
 * @param minSize Minimum acceptable size for a cluster; smaller ones will be discarded.
 * @param maxSize Maximum acceptable size for a cluster; larger ones will be split.
 * 
 * Resources:
 * 
 * [1] - https://knowledge.udacity.com/questions/296395
 * [2] - https://docs.opencv.org/4.2.0/db/d18/classcv_1_1flann_1_1GenericIndex.html
 * [3] - https://vovkos.github.io/doxyrest-showcase/opencv/sphinxdoc/struct_cvflann_KDTreeSingleIndexParams.html
 * [4] - https://github.com/opencv/opencv/issues/12683
 * [5] - https://vovkos.github.io/doxyrest-showcase/opencv/sphinxdoc/struct_cvflann_SearchParams.html
 * [6] - https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/src/custom/clustering.h
 */
void euclideanClustering(const std::vector<LidarPoint>& src, std::vector<std::set<int>>& clusters, 
  std::vector<std::set<int>>& removed, float radius, int knn, int minSize, int maxSize);

/** 
 * @brief FP.2: Filter out outliers from a cloud of 3-dimensional points based on the desired filtering method.
 * 
 * @param src The input LiDAR point cloud data.
 * @param dst The filtered LiDAR point cloud data, to populate.
 * @param method The desired filtering method.
 * 
 * Euclidean clustering -only parameters:
 * 
 * @param radius Distance tolerance to query point for the neighborhood search; will be squared (L2-norm).
 * @param knn No. of points to include at each radius search.
 * @param minSize Minimum cluster size. Clusters smaller than this value will be discarded as outliers.
 * @param maxSize Maximum cluster size. Clusters larger than this value will be broken down.
 * @param bRenderClusters Whether to display 3D rendering of clustered LiDAR points.
 * @param bShowRemoved Whether to include colorless outlier points in the rendering view.
 * 
 * Resources:
 * 
 * [1] - https://en.wikipedia.org/wiki/Outlier
 * [2] - https://knowledge.udacity.com/questions/296395
 * [3] - https://docs.opencv.org/4.2.0/db/d18/classcv_1_1flann_1_1GenericIndex.html
 */
void removeOutliers(std::vector<LidarPoint>& src, std::vector<LidarPoint>& dst, FilteringMethod method, float radius,
  int knn, int minSize, int maxSize, bool bRenderClusters, bool bShowRemoved);

/**
 * @brief FP.2: Compute stable LiDAR time-to-collision (TTC) between consecutive frames assuming constant velocity [1].
 * 
 * @param lidarPointsPrev LiDAR point cloud data from the previous frame.
 * @param lidarPointsCurr LiDAR point cloud data from the current frame.
 * @param frameRate Time between two measurements, in seconds.
 * @param TTC LiDAR time-to-collision measure, to populate.
 * @param filteringMethod Outlier filtering method, either Tukey's fences [2] or Euclidean clustering [3].
 * 
 * Euclidean clustering -only parameters:
 * 
 * @param radius Distance tolerance to query point for the neighborhood search; will be squared (L2-norm).
 * @param knn No. of points to include at each radius search.
 * @param minSize Minimum cluster size. Clusters smaller than this value will be discarded as outliers.
 * @param maxSize Maximum cluster size. Clusters larger than this value will be broken down.
 * @param bRenderClusters Whether to display 3D rendering of clustered LiDAR points.
 * @param bShowRemoved Whether to include colorless outlier points in the rendering view.
 * 
 * Resources:
 * 
 * [1] - https://knowledge.udacity.com/questions/628925
 * [2] - https://en.wikipedia.org/wiki/Outlier
 * [3] - Lidar Obstacle Detection: Euclidean Clustering (Clustering Obstacles: Lesson 8), Udacity Sensor Fusion
 */
void computeTTCLidar(std::vector<LidarPoint>& lidarPointsPrev, std::vector<LidarPoint>& lidarPointsCurr, 
  double frameRate, double& TTC, FilteringMethod& filteringMethod, float radius, int knn, int minSize, int maxSize, 
  bool bRenderClusters=false, bool bShowRemoved=false);

/**
 * @brief FP.3: Associate a given bounding box with the keypoints it contains.
 * 
 * @param boundingBox A particular bounding box in the current frame.
 * @param kptsPrev Container of keypoints detected in the previous frame.
 * @param kptsCurr Container of keypoints detected in the current frame.
 * @param kptMatches Container of matches between the previous and the current frame.
 * 
 * Resources:
 * 
 * [1] - https://knowledge.udacity.com/questions/110934
 * [2] - https://docs.opencv.org/4.2.0/d2/de8/group__core__array.html
 * [3] - Estimating TTC with a Camera (Lesson 3: Engineering a Collision Detection System), Udacity Sensor Fusion
 * [4] - https://knowledge.udacity.com/questions/624666
 */
void clusterKptMatchesWithROI(BoundingBox& boundingBox, std::vector<cv::KeyPoint>& kptsPrev, 
  std::vector<cv::KeyPoint>& kptsCurr, std::vector<cv::DMatch>& kptMatches);

/**
 * brief FP.4: Compute stable LiDAR time-to-collision (TTC) between consecutive frames assuming constant velocity [1].
 * 
 * @param kptsPrev Container of keypoints detected in the previous frame.
 * @param kptsCurr Container of keypoints detected in the current frame.
 * @param kptMatches Container of matches between the previous and the current frame.
 * @param frameRate Time between two measurements, in seconds.
 * @param TTC Camera time-to-collision measure, to populate.
 * @param visImg ???
 */
void computeTTCCamera(std::vector<cv::KeyPoint>& kptsPrev, std::vector<cv::KeyPoint>& kptsCurr,
  std::vector<cv::DMatch> kptMatches, double frameRate, double& TTC, cv::Mat* visImg=nullptr);

#endif /* camFusion_hpp */
