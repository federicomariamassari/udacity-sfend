#ifndef CAMFUSION_HPP
#define CAMFUSION_HPP

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
 * @brief Display LiDAR top-view perspective with superimposed cluster statistics.
 * 
 * The function below can handle different output image sizes, but the text output has been manually tuned to fit the 
 * 2000x2000 size. However, you can make this function work for other sizes too. For instance, to use a 1000x1000 size, 
 * adjusting the text positions by dividing them by 2.
 * 
 * @param boundingBoxes Set of bounding boxes detected in a particular image.
 * @param worldSize Dimensions of the 3D world in which the object exists.
 * @param imageSize Size of the image in which the 3D objects are visualised.
 * @param bSaveLidarTopView Whether to save LiDAR top-view perspectives in the current working directory.
 * @param saveAs Name of the top-view perspective to save, if bSaveLidarTopView = true.
 * @param bWait Whether to wait for key to be pressed before displaying the next perspective.
 */
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, 
  bool bSaveLidarTopView, std::string saveAs, bool bWait=true);

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
 * @brief FP.5, FP.6: Compute the sample excess kurtosis of a vector's components using estimator g_2 in [1].
 *   Excess kurtosis is defined as kurtosis - 3, where 3 is the kurtosis of the symmetric Gaussian distribution.
 *   Distributions with negative (positive) excess kurtosis are thin-tailed or platykurtic (fat-tailed, leptokurtic).
 * 
 * @param vec The input vector.
 * 
 * Resources:
 * 
 * [1] - https://en.wikipedia.org/wiki/Kurtosis
 */
double excess_kurtosis(const std::vector<double>& vec);

/**
 * @brief FP.5, FP.6: Compute the sample skewness of a vector's components using estimator b_1 in [1].
 * 
 * Resources:
 * 
 * [1] - https://en.wikipedia.org/wiki/Skewness
 */
double skewness(const std::vector<double>& vec);

/**
 * @brief FP.5, FP.6: Compute the sample variance of a vector's components.
 * 
 * @param vec The input vector.
 */
double variance(const std::vector<double>& vec);

/**
 * @brief FP.2: Compute the simple average of a vector's components.
 * 
 * @param vec The input vector.
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
 * @param clusters The populated Euclidean clusters container.
 * @param removed The container of discarded points.
 * @param bShowRemoved Whether to also render discarded clusters (in white). 
 * 
 * Resources:
 * 
 * [1] - https://docs.opencv.org/4.2.0/d4/dba/classcv_1_1viz_1_1Color.html
 */
void renderClusters(const std::vector<std::set<int>>& clusters, const std::vector<std::set<int>>& removed, 
  bool bShowRemoved=true);

/**
 * @brief FP.2: Recursive function to compute the nearest neighbors of a query point based on input L2-norm [1] [2].
 * 
 * @param index Id of the query point.
 * @param src The input LiDAR point cloud data.
 * @param srcMat The input LiDAR point cloud data in matrix form.
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
void clusterHelper(int index, const std::vector<LidarPoint>& src, const cv::Mat& srcMat, std::vector<LidarPoint>& cluster, 
  std::vector<bool>& processed, cv::flann::GenericIndex<T>& tree, float radius, int knn, int minSize, int maxSize);

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
void euclideanClustering(const std::vector<LidarPoint>& src, std::vector<std::vector<LidarPoint>>& clusters, 
  std::vector<std::vector<LidarPoint>>& removed, float radius, int knn, int minSize, int maxSize);

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
 * @param bShowRemoved Whether to print statistics on filtering outcome and distribution of points.
 * 
 * Resources:
 * 
 * [1] - https://en.wikipedia.org/wiki/Outlier
 * [2] - https://knowledge.udacity.com/questions/296395
 * [3] - https://docs.opencv.org/4.2.0/db/d18/classcv_1_1flann_1_1GenericIndex.html
 */
void removeOutliers(std::vector<LidarPoint>& src, std::vector<LidarPoint>& dst, FilteringMethod method, float radius,
  int knn, int minSize, int maxSize, bool bRenderClusters, bool bShowRemoved, bool bPrintStats=true);

/**
 * @brief FP.2: Compute stable LiDAR time-to-collision (TTC) between consecutive frames assuming constant velocity [1].
 * 
 * @param lidarPointsPrev LiDAR point cloud data from the previous frame.
 * @param lidarPointsCurr LiDAR point cloud data from the current frame.
 * @param frameRate Time between two measurements, in seconds.
 * @param TTC LiDAR time-to-collision measure, to populate.
 * @param filteringMethod Outlier filtering method, either Tukey's fences [2] or Euclidean clustering [3].
 * @param bRenderClusters Whether to display 3D rendering of clustered LiDAR points.
 * @param bShowRemoved Whether to include colorless outlier points in the rendering view.

 * Euclidean clustering -only parameters:
 * 
 * @param radius Distance tolerance to query point for the neighborhood search; will be squared (L2-norm).
 * @param knn No. of points to include at each radius search.
 * @param minSize Minimum cluster size. Clusters smaller than this value will be discarded as outliers.
 * @param maxSize Maximum cluster size. Clusters larger than this value will be broken down.
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
 * brief FP.4: Compute stable time-to-collision (TTC) based on keypoint correspondences in successive images [1].
 * 
 * @param kptsPrev Container of keypoints detected in the previous frame.
 * @param kptsCurr Container of keypoints detected in the current frame.
 * @param kptMatches Container of matches between the previous and the current frame.
 * @param frameRate Time between two measurements, in seconds.
 * @param TTC Camera time-to-collision measure, to populate.
 * @param visImg Unused.
 * 
 * Resources:
 * 
 * [1] https://knowledge.udacity.com/questions/668076
 */
void computeTTCCamera(std::vector<cv::KeyPoint>& kptsPrev, std::vector<cv::KeyPoint>& kptsCurr,
  std::vector<cv::DMatch> kptMatches, double frameRate, double& TTC, cv::Mat* visImg=nullptr);

/**
 * @brief FP.2: Display output statistics from Tukey's fences.
 * 
 * @param src The input LiDAR point cloud data.
 * @param kept Container of points kept post filtering.
 * @param removed Container of points removed by filtering.
 * @param lowerBound The lower Tukeys' fence.
 * @param upperBound The upper Tukey's fence.
 */
void printFilteringStats(const std::vector<LidarPoint>& src, const std::vector<LidarPoint>& kept, 
  const std::vector<LidarPoint>& removed, double lowerBound, double upperBound);

/**
 * @brief FP.2: Display output statistics from Euclidean clustering.
 * 
 * @param src The input LiDAR point cloud data.
 * @param clusters Container of points kept after Euclidean clustering.
 * @param removed Container of points removed after Euclidean clustering.
 * @param radius Distance tolerance to query point used for the neighborhood search.
 * @param minSize Minimum acceptable size for a cluster.
 * @param maxSize Maximum acceptable size for a cluster.
 */
void printFilteringStats(const std::vector<LidarPoint>& src, const std::vector<std::vector<LidarPoint>>& clusters, 
  const std::vector<std::vector<LidarPoint>>& removed, float radius, int minSize, int maxSize);

/**
 * @brief FP.6: Print camera-based filtering statistics.
 * 
 * @param distRatios Candidate distance ratios.
 * @param filteredDistRatios Available distance ratios post filtering.
 * @param medianDistRatio Median distance ratio to compute TTC.
 */
void printFilteringStats(const std::vector<double> distRatios, const std::vector<double> filteredDistRatios, 
  double medianDistRatio);

/**
 * @brief FP.5, FP.6: Print moments, size, and shape statistics on the distribution of input points.
 * 
 * @param vec The input data.
 */
void printDistributionStats(const std::vector<double>& vec);

/**
 * @brief: FP.2: Print LiDAR time-to-collision statistics.
 * 
 * @param medianXPrev Median x-coordinate of the LiDAR points in the previous frame.
 * @param medianXCurr Median x-coordinate of the LiDAR points in the current frame.
 * @param TTC LiDAR time-to-collision based on the previous and current frames.
 */
void printTTCStats(const double medianXPrev, const double medianXCurr, const double TTC);

/**
 * @brief FP.6: Print camera time-to-collision statistics.
 * 
 * @param TTC Camera time-to-collision based on the previous and current frames.
 */
void printTTCStats(const double TTC);

/**
 * @brief FP.5, FP.6: Print summary statistics on time-to-collision for all image pairs in scope.
 * 
 * @param imgStartIndex First file index loaded.
 * @param ttcStats Collection of statistics on LiDAR and camera TTC for all image pairs.
 */
void printSummaryStats(const int imgStartIndex, const std::vector<std::vector<double>>& ttcStats);

#endif /* CAMFUSION_HPP */
