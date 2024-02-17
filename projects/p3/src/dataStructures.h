#ifndef DATASTRUCTURES_H
#define DATASTRUCTURES_H

#include <map>
#include <vector>

#include <opencv2/core.hpp>

/**
 * Single LiDAR point in space.
 */
struct LidarPoint 
{ 
  double x, y, z, r;  // x,y,z in [m], r is point reflectivity
};

/**
 * Bounding box around a classified object (contains both 2D and 3D data).
 */
struct BoundingBox 
{
  int boxID;  // Unique identifier for this bounding box
  int trackID;  // Unique identifier for the track to which this bounding box belongs
  
  cv::Rect roi;  // 2D region-of-interest in image coordinates
  int classID;  // ID based on class file provided to YOLOv3 framework
  double confidence;  // Classification trust

  std::vector<LidarPoint> lidarPoints;  // LiDAR 3D points which project into 2D image roi
  std::vector<cv::KeyPoint> keypoints;  // Keypoints enclosed by 2D roi
  std::vector<cv::DMatch> kptMatches;  // Keypoint matches enclosed by 2D roi
};

/**
 * Represents the available sensor information at the same time instance.
 */
struct DataFrame 
{ 
  cv::Mat cameraImg;  // Camera image
  
  std::vector<cv::KeyPoint> keypoints;  // 2D keypoints within camera image
  cv::Mat descriptors;  // Keypoint descriptors
  std::vector<cv::DMatch> kptMatches;  // Keypoint matches between previous and current frame
  std::vector<LidarPoint> lidarPoints;

  std::vector<BoundingBox> boundingBoxes;  // ROI around detected objects in 2D image coordinates
  std::map<int, int> bbMatches;  // Bounding box matches between previous and current frame
};

#endif /* DATASTRUCTURES_H */
