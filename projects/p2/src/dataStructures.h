#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame {  // Represents the available sensor information at the same time instance

  cv::Mat cameraImg;  // Camera image

  std::vector<cv::KeyPoint> keypoints;  // 2D keypoints within camera image
  cv::Mat descriptors;  // Keypoint descriptors
  std::vector<cv::DMatch> kptMatches;  // Keypoint matches between previous and current frame
};

#endif /* dataStructures_h */
