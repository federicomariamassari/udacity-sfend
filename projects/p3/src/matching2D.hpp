#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"


/**
 * @brief Detect keypoints in an image using the traditional Shi-Thomasi (1994) detector [1]. Part of the starter code.
 *
 * @param keypoints The structure that will hold the Shi-Tomasi -detected keypoints.
 * @param img The input grayscale image.
 * @param bVis Whether to visualize the output image with superimposed detection marks.
 * 
 * Resources:
 * 
 * [1] - Shi, Tomasi: "Good Features to Track" (1994)
 */
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);

/** 
 * @brief Perform Harris' corner detection (1988) with non-maxima suppression (NMS) [1].
 * Adapted from solution to "Harris Corner Detection", Tracking Image Features: Lesson 5, Udacity [2].
 * 
 * @param keypoints The structure that will hold the Harris-detected keypoints.
 * @param img The input grayscale image.
 * @param bVis Whether to visualize the output image with superimposed detection marks.
 * 
 * Resources:
 *
 * [1] - Harris, Stephens: "A Combined Corner and Edge Detector" (1988)
 * [2] - Harris Corner Detection (Tracking Image Features: Lesson 5), Udacity Sensor Fusion Nanodegree
 * [3] - Addendum - NMS Algorithm (Tracking Image Features: Lesson 6), Udacity Sensor Fusion Nanodegree
 * [4] - Harris Corner Detector Tutorial (https://docs.opencv.org/4.2.0/d4/d7d/tutorial_harris_detector.html)
 */
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);

/**
 * @brief Detect image keypoints using modern algorithms: SIFT, SURF, FAST, ORB, BRISK, AKAZE [1].
 *
 * @param keypoints The structure that will hold the detected keypoints.
 * @param img The input grayscale image.
 * @param detectorType The name of the used detector.
 * @param bVis Whether to visualize the output image with superimposed detection marks.
 * 
 * Resources:
 * 
 * [1] - https://docs.opencv.org/4.2.0/d5/d51/group__features2d__main.html
 */
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis=false);

/**
 * @brief Use one among several state-of-art descriptors to uniquely identify keypoints.
 * 
 * @param keypoints The structure that holds the detected keypoints.
 * @param img The input grayscale image.
 * @param descriptors The structure that will hold the descriptors' info.
 * @param detectorType The detector's name. To output meaningful warnings for bad detector-descriptor combinations.
 * @param descriptorType The descriptor's name.
 */ 
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string detectorType, 
  std::string descriptorType);

/**
 * @brief Find (best) matches for keypoints in two camera images based on several matching methods.
 *
 * @param kPtsSource The structure holding the keypoints detected in the source image.
 * @param kPtsRef The structure holding the keypoints detected in the reference (i.e., destination) image.
 * @param descSource The structure holding the descriptors associated to the source image keypoints.
 * @param descRef The structure holding the descriptors associated to the reference image keypoints.
 * @param matches The structure that will hold the keypoint matches.
 * @param descriptorType The target descriptor's name.
 * @param descriptorGroup The category to which the selected descriptor belongs.
 * @param matcherType The matching algorithm's name.
 * @param selectorType The best matches' selector's name.
 * @param minDescDistanceRatio Minimum descriptor distance ratio test (based on solution to [2]).
 * 
 * Resources:
 *
 * [1] - https://docs.opencv.org/4.2.0/dg8/d9b/group__features2d__match.html
 * [2] - Exercise - Descriptor Matching (Tracking Image Features: Lesson 12), Udacity Sensor Fusion Nanodegree
 * [3] - https://knowledge.udacity.com/questions/118373
 * [4] - Tareen, Saleem: "A Comparative Analysis of SIFT, SURF, KAZE, AKAZE, ORB and BRISK" (2018)
 */
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, 
  cv::Mat &descRef, std::vector<cv::DMatch> &matches, std::string descriptorType, std::string descriptorGroup, 
  std::string matcherType, std::string selectorType, float minDescDistanceRatio);

#endif /* matching2D_hpp */