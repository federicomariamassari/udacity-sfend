#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <limits>
#include <map>  // MP.8-9: To pretty-print matched keypoints' statistics and total detection-description time
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>  // MP.7: To pretty-print keypoints' neighborhood sizes statistics as key-value pairs
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
 * @param detTickCounts The structure to hold the detection tick count for the analysed image.
 * @param bVis Whether to visualize the output image with superimposed detection marks.
 * @param bPrintMsg Whether to print logs.
 * 
 * Resources:
 * 
 * [1] - Shi, Tomasi: "Good Features to Track" (1994)
 */
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, std::vector<double>& detTickCounts, 
  const bool bVis=false, const bool bPrintMsg=true);

/** 
 * @brief Perform Harris' corner detection (1988) with non-maxima suppression (NMS) [1].
 * Adapted from solution to "Harris Corner Detection", Tracking Image Features: Lesson 5, Udacity [2].
 * 
 * @param keypoints The structure that will hold the Harris-detected keypoints.
 * @param img The input grayscale image.
 * @param detTickCounts The structure to hold the detection tick count for the analysed image.
 * @param bVis Whether to visualize the output image with superimposed detection marks.
 * @param bPrintMsg Whether to print logs.
 * 
 * Resources:
 *
 * [1] - Harris, Stephens: "A Combined Corner and Edge Detector" (1988)
 * [2] - Harris Corner Detection (Tracking Image Features: Lesson 5), Udacity Sensor Fusion Nanodegree
 * [3] - Addendum - NMS Algorithm (Tracking Image Features: Lesson 6), Udacity Sensor Fusion Nanodegree
 * [4] - Harris Corner Detector Tutorial (https://docs.opencv.org/4.2.0/d4/d7d/tutorial_harris_detector.html)
 */
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, std::vector<double>& detTickCounts, 
  const bool bVis=false, const bool bPrintMsg=true);

/**
 * @brief Detect image keypoints using modern algorithms: SIFT, SURF, FAST, ORB, BRISK, AKAZE [1].
 *
 * @param keypoints The structure that will hold the detected keypoints.
 * @param img The input grayscale image.
 * @param detectorType The name of the used detector.
 * @param detTickCounts The structure to hold the detection tick count for the analysed image.
 * @param bVis Whether to visualize the output image with superimposed detection marks.
 * @param bPrintMsg Whether to print logs.
 * 
 * Resources:
 * 
 * [1] - https://docs.opencv.org/4.2.0/d5/d51/group__features2d__main.html
 */
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, const std::string detectorType, 
  std::vector<double>& detTickCounts, const bool bVis=false, const bool bPrintMsg=true);

/**
 * @brief Use one among several state-of-art descriptors to uniquely identify keypoints.
 * 
 * @param keypoints The structure that holds the detected keypoints.
 * @param img The input grayscale image.
 * @param descriptors The structure that will hold the descriptors' info.
 * @param detectorType The detector's name. To output meaningful warnings for bad detector-descriptor combinations.
 * @param descriptorType The descriptor's name.
 * @param descTickCounts The structure to hold the description tick count for the analysed image.
 * @param bPrintMsg Whether to print logs.
 */ 
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, cv::Mat &descriptors, 
  const std::string detectorType, const std::string descriptorType, std::vector<double>& descTickCounts, 
  const bool bPrintMsg=true);

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
 * @param infoCounter To avoid cluttering the console when an info message has already been printed for an image.
 * @param bPrintMsg Whether to print logs.
 * 
 * Resources:
 *
 * [1] - https://docs.opencv.org/4.2.0/dg8/d9b/group__features2d__match.html
 * [2] - Exercise - Descriptor Matching (Tracking Image Features: Lesson 12), Udacity Sensor Fusion Nanodegree
 * [3] - https://knowledge.udacity.com/questions/118373
 */
void matchDescriptors(const std::vector<cv::KeyPoint> &kPtsSource, const std::vector<cv::KeyPoint> &kPtsRef, 
  const cv::Mat &descSource, const cv::Mat &descRef, std::vector<cv::DMatch> &matches, const std::string descriptorType, 
  std::string descriptorGroup, const std::string matcherType, const std::string selectorType, int &infoCounter, 
  const bool bPrintMsg=true);

/*********************************************************************************************************************
 * CUSTOM ADDITIONS
 *********************************************************************************************************************/

/** 
 * @brief MP.7: Holds basic statistics of keypoints' neighborhood distributions, for all detectors.
 */
struct Stats {

  int img_count;

  // Vectors are chosen to ease printing on Terminal
  std::vector<int> vec_size;
  std::vector<double> kpt_img, avg_det_time;
  std::vector<double> mean, median, mode, std_dev, min, max, range, p25, p75, iqr;
};

/** 
 * @brief MP.1: Load image into a ring buffer of custom size. Discard oldest element if full (FIFO).
 * 
 * @param filename Name, including full path, of the image to load.
 * @param dataBuffer The data structure to serve as ring buffer.
 * @param dataBufferSize The size of the buffer.
 * 
 * @return frame Represents the available sensor information at the same time instance.
 */
DataFrame loadImageIntoBuffer(const std::string filename, std::vector<DataFrame>& dataBuffer, 
  const int dataBufferSize);

/**
 * @brief MP.2: Detect image keypoints and time the extraction process. A layer on top of the detection methods.
 * 
 * @param detectorType The name of the detector.
 * @param img The input grayscale image.
 * @param detTickCounts The structure to hold detection tick count for the analysed image.
 * @param bVis Whether to visualize the output image with superimposed detection marks.
 * @param bPrintMsg Whether to print logs.
 * 
 * @return keypoints The data structure holding the found keypoints.
 */
std::vector<cv::KeyPoint> detectKeypoints(const std::string detectorType, cv::Mat& img, 
  std::vector<double>& detTickCounts, const bool bVis=false, const bool bPrintMsg=true);

/**
 * @brief MP.3: Remove keypoints outside of a rectangle area centered on the vehicle ahead.
 * 
 * @param keypoints The data structure holding the keypoints.
 * @param bFocusOnVehicle Whether to restrict the focus to the preceding vehicle.
 * @param bPrintMsg Whether to print logs.
 */
void focusOnArea(std::vector<cv::KeyPoint>& keypoints, const bool bFocusOnVehicle=true, const bool bPrintMsg=true);

/**
 * @brief Optional: Limit number of keypoints (helpful for debugging and learning).
 * 
 * @param keypoints The data structure holding the keypoints.
 * @param detector The name of the detector.
 * @param maxKeypoints The maximum allowed number of keypoints.
 */
void limitKeypoints(std::vector<cv::KeyPoint>& keypoints, const std::string detector, const int maxKeypoints=50);

/**
 * @brief MP.7: Extract keypoints' neighborhood sizes for a single detector.
 * 
 * @param keypoints The data structure holding the keypoints.
 * @param neighborhoodSizes The structure to hold the neighborhood sizes of all detected keypoints.
 */
void extractNeighborhoodSizes(const std::vector<cv::KeyPoint>& keypoints, std::vector<double>& neighborhoodSizes);

/**
 * @brief MP.7: Calculate percentile p on a detector's sorted distribution of keypoints' neighborhood sizes.
 * 
 * @param vec The sorted vector of keypoints' neighborhood sizes.
 * @param p The desired decimal percentile.
 * 
 * @return res The vector element corresponding to the desired percentile.
 */
double percentile(const std::vector<cv::KeyPoint>& vec, const float p);

/**
 * @brief MP.7: Simplified mode calculation assuming input vector is sorted.
 * 
 * @param vec The sorted vector on which to compute the mode.
 * 
 * @return mode The mode of the input vector.
 */
double computeMode(const std::vector<double>& vec);

/**
 * @brief MP.7: Calculate keypoints' neighborhood distribution statistics for a single detector.
 * 
 * @param s The structure to hold the input vectors' statistics. 
 * @param vec The structure holding the keypoints' neighborhood sizes.
 */
void computeStatistics(Stats& s, std::vector<double>& vec);

/**
 * @brief Print a key, value pair of an input map.
 * 
 * @param p The input pair, with string key and mutable-type value.
 */
template<typename T>
void printLine(const std::pair<const std::string, typename std::vector<T>>& p);

/**
 * @brief Print the elements of a vector with custom spacing.
 * 
 * @param v The input vector.
 * @param imgEndIndex The cutoff for custom spacing.
 */
template<typename T>
void printLine(const std::vector<T> v, const int& imgEndIndex);

/**
 * @brief MP.7: Pretty-print detectors' statistics.
 * 
 * @param detector The structure holding the detector name(s).
 * @param s The structure holding the statistics to print.
 */
void printStatistics(const std::vector<std::string>& detectors, const Stats& s);

/**
 * @brief MP.8-9: Pretty-print matched keypoints' statistics and total detection-description time.
 * 
 * @param m The structure holding the key-value pairs to print.
 * @param imgEndIndex The cutoff for custom spacing.
 * @param sep The map key delimiter separating detector and descriptor names.
 */
void printStatistics(const std::map<std::string, std::vector<double>>& m, const int& imgEndIndex, 
  const std::string sep="-");

#endif /* matching2D_hpp */