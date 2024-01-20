#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <set>
#include <vector>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dataStructures.h"


void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT);
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches);

/**
 * @brief FP.1: Match consecutive bounding box pairs by the frequency of matches they contain [1].
 * 
 * @param matches The keypoint matches between previous (query) and current (train) frames.
 * @param bbBestMatches Container of previous-current bounding box IDs that result in the best matches, to populate.
 * @param prevFrame Sensor information at previous time.
 * @param currFrame Sensor information at current time.
 * 
 * Resources:
 * 
 * [1] - https://knowledge.udacity.com/questions/570553 (additional refactoring suggested by Udacity GPT)
 * [2] - https://docs.opencv.org/4.2.0/d4/de0/classcv_1_1DMatch.html
 */
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, 
    DataFrame &currFrame);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait=true);

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg=nullptr);
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC);                  

#endif /* camFusion_hpp */
