#ifndef LIDARDATA_HPP
#define LIDARDATA_HPP

#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dataStructures.h"


/**
 * @brief Remove LiDAR points based on min-max distance in X, Y, Z, and min reflectivity
 * 
 * @param lidarPoints Detected 3D LiDAR points for a particular image.
 * @param minX Minimum tolerated distance from ego vehicle (x-coordinate).
 * @param maxX Maximum tolerated distance from ego vehicle (x-coordinate).
 * @param maxY Maximum tolerated distance on the RHS of the ego lane (y-coordinate).
 * @param minZ Minimum tolerated distance below the LiDAR sensor on top of ego vehicle (z-coordinate).
 * @param maxZ Maximum tolerated distance above the LiDAR sensor on top of ego vehicle (z-coordinate).
 * @param minR Minimum reflectivity level, 0 <= r <= 1.
 */
void cropLidarPoints(std::vector<LidarPoint>& lidarPoints, float minX, float maxX, float maxY, float minZ, float maxZ, 
	float minR);

/**
 * @brief Load LiDAR points from a given location and store them in a vector
 * 
 * @param lidarPoints Detected 3D LiDAR points for a particular image.
 * @param filename Full path of the file containing LiDAR point data.
 */
void loadLidarFromFile(std::vector<LidarPoint>& lidarPoints, std::string filename);

/**
 * @brief Display LiDAR top-view perspective augmented with cluster statistics.
 * 
 * @param lidarPoints Detected 3D LiDAR points for a particular image.
 * @param worldSize Dimensions of the 3D world in which the object exists.
 * @param imageSize Size of the image in which the 3D objects are visualised.
 * @param bWait Whether to wait for key to be pressed before displaying the next perspective.
 */
void showLidarTopview(std::vector<LidarPoint>& lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait=true);

/**
 * #brief Display LiDAR points on top of a 2-dimensional frame.
 * 
 * @param img The input image.
 * @param lidarPoints Detected 3D LiDAR points for a particular image.
 * @param P_rect_xx Intrinsic parameter matrix K (3x4 projection matrix after rectification).
 * @param R_rect_xx 3x3 rectifying rotation matrix to make image planes co-planar.
 * @param RT Extrinsic parameters (rotation matrix and translation vector).
 * @param extVisImg The image on which to display the LiDAR points overlay.
 */
void showLidarImgOverlay(cv::Mat& img, std::vector<LidarPoint>& lidarPoints, cv::Mat& P_rect_xx, cv::Mat& R_rect_xx, 
	cv::Mat& RT, cv::Mat* extVisImg=nullptr);

#endif /* LIDARDATA_HPP */
