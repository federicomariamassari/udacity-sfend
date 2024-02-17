#ifndef OBJECTDETECTION2D_HPP
#define OBJECTDETECTION2D_HPP

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include "dataStructures.h"


/**
 * @brief Detects objects in an image using the YOLOv3 library and a set of pre-trained objects from the COCO database;
 *        A set of 80 classes is listed in 'coco.names' and pre-trained weights are stored in 'yolov3.weights'.
 * 
 * @param img The input camera image.
 * @param bBoxes Container of detected bounding boxes in the image, to populate. 
 * @param confThreshold The confidence level threshold.
 * @param nmsThreshold Non-maxima suppression threshold; choose by confidence if boxes overlap more than this value.
 * @param basePath Base location containing the required YOLOv3 files.
 * @param classesFile File containing a set of pre-trained objects from the COCO database.
 * @param modelConfiguration YOLOv3 configurations file.
 * @param modelWeights File containing a set of pre-trained YOLOv3 weights. 
 * @param bVis Whether to display the object detection output: bounding boxes, names, and confidence levels.
 * @param bExtraAccuracy If true, increase blob pixel size from (416 x 416) to (448 x 448); must be a multiple of 32.
 */
void detectObjects(cv::Mat& img, std::vector<BoundingBox>& bBoxes, float confThreshold, float nmsThreshold, 
  std::string basePath, std::string classesFile, std::string modelConfiguration, std::string modelWeights, 
  bool bVis, bool& bExtraAccuracy);

#endif /* OBJECTDETECTION2D_HPP */
