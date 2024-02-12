#ifndef objectDetection2D_hpp
#define objectDetection2D_hpp

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include "dataStructures.h"

void detectObjects(cv::Mat& img, std::vector<BoundingBox>& bBoxes, float confThreshold, float nmsThreshold, 
  std::string basePath, std::string classesFile, std::string modelConfiguration, std::string modelWeights, 
  bool bVis, bool& bExtraAccuracy);

#endif /* objectDetection2D_hpp */
