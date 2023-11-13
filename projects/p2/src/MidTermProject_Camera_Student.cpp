#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <numeric>
#include <limits>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

// Custom addition for tasks MP.7-9
#include "custom/stats.h"
#include "custom/stats.cpp"  // To help linker

using namespace std;


int main(int argc, const char *argv[])
{
  // Data location
  string dataPath = "../";
  string sep = string(80, '*');

  // Camera
  string imgBasePath = dataPath + "images/";
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";  // Left camera, color
  string imgFileType = ".png";
  int imgStartIndex = 0;  // First file index to load (assumes LiDAR and camera names have identical naming convention)
  int imgEndIndex = 9;  // Last file index to load
  int imgFillWidth = 4;  // No. of digits which make up the file index (e.g. img-0001.png)

  // Miscellanea
  int dataBufferSize = 2;  // No. of images which are held in memory (ring buffer) at the same time
  vector<DataFrame> dataBuffer;  // List of data frames which are held in memory at the same time
  bool bVis = false;  // True to visualize results

  /* STRING VARIABLES INITIALIZATION */

  string detectorType = "SIFT";  // HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, SIFT, SURF
  string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
  
  string matcherType = "MAT_BF";  // MAT_BF, MAT_FLANN
  string descriptorGroup = "DES_BINARY";  // DES_HOG, DES_BINARY
  string selectorType = "SEL_NN";  // SEL_NN, SEL_KNN

  /* END STRING VARIABLES INITIALIZATION */

  vector<double> neighborhoodSizes;

  cout << endl;
  cout << "Keypoint detector: " << detectorType << endl;
  cout << "Keypoint descriptor: " << descriptorType << endl;

  if (matcherType.compare("MAT_BF"))
  {
    // https://answers.opencv.org/question/10046/feature-2d-feature-matching-fails-with-assert-statcpp/
    // https://knowledge.udacity.com/questions/117307
    if ((descriptorType.compare("SIFT") == 0 || descriptorType.compare("SURF") == 0) 
      && descriptorGroup.compare("DES_BINARY") == 0)
    {
      descriptorGroup = "DES_HOG";
      cout << endl;
      cerr << "(!) WARNING: DES_BINARY is incompatible with SIFT/SURF. Switching to DES_HOG" << endl;
    }
  }

  cout << endl;
  cout << "Feature matcher: " << matcherType << endl;
  cout << "Descriptor group: " << descriptorGroup << endl;
  cout << "Selector type: " << selectorType << endl;

  /* MAIN LOOP OVER ALL IMAGES */
  for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
  {
    /* LOAD IMAGE INTO BUFFER*/

    cout << endl << sep << endl << "Loading image: " << imgIndex << endl << sep << endl;

    // Assemble filenames for current index
    ostringstream imgNumber;
    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

    // Load image from file and convert to grayscale
    cv::Mat img, imgGray;
    img = cv::imread(imgFullFilename);
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    //// STUDENT ASSIGNMENT
    //// TASK MP.1: Replace the following code with ring buffer of size dataBufferSize

    DataFrame frame;
    frame.cameraImg = imgGray;

    if (dataBuffer.size() == dataBufferSize)

      // https://knowledge.udacity.com/questions/644337
      dataBuffer.erase(dataBuffer.begin());
    
    dataBuffer.push_back(frame);

    //// EOF STUDENT ASSIGNMENT TASK MP.1

    cout << "#1: LOAD IMAGE INTO BUFFER done" << endl << endl;

    /* DETECT IMAGE KEYPOINTS */

    // Extract 2D keypoints from current image
    vector<cv::KeyPoint> keypoints; // Create empty feature list for current image

    //// STUDENT ASSIGNMENT
    //// TASK MP.2: Add below keypoint detectors in matching2D.cpp and enable string-based selection based on detectorType
    //// HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

    if (detectorType.compare("SHITOMASI") == 0)
      detKeypointsShiTomasi(keypoints, imgGray, bVis);

    else if (detectorType.compare("HARRIS") == 0)
      detKeypointsHarris(keypoints, imgGray, bVis);

    else
      detKeypointsModern(keypoints, imgGray, detectorType, bVis);

    //// EOF STUDENT ASSIGNMENT

    //// STUDENT ASSIGNMENT
    //// TASK MP.3: Only keep keypoints on the preceding vehicle

    // Only keep keypoints on the preceding vehicle
    bool bFocusOnVehicle = true;
    cv::Rect vehicleRect(535, 180, 180, 150);  // Will include side mirror of left vehicle and partial ego shadow

    if (bFocusOnVehicle)
    {
      vector<cv::KeyPoint> rectKeyPoints;

      for (const auto& keypoint : keypoints)
      {
        if (vehicleRect.contains(keypoint.pt))
        {
          rectKeyPoints.push_back(keypoint);

          // Cumulate neighborhood sizes for average statistics
          neighborhoodSizes.push_back(keypoint.size);
        }
      }

      keypoints = rectKeyPoints;
      cout << "Number of keypoints in the region of interest = " << keypoints.size() << endl;
    }

    //// EOF STUDENT ASSIGNMENT
    
    // Optional : limit number of keypoints (helpful for debugging and learning)
    bool bLimitKpts = false;
    if (bLimitKpts)
    {
      int maxKeypoints = 50;

      if (detectorType.compare("SHITOMASI") == 0)

        // There is no response info, so keep the first 50 as they are sorted in descending quality order
        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());

      cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
      cout << "(!) NOTE: Keypoints have been limited!" << endl;
    }

    // Push keypoints and descriptor for current frame to end of data buffer
    (dataBuffer.end() - 1)->keypoints = keypoints;
    cout << "#2: DETECT KEYPOINTS done" << endl << endl;

    /* CALCULATE STATISTICS ON KEYPOINT NEIGHBORHOOD DISTRIBUTION */

    vector<double> sizes = extractNeighborhoodSizes(keypoints);
    Stats stats = calculateStatistics(sizes);
    printStatistics(stats);

    /* EXTRACT KEYPOINT DESCRIPTORS */

    //// STUDENT ASSIGNMENT
    //// TASK MP.4: Add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
    //// BRIEF, ORB, FREAK, AKAZE, SIFT

    cv::Mat descriptors;

    if ((detectorType == "SIFT" && descriptorType == "ORB") || (detectorType == "ORB" && descriptorType == "SIFT"))
    {
      // https://knowledge.udacity.com/questions/105392
      string errorMsg = detectorType + " detector does not work with " + descriptorType + " descriptor.";
      throw invalid_argument(errorMsg);
    }

    if (descriptorType == "AKAZE" && detectorType != "AKAZE")
      throw invalid_argument("AKAZE descriptor only works with AKAZE detector.");

    descKeypoints((dataBuffer.end()-1)->keypoints, (dataBuffer.end()-1)->cameraImg, descriptors, descriptorType);

    //// EOF STUDENT ASSIGNMENT

    // Push descriptors for current frame to end of data buffer
    (dataBuffer.end()-1)->descriptors = descriptors;

    cout << "#3: EXTRACT DESCRIPTORS done" << endl << endl;

    if (dataBuffer.size() > 1)  // Wait until at least two images have been processed
    {
      /* MATCH KEYPOINT DESCRIPTORS */

      vector<cv::DMatch> matches;

      //// STUDENT ASSIGNMENT
      //// TASK MP.5: Add FLANN matching in file matching2D.cpp
      //// TASK MP.6: Add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

      if (dataBuffer.size() == dataBufferSize)
        cout << "Comparing images: " << imgIndex-1 << " and " << imgIndex << endl;

      matchDescriptors((dataBuffer.end()-2)->keypoints, (dataBuffer.end()-1)->keypoints, (dataBuffer.end()-2)->descriptors, 
        (dataBuffer.end()-1)->descriptors, matches, descriptorGroup, matcherType, selectorType);

      //// EOF STUDENT ASSIGNMENT

      // Store matches in current data frame
      (dataBuffer.end() - 1)->kptMatches = matches;

      cout << "#4: MATCH KEYPOINT DESCRIPTORS done" << endl << endl;

      // Visualize matches between current and previous image
      bVis = true;

      if (bVis)
      {
        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints, 
          (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints, matches, matchImg, cv::Scalar::all(-1), 
          cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        string windowName = "Matching keypoints between two camera images";
        cv::namedWindow(windowName, 7);
        imshow(windowName, matchImg);
        cout << "Press any key to continue to next image" << endl;
        cv::waitKey(0);  // Wait for key to be pressed
      }
      bVis = false;
    }

    else
      cout << "#4: MATCH KEYPOINT DESCRIPTORS skipped (no image to compare to)" << endl;

  }  // eof loop over all images

  /* AVERAGE STATISTICS ON THE DISTRIBUTION OF KEYPOINT NEIGHBORHOOD SIZES */

  cout << endl;
  string msg = "MP.7: Average statistics, keypoint neighborhood sizes distribution";
  cout << endl << sep << endl << msg << endl << sep << endl;

  Stats avgStatistics = calculateStatistics(neighborhoodSizes);
  printStatistics(avgStatistics);

  return 0;
}