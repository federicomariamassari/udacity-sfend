#include <cmath>
#include <fstream>
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
#include "camFusion.hpp"
#include "lidarData.hpp"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"


using namespace std;


/** 
 * Customise main output
 */
struct Options
{
  /*******************************************************************************************************************
   * 2D FEATURE TRACKING OPTIONS
   *******************************************************************************************************************/
  string detectorType = "FAST";  // HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, SIFT, SURF
  string descriptorType = "BRIEF";  // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT, SURF
  string descriptorGroup = "DES_BINARY";  // DES_BINARY, DES_HOG

  string matcherType = "MAT_BF";  // MAT_BF, MAT_FLANN
  string selectorType = "SEL_KNN";  // SEL_NN, SEL_KNN

  /*******************************************************************************************************************
   * INPUT DATA OPTIONS
   *******************************************************************************************************************/
  int imgStartIndex = 0;  // First file index to load (assumes LiDAR and camera names have same naming convention)
  int imgEndIndex = 18;  // Last file index to load (default: 18; maximum: 77)
  int imgStepWidth = 1;  // 10/n Hz, n = 1, ... (if 2, skip every other image; 10 frames == 1 second)

  bool bExtraAccuracy = false;  // true for more-accurate YOLOv3 blob size (448 x 448), false for default (416 x 416).
                                // If imgEndIndex >= 48, set to true to avoid spurious bounding boxes!

  /*******************************************************************************************************************
   * VISUALISATION AND OUTPUT OPTIONS
   *******************************************************************************************************************/
  
  bool bVisYoloBoundingBoxes = false;  // true to show YOLOv3 bounding boxes, COCO names and confidence for each frame
  bool bVisLidarTopView = false;  // true to display LiDAR top-view perspective, false to skip
  bool bStopAtLidarTopView = false;  // wrapper around the continue statement; true to cycle through LiDAR top-views
                                     // (if bVisLidarTopView = true) false to proceed with TTC calculation

  bool bVisFinalOutput = true;  // true to view final output image with superimposed time-to-collision estimates
  bool bVisLidarOverlay = true;  // true to additionally superimpose LiDAR points on preceding vehicle bounding box
  bool bVisKeypointsOverlay = true;  // true to additionally superimpose keypoints on preceding vehicle bounding box

  bool bSaveYoloBBFrames = false;  // true to write YOLO frames in working directory (if bVisYoloBoundingBoxes = true)
  bool bSaveLidarTopView = false;  // true to write LiDAR top-views in working directory (if bVisLidarTopView = true)
  bool bSaveOutputFrames = false;  // true to write output frames in working directory (if bVisFinalOutput = true)

  /*******************************************************************************************************************
   * OUTLIER DETECTION AND DIAGNOSTICS OPTIONS
   *******************************************************************************************************************/

  // LiDAR 3D points outlier removal method
  FilteringMethod filteringMethod = FilteringMethod::TUKEY;  // TUKEY, EUCLIDEAN_CLUSTERING

  bool bLimitKpts = false;  // true to limit the number of keypoints; Helpful for debugging and learning, but it
                            // will lead to instability in time-to-collision calculations (NaN values)

  bool bRenderClusters = false;  // true to visualize 3D LiDAR point clusters after Euclidean clustering
  bool bShowRemoved = true;  // true to also display colorless outliers (if bRenderClusters = true)

  // EUCLIDEAN CLUSTERING -ONLY OPTIONS

  int knn = 5;  // No. of points to include at each radius search; set k > 3 for at least one new at each iteration,
                // but k too large will greatly increase computational time

  float radius = 0.12;  // Distance tolerance to query point for the neighborhood search; will be squared (L2-norm).
  int minSize = 15;  // Minimum cluster size. Clusters smaller than this value will be discarded as outliers.
  int maxSize = 600;  // Maximum cluster size. Clusters larger than this value will also be discarded.
};


int main(int argc, const char *argv[])
{
  /* INIT VARIABLES AND DATA STRUCTURES */

  Options opts = Options();

  // Data location
  string dataPath = "../";
  string sep = string(80, '*');

  // Additional camera parameters
  string imgBasePath = dataPath + "images/";
  string imgPrefix = "KITTI/2011_09_26/image_02/data/000000";  // Left camera, color
  string imgFileType = ".png";
  int imgFillWidth = 4;  // No. of digits which make up the file index (e.g. img-0001.png)

  // Object detection (YOLOv3)
  string yoloBasePath = dataPath + "dat/yolo/";
  string yoloClassesFile = yoloBasePath + "coco.names";
  string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
  string yoloModelWeights = yoloBasePath + "yolov3.weights";

  // LiDAR
  string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
  string lidarFileType = ".bin";

  // Calibration data for camera and LiDAR
  cv::Mat P_rect_00(3, 4, cv::DataType<double>::type);  // 3x4 projection matrix after rectification
  cv::Mat R_rect_00(4, 4, cv::DataType<double>::type);  // 3x3 rectifying rotation to make image planes co-planar
  cv::Mat RT(4, 4, cv::DataType<double>::type);  // Rotation matrix and translation vector
  
  RT.at<double>(0, 0) = 7.533745e-03; RT.at<double>(0, 1) = -9.999714e-01; RT.at<double>(0, 2) = -6.166020e-04; RT.at<double>(0, 3) = -4.069766e-03;
  RT.at<double>(1, 0) = 1.480249e-02; RT.at<double>(1, 1) = 7.280733e-04; RT.at<double>(1, 2) = -9.998902e-01; RT.at<double>(1, 3) = -7.631618e-02;
  RT.at<double>(2, 0) = 9.998621e-01; RT.at<double>(2, 1) = 7.523790e-03; RT.at<double>(2, 2) = 1.480755e-02; RT.at<double>(2, 3) = -2.717806e-01;
  RT.at<double>(3, 0) = 0.0; RT.at<double>(3, 1) = 0.0; RT.at<double>(3, 2) = 0.0; RT.at<double>(3, 3) = 1.0;

  R_rect_00.at<double>(0, 0) = 9.999239e-01; R_rect_00.at<double>(0, 1) = 9.837760e-03; R_rect_00.at<double>(0, 2) = -7.445048e-03; R_rect_00.at<double>(0, 3) = 0.0;
  R_rect_00.at<double>(1, 0) = -9.869795e-03; R_rect_00.at<double>(1, 1) = 9.999421e-01; R_rect_00.at<double>(1, 2) = -4.278459e-03; R_rect_00.at<double>(1, 3) = 0.0;
  R_rect_00.at<double>(2, 0) = 7.402527e-03; R_rect_00.at<double>(2, 1) = 4.351614e-03; R_rect_00.at<double>(2, 2) = 9.999631e-01; R_rect_00.at<double>(2, 3) = 0.0;
  R_rect_00.at<double>(3, 0) = 0; R_rect_00.at<double>(3, 1) = 0; R_rect_00.at<double>(3, 2) = 0; R_rect_00.at<double>(3,3) = 1;

  P_rect_00.at<double>(0, 0) = 7.215377e+02; P_rect_00.at<double>(0, 1) = 0.000000e+00; P_rect_00.at<double>(0, 2) = 6.095593e+02; P_rect_00.at<double>(0, 3) = 0.000000e+00;
  P_rect_00.at<double>(1, 0) = 0.000000e+00; P_rect_00.at<double>(1, 1) = 7.215377e+02; P_rect_00.at<double>(1, 2) = 1.728540e+02; P_rect_00.at<double>(1, 3) = 0.000000e+00;
  P_rect_00.at<double>(2, 0) = 0.000000e+00; P_rect_00.at<double>(2, 1) = 0.000000e+00; P_rect_00.at<double>(2, 2) = 1.000000e+00; P_rect_00.at<double>(2, 3) = 0.000000e+00;  

  // Miscellanea
  double sensorFrameRate = 10.0 / opts.imgStepWidth;  // Frames per second for LiDAR and camera (Hertz); 10Hz is the
                                                      // spin frequency of KITTI Velodyne HDL-64E sensor (~100k pps)

  int dataBufferSize = 2;  // No. of images which are held in memory (ring buffer) at the same time
  vector<DataFrame> dataBuffer;  // List of data frames which are held in memory at the same time

  // Time-to-collision statistics
  vector<vector<double>> ttcStats;  // FP.5, FP.6: Container of TTC statistics for later tabulation

  cout << endl;
  cout << "Keypoint detector: " << opts.detectorType << endl;
  cout << "Keypoint descriptor: " << opts.descriptorType << endl;
  cout << "Descriptor group: " << opts.descriptorGroup << endl;
  cout << "Feature matcher: " << opts.matcherType << endl;
  cout << "Selector type: " << opts.selectorType << endl;
  cout << endl;


  /* MAIN LOOP OVER ALL IMAGES */

  for (size_t imgIndex = 0; imgIndex <= (opts.imgEndIndex - opts.imgStartIndex); imgIndex += opts.imgStepWidth)
  {
    /* LOAD IMAGE INTO BUFFER */

    // Assemble filenames for current index
    ostringstream imgNumber;
    imgNumber << setfill('0') << setw(imgFillWidth) << opts.imgStartIndex + imgIndex;
    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

    cout << sep << endl << "Loading image: " << (opts.imgStartIndex + imgIndex) << endl << sep << endl;

    // Load image from file
    cv::Mat img = cv::imread(imgFullFilename);

    // Push image into data frame buffer
    DataFrame frame;
    frame.cameraImg = img;
    dataBuffer.push_back(frame);

    cout << "#1: LOAD IMAGE INTO BUFFER done" << endl << endl;

    /* DETECT & CLASSIFY OBJECTS */

    float confThreshold = 0.2;
    float nmsThreshold = 0.4;

    string yoloSaveAs = "yolo_" + imgNumber.str() + imgFileType;

    detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, 
      nmsThreshold, yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, opts.bVisYoloBoundingBoxes, 
      opts.bExtraAccuracy, opts.bSaveYoloBBFrames, yoloSaveAs);

    cout << "#2: DETECT & CLASSIFY OBJECTS done" << endl << endl;

    /* CROP LIDAR POINTS */

    // Load 3D LiDAR points from file
    string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
    vector<LidarPoint> lidarPoints;
    loadLidarFromFile(lidarPoints, lidarFullFilename);

    // Remove LiDAR points based on distance properties
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1;  // Focus on ego lane
    cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
  
    (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

    cout << "#3: CROP LIDAR POINTS done" << endl << endl;

    /* CLUSTER LIDAR POINT CLOUD */

    // Associate LiDAR points with camera-based ROI

    float shrinkFactor = 0.10;  // Shrinks each bounding box by the given percentage to avoid 3D-object merging 
                                // at the edges of an ROI

    clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, 
      P_rect_00, R_rect_00, RT);

    // Visualize 3D objects
    if (opts.bVisLidarTopView)
    {
      string lidarSaveAs = "lidar_" + imgNumber.str() + imgFileType;
      
      show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), 
        opts.bSaveLidarTopView, lidarSaveAs, true);
    }

    cout << "#4: CLUSTER LIDAR POINT CLOUD done" << endl << endl;
    
    if (opts.bStopAtLidarTopView)
    {
      cout << "(!) WARNING: Skipping time-to-collision calculation." << endl;
      continue;
    }

    /* DETECT IMAGE KEYPOINTS */

    // Convert current image to grayscale
    cv::Mat imgGray;
    cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

    // Extract 2D keypoints from current image
    vector<cv::KeyPoint> keypoints;  // Create empty feature list for current image
    
    if (opts.detectorType.compare("SHITOMASI") == 0)
      detKeypointsShiTomasi(keypoints, imgGray, false);  // Suppress visualization by default (out of scope)

    else if (opts.detectorType.compare("HARRIS") == 0)
      detKeypointsHarris(keypoints, imgGray, false);

    else
      detKeypointsModern(keypoints, imgGray, opts.detectorType, false);

    // Optional: limit number of keypoints; helpful to debug and learn, but will introduce NaN values in camera-based
    // time to collision estimates hence should be avoided (https://knowledge.udacity.com/questions/366760)
    if (opts.bLimitKpts)
    {
      int maxKeypoints = 50;

      if (opts.detectorType.compare("SHITOMASI") == 0)

        // There is no response info, so keep the first 50 as they are sorted in descending quality order
        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());

      cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);

      cerr << "(!) WARNING: Limit keypoints is ON. Keypoints will be limited!" << endl;
    }

    // Push keypoints and descriptor for current frame to end of data buffer
    (dataBuffer.end() - 1)->keypoints = keypoints;

    cout << "#5: DETECT KEYPOINTS done" << endl << endl;

    /* EXTRACT KEYPOINT DESCRIPTORS */

    cv::Mat descriptors;
    descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, opts.detectorType, 
      opts.descriptorType);

    // Push descriptors for current frame to end of data buffer
    (dataBuffer.end() - 1)->descriptors = descriptors;

    cout << "#6: EXTRACT DESCRIPTORS done" << endl << endl;

    if (dataBuffer.size() > 1)  // Wait until at least two images have been processed
    {
      cout << "Comparing images: " << (opts.imgStartIndex + imgIndex - 1) << " and " << 
        (opts.imgStartIndex + imgIndex) << endl;

      /* MATCH KEYPOINT DESCRIPTORS */

      vector<cv::DMatch> matches;

      float minDescDistanceRatio = 0.8;  // Minimum descriptor distance ratio test, 0. <= x <= 1.

      matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, 
        (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors, matches, opts.descriptorType, 
        opts.descriptorGroup, opts.matcherType, opts.selectorType, minDescDistanceRatio);

      // Store matches in current data frame
      (dataBuffer.end() - 1)->kptMatches = matches;

      cout << "#7: MATCH KEYPOINT DESCRIPTORS done" << endl << endl;
      
      /* TRACK 3D OBJECT BOUNDING BOXES */

      // STUDENT ASSIGNMENT (FP.1)
      
      // Task FP.1: Match list of 3D objects
      map<int, int> bbBestMatches;

      // Associate bounding boxes between current and previous frame using keypoint matches
      matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1));
      
      // END OF STUDENT ASSIGNMENT (FP.1)

      // Store matches in current data frame
      (dataBuffer.end()-1)->bbMatches = bbBestMatches;

      cout << "#8: TRACK 3D OBJECT BOUNDING BOXES done" << endl << endl;


      /* COMPUTE TTC ON OBJECT IN FRONT */

      // Loop over all bounding box match pairs
      for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
      {
        // Find bounding boxes associates with current match
        BoundingBox *prevBB, *currBB;
        for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
        {
          if (it1->second == it2->boxID) // Check whether current match partner corresponds to this bounding box
            currBB = &(*it2);
        }

        for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
        {
          if (it1->first == it2->boxID) // Check whether current match partner corresponds to this bounding box
            prevBB = &(*it2);
        }

        // Compute TTC for current match (if we have LiDAR points)
        if (currBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0) 
        {
          // STUDENT ASSIGNMENT (FP.2)
          
          // Task FP.2: Compute time-to-collision based on LiDAR data
          double ttcLidar; 

          vector<double> frameStats;  // FP.5, FP.6: Container of statistics for later tabulation

          try
          {
            if (imgIndex == 1)  // To display outlier statistics for the first image as well
            {
              vector<LidarPoint> filtered;

              removeOutliers(prevBB->lidarPoints, filtered, opts.filteringMethod, opts.radius, opts.knn, opts.minSize, 
                opts.maxSize, opts.bRenderClusters, opts.bShowRemoved, true);
            }

            computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar, opts.filteringMethod, 
              opts.radius, opts.knn, opts.minSize, opts.maxSize, opts.bRenderClusters, opts.bShowRemoved);
            
            frameStats.push_back(ttcLidar);  // FP.5, FP.6: Push back statistics for later tabulation
          }
          
          catch (length_error e)  // Applies to Euclidean clustering for large imgEndIndex values (>= 48)
          {
            cout << "(!) WARNING: No usable cluster found! Unreliable data, skipping." << endl << endl;
          }
          
          // END OF STUDENT ASSIGNMENT (FP.2)

          // STUDENT ASSIGNMENT (FP.3, FP.4)
          
          double ttcCamera;

          // Task FP.3: Assign enclosed keypoint matches to bounding box
          clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, 
            (dataBuffer.end() - 1)->kptMatches);

          // Task FP.4: Compute time-to-collision based on camera
          computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, currBB->kptMatches, 
            sensorFrameRate, ttcCamera);

          // END OF STUDENT ASSIGNMENT (FP.3, FP.4)

          // FP.5, FP.6: Collect statistics to later tabulate
          frameStats.push_back(ttcCamera);
          ttcStats.push_back(frameStats);

          if (opts.bVisFinalOutput)
          {
            cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();

            if (opts.bVisLidarOverlay)
              showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);

            if (opts.bVisKeypointsOverlay)
              cv::drawKeypoints(visImg, currBB->keypoints, visImg, cv::Scalar::all(-1), 
                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, 
              currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);

            char str[200];
            sprintf(str, "TTC Lidar: %f s, TTC Camera: %f s", ttcLidar, ttcCamera);

            // Font and size specifications were changed to make it more visible on the screen
            putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 255));

            string windowName = "Final Results: TTC";
            cv::namedWindow(windowName, 4);
            cv::imshow(windowName, visImg);

            if (opts.bSaveOutputFrames)  // Save output frames in current working directory
            {
              string saveAs = opts.detectorType + "_" + opts.descriptorType + "_" + imgNumber.str() + imgFileType;
              imwrite(saveAs, visImg);
              cout << endl << "Saved image: " << saveAs << endl << endl;
            }

            cout << "Press key to continue to next frame" << endl << endl;
            cv::waitKey(0);
          }

        } // eof time-to-collision computation
      } // eof loop over all bounding box matches
    }
  } // eof loop over all images

  // Print TTC summary statistics for all image pairs in scope
  printSummaryStats(opts.imgStartIndex, ttcStats);

  return 0;
}
