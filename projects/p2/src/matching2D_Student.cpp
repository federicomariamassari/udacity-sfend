#include <numeric>
#include <stdexcept>
#include "matching2D.hpp"

using namespace std;


/* Detect keypoints in an image using the traditional Shi-Thomasi detector [1].
 *
 * Resources:
 * [1] - Shi, Tomasi: "Good Features to Track" (1994)
 */
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  // Compute detector parameters based on image size
  int blockSize = 4;  // Size of an average block for computing a derivative covariation matrix over each pixel neighborhood
  double maxOverlap = 0.0;  // Maximum permissible % overlap between two features
  double minDistance = (1.0 - maxOverlap) * blockSize;
  int maxCorners = img.rows * img.cols / max(1.0, minDistance);  // Maximum number of keypoints

  double qualityLevel = 0.01;  // Minimal accepted quality of image corners
  double k = 0.04;  // Free parameter of the Harris detector
  bool bUseHarrisDetector = false;

  // Apply corner detection
  double t = (double) cv::getTickCount();
  vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, bUseHarrisDetector, k);

  // Add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it)
  {
    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }

  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Shi-Tomasi detection with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  // Visualize results
  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Shi-Tomasi Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);  // highgui methods do not require cv namespace specification
    cv::waitKey(0);
  }
}

/* Perform Harris' corner detection (1988) with non-maxima suppression (NMS) [1].
 * Adapted from solution to "Harris Corner Detection", Tracking Image Features: Lesson 5, Udacity [2].
 * 
 * Resources:
 *
 * [1] - Harris, Stephens: "A Combined Corner and Edge Detector" (1988)
 * [2] - Harris Corner Detection (Tracking Image Features: Lesson 5), Udacity Sensor Fusion Nanodegree
 * [3] - Addendum - NMS Algorithm (Tracking Image Features: Lesson 6), Udacity Sensor Fusion Nanodegree
 * [4] - Harris Corner Detector Tutorial (https://docs.opencv.org/4.2.0/d4/d7d/tutorial_harris_detector.html)
 */
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
  int blockSize = 2;  // Size of pixel neighborhood considered for corner detection (W)
  int apertureSize = 3;  // Aperture size for the Sobel operator
  int minResponse = 100;  // Pixel is marked as corner iff R = det(H_w) - k*(tr(H_w))^2 >= minResponse
  double k = 0.04;  // Empirical constant in [0.04-0.06]

  cv::Mat dst, dstNorm, dstNormScaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1);

  double t = (double) cv::getTickCount();
  cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

  // Scale each response to 8-bit pixel range (0-255) but keep format to 32-bit floats
  cv::normalize(dst, dstNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

  // Convert response matrix to CV_8UC1 (8-bit unsigned) to display with imshow
  cv::convertScaleAbs(dstNorm, dstNormScaled);

  // Non-maxima suppression algorithm (based on solutions to [2], [3])
  double maxOverlap = 0.;  // No tolerated overlap between windows of currently-analysed keypoints

  for (size_t i=0; i < dstNorm.rows; i++)
  {
    for (size_t j=0; j < dstNorm.cols; j++)
    {
      int response = (int) dstNorm.at<float>(i, j);

      if (response > minResponse)
      {
        cv::KeyPoint newKeyPoint;

        // Coordinates are inverted due to different conventions between 2D-space point representation
        // and matrix indexing (sources: Udacity GPT, https://knowledge.udacity.com/questions/657410)
        newKeyPoint.pt = cv::Point2f(j, i);
        newKeyPoint.size = 2 * apertureSize;  // Constant keypoint neighborhood size
        newKeyPoint.response = response;

        bool bOverlap = false;

        for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
        {
          double keypointOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);

          if (keypointOverlap > maxOverlap)  // Do keypoint windows overlap?
          {
            bOverlap = true;
            *it = newKeyPoint.response > it->response ? newKeyPoint : *it;  // [3]
            break;
          }
        }

        if (!bOverlap)  // No overlap: add keypoint
          keypoints.push_back(newKeyPoint);
      }
    }
  }

  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
  cout << "Harris corner detection with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  if (bVis)
  {
    cv::Mat visImage = dstNormScaled.clone();
    cv::drawKeypoints(dstNormScaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Harris Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}

/* Detect image keypoints using modern algorithms: SIFT, SURF, FAST, ORB, BRISK, AKAZE [1].
 *
 * Resources:
 * [1] - https://docs.opencv.org/4.2.0/d5/d51/group__features2d__main.html
 */
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
  cv::Ptr<cv::FeatureDetector> detector;
  
  if (detectorType.compare("FAST") == 0)  // https://docs.opencv.org/4.2.0/df/d74/classcv_1_1FastFeatureDetector.html
  {
    int threshold = 30;
    bool bUseNonMaxSuppression = true;  // For uniform comparison with Harris, Shi-Tomasi

    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;  // TYPE_5_8, TYPE_7_12
    detector = cv::FastFeatureDetector::create(threshold, bUseNonMaxSuppression, type);

  } else if (detectorType.compare("BRISK") == 0)  // https://docs.opencv.org/4.2.0/de/dbf/classcv_1_1BRISK.html
  {
    int threshold = 30;
    int octaves = 3;
    float patternScale = 1.f;

    detector = cv::BRISK::create(threshold, octaves, patternScale);

  } else if (detectorType.compare("ORB") == 0)  // https://docs.opencv.org/4.2.0/db/d95/classcv_1_1ORB.html
  {
    int nFeatures = 500;
    float scaleFactor = 1.2f;
    int nLevels = 8;
    int edgeFeatures = 31;
    int firstLevel = 0;
    int WTA_K = 2;
    cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
    int patchSize = 31;
    int fastThreshold = 20;

    detector = cv::ORB::create(nFeatures, scaleFactor, nLevels, edgeFeatures, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

  } else if (detectorType.compare("AKAZE") == 0)  // https://docs.opencv.org/4.2.0/d8/d30/classcv_1_1AKAZE.html
  {
    cv::AKAZE::DescriptorType descriptorType = cv::AKAZE::DESCRIPTOR_MLDB; 
    int descriptorSize = 0;
    int descriptorChannels = 3;
    float threshold = .001f; 
    int nOctaves = 4;
    int nOctaveLayers = 4;
    cv::KAZE::DiffusivityType diffusivity = cv:: KAZE::DIFF_PM_G2;

    detector = cv::AKAZE::create(descriptorType, descriptorSize, descriptorChannels, threshold, nOctaves, nOctaveLayers, diffusivity);

  } else if (detectorType.compare("SIFT") == 0)  // https://docs.opencv.org/4.x/d7/d60/classcv_1_1SIFT.html
  {
    int nFeatures = 0;
    int nOctaveLayers = 3;
    double contrastThreshold = 0.04;
    double edgeThreshold = 10;
    double sigma = 1.6;
    bool enable_precise_upscale = false;

    detector = cv::SIFT::create(nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma, enable_precise_upscale);

  } else if (detectorType.compare("SURF") == 0)  // https://docs.opencv.org/4.2.0/d5/df7/classcv_1_1xfeatures2d_1_1SURF.html
  {
    double hessianThreshold = 100;
    int nOctaves = 4;
    int nOctaveLayers = 3;
    bool extended = false;
    bool upright = false;

    detector = cv::xfeatures2d::SURF::create(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);

  } else 
    throw invalid_argument("Invalid detector type. Must be among: SHITOMASI, HARRIS, SIFT, SURF, FAST, ORB, BRISK, AKAZE.");

  // Perform and time keypoint detection
  double t = (double) cv::getTickCount();
  detector->detect(img, keypoints);
  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();

  cout << detectorType + " with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = detectorType + " Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}

/* Use one among several state-of-art descriptors to uniquely identify keypoints.
 */ 
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
  cv::Ptr<cv::DescriptorExtractor> extractor;

  if (descriptorType.compare("BRISK") == 0)
  {
    int threshold = 30;
    int octaves = 3;
    float patternScale = 1.0f;

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
  }

  else if (descriptorType.compare("BRIEF") == 0)
    extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();

  else if (descriptorType.compare("ORB") == 0)
    extractor = cv::ORB::create();

  else if (descriptorType.compare("FREAK") == 0)
    extractor = cv::xfeatures2d::FREAK::create();
  
  else if (descriptorType.compare("AKAZE") == 0)
    extractor = cv::AKAZE::create();

  else if (descriptorType.compare("SIFT") == 0)
    extractor = cv::SiftDescriptorExtractor::create();

  else if (descriptorType.compare("SURF") == 0)
    extractor = cv::xfeatures2d::SURF::create();

  else
    throw invalid_argument("Invalid descriptor type. Must be among: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT.");

  // Perform and time feature description
  double t = (double) cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();

  cout << descriptorType << " Descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

/* Find best matches for keypoints in two camera images based on several matching methods.
 *
 * Resources:
 *
 * [1] - https://docs.opencv.org/4.2.0/d8/d9b/group__features2d__match.html
 * [2] - Exercise - Descriptor Matching (Tracking Image Features: Lesson 12), Udacity Sensor Fusion Nanodegree
 * [3] - https://knowledge.udacity.com/questions/118373
 */
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
  std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
  // Configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;  // https://docs.opencv.org/4.2.0/db/d39/classcv_1_1DescriptorMatcher.html

  if (descriptorType.compare("DES_BINARY") == 1 || descriptorType.compare("DES_HOG") == 1)
  {
    string errorMsg = "Invalid descriptor group " + descriptorType + ". Choose between: DES_BINARY, DES_HOG";
    throw invalid_argument(errorMsg);
  }

  if (matcherType.compare("MAT_BF") == 0)  // https://docs.opencv.org/4.2.0/d3/da1/classcv_1_1BFMatcher.html
  {
    // Use Hamming norm for binary descriptors and L2 norm for histogram-of-gradients ones
    // https://docs.opencv.org/4.2.0/d3/da1/classcv_1_1BFMatcher.html
    int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;

    matcher = cv::BFMatcher::create(normType, crossCheck);
  }

  else if (matcherType.compare("MAT_FLANN") == 0)  // https://docs.opencv.org/4.2.0/dc/de2/classcv_1_1FlannBasedMatcher.html
  {
    // https://knowledge.udacity.com/questions/211123
    if (descSource.type() != CV_32F || descRef.type() != CV_32F)  // Ensure both have valid type
    {
      descSource.convertTo(descSource, CV_32F);
      descRef.convertTo(descRef, CV_32F);
    }

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  }

  else
  {
    string errorMsg = "Invalid matcher type " + matcherType + ". Choose between: MAT_BF, MAT_FLANN";
    throw invalid_argument(errorMsg);
  }

  // Perform matching task
  double t = (double) cv::getTickCount();

  if (selectorType.compare("SEL_NN") == 0)
  {
    // Nearest neighbor (single best match)
    matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1

    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
    cout << selectorType << " with n = " << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
  }

  else if (selectorType.compare("SEL_KNN") == 0)
  { 
    // k-nearest neighbors (k=2)
    vector<vector<cv::DMatch>> knnMatches;  // Two best matches, each in a vector
    matcher->knnMatch(descSource, descRef, knnMatches, 2);

    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
    cout << selectorType << " with n = " << knnMatches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

    // Minimum descriptor distance ratio test (based on solution to [2])
    float minDescDistanceRatio = 0.8;

    // Computed on k-nearest neighbors, not brute force [3]
    for (auto it = knnMatches.begin(); it != knnMatches.end(); ++it)
    {
      if ((*it)[0].distance / (*it)[1].distance < minDescDistanceRatio)
        matches.push_back((*it)[0]);
    }

    cout << "Number of keypoints removed = " << knnMatches.size() - matches.size() << endl;
  }

  else
  {
    string errorMsg = "Invalid selector type " + selectorType + ". Choose between: SEL_NN, SEL_KNN";
    throw invalid_argument(errorMsg);
  }
}
