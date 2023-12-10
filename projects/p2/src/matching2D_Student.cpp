#include "matching2D.hpp"

using namespace std;


void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, const cv::Mat &img, vector<double>& detTickCounts, 
  const bool bVis, const bool bPrintMsg)
{
  // Compute detector parameters based on image size
  int blockSize = 4;  // Average block size to compute derivative covariation matrix over each pixel neighborhood
  double maxOverlap = 0.0;  // Maximum permissible % overlap between two features
  double minDistance = (1.0 - maxOverlap) * blockSize;
  int maxCorners = img.rows * img.cols / max(1.0, minDistance);  // Maximum number of keypoints

  double qualityLevel = 0.01;  // Minimal accepted quality of image corners
  double k = 0.04;  // Free parameter of the Harris detector (unused with bUseHarrisDetector = false)
  bool bUseHarrisDetector = false;

  // Apply corner detection
  double t = (double) cv::getTickCount();
  vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, 
    bUseHarrisDetector, k);

  // Add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it)
  {
    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }

  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();

  // Capture statistic to enable average time spent on keypoint detection analysis
  detTickCounts.push_back(t);

  if (bPrintMsg)
    cout << "Shi-Tomasi detection with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

  // Visualize results
  if (bVis)
  {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Shi-Tomasi Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);  // highgui methods don't require cv namespace specification (source: Udacity GPT)
    cv::waitKey(0);
  }
}

void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, const cv::Mat &img, vector<double>& detTickCounts, 
  const bool bVis, const bool bPrintMsg)
{
  int blockSize = 2;  // Size of pixel neighborhood considered for corner detection (W)
  int apertureSize = 3;  // Aperture size for the Sobel operator (must be odd)
  int minResponse = 100;  // Pixel is marked as corner iff R = det(H_w) - k*(tr(H_w))^2 >= minResponse
  double k = 0.04;  // Empirical constant in [0.04-0.06], keep low for best outcome/time trade-off

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
  detTickCounts.push_back(t);
  
  if (bPrintMsg)
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

void detKeypointsModern(vector<cv::KeyPoint> &keypoints, const cv::Mat &img, const string detectorType, 
  vector<double>& detTickCounts, const bool bVis, const bool bPrintMsg)
{
  cv::Ptr<cv::FeatureDetector> detector;
  
  if (detectorType.compare("FAST") == 0)  // https://docs.opencv.org/4.2.0/df/d74/classcv_1_1FastFeatureDetector.html
  {
    int threshold = 20;  // Improved accuracy compared to default 10
    bool bUseNonMaxSuppression = true;  // For uniform comparison with Harris and Shi-Tomasi
    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;

    detector = cv::FastFeatureDetector::create(threshold, bUseNonMaxSuppression, type);

  } else if (detectorType.compare("BRISK") == 0)  // https://docs.opencv.org/4.2.0/de/dbf/classcv_1_1BRISK.html
  {
    // Keeping default parameter values for most implementations
    detector = cv::BRISK::create();

  } else if (detectorType.compare("ORB") == 0)  // https://docs.opencv.org/4.2.0/db/d95/classcv_1_1ORB.html
  {
    detector = cv::ORB::create();

  } else if (detectorType.compare("AKAZE") == 0)  // https://docs.opencv.org/4.2.0/d8/d30/classcv_1_1AKAZE.html
  {
    detector = cv::AKAZE::create();

  } else if (detectorType.compare("SIFT") == 0)  // https://docs.opencv.org/4.x/d7/d60/classcv_1_1SIFT.html
  {
    detector = cv::SIFT::create();

  } else if (detectorType.compare("SURF") == 0) // https://docs.opencv.org/4.2.0/d5/df7/classcv_1_1xfeatures2d_1_1SURF.html
  {
    detector = cv::xfeatures2d::SURF::create();

  } else 
    throw invalid_argument("Invalid detector. Must be among: SHITOMASI, HARRIS, SIFT, SURF, FAST, ORB, BRISK, AKAZE.");

  // Perform and time keypoint detection
  double t = (double) cv::getTickCount();
  detector->detect(img, keypoints);
  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();

  detTickCounts.push_back(t);

  if (bPrintMsg)
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

void descKeypoints(vector<cv::KeyPoint> &keypoints, const cv::Mat &img, cv::Mat &descriptors, const string detectorType, 
  const string descriptorType, vector<double>& descTickCounts, const bool bPrintMsg)
{
  if ((detectorType == "SIFT" && descriptorType == "ORB") || (detectorType == "ORB" && descriptorType == "SIFT"))
  {
    // https://knowledge.udacity.com/questions/105392
    string errorMsg = detectorType + " detector does not work with " + descriptorType + " descriptor.";
    throw invalid_argument(errorMsg);
  }

  if (descriptorType == "AKAZE" && detectorType != "AKAZE")
    throw invalid_argument("AKAZE descriptor only works with AKAZE detector.");

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
    throw invalid_argument("Invalid descriptor type. Must be among: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT, SURF.");

  // Perform and time feature description
  double t = (double) cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();

  descTickCounts.push_back(t);

  if (bPrintMsg)
    cout << descriptorType << " Descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

void matchDescriptors(const vector<cv::KeyPoint> &kPtsSource, const vector<cv::KeyPoint> &kPtsRef, 
  const cv::Mat &descSource, const cv::Mat &descRef, vector<cv::DMatch> &matches, const string descriptorType,
  string descriptorGroup, const string matcherType, const string selectorType, vector<double>& rejected, 
  int &infoCounter, const float minDescDistanceRatio, const bool bPrintMsg)
{
  // Configure matcher
  bool crossCheck = false;  // false to avoid bypassing caught BF error (https://knowledge.udacity.com/questions/243086)
  cv::Ptr<cv::DescriptorMatcher> matcher;  // https://docs.opencv.org/4.2.0/db/d39/classcv_1_1DescriptorMatcher.html

  if (descriptorGroup.compare("DES_BINARY") != 0 && descriptorGroup.compare("DES_HOG") != 0)
  {
    string errorMsg = "Invalid descriptor group " + descriptorGroup + ". Choose between: DES_BINARY, DES_HOG";
    throw invalid_argument(errorMsg);
  }

  if (matcherType.compare("MAT_BF") == 0)  // https://docs.opencv.org/4.2.0/d3/da1/classcv_1_1BFMatcher.html
  {
    // https://answers.opencv.org/question/10046/feature-2d-feature-matching-fails-with-assert-statcpp/
    // https://knowledge.udacity.com/questions/117307
    if ((descriptorType.compare("SIFT") == 0 || descriptorType.compare("SURF") == 0) 
      && descriptorGroup.compare("DES_BINARY") == 0)
    {
      descriptorGroup = "DES_HOG";

      if (infoCounter == 0)
      {
        cerr << "(!) WARNING: " << descriptorGroup << " is incompatible with SIFT/SURF. Switching to DES_HOG for " << 
          descriptorType << "." << endl;
        infoCounter++;
      }
    }

    // Use Hamming norm for binary descriptors and L2 norm for histogram-of-gradients ones
    int normType = descriptorGroup.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;

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
    
    if (bPrintMsg)
      cout << selectorType << " with n = " << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
  }

  else if (selectorType.compare("SEL_KNN") == 0)
  { 
    // k-nearest neighbors (k=2)
    vector<vector<cv::DMatch>> knnMatches;  // Two best matches, each in a vector
    matcher->knnMatch(descSource, descRef, knnMatches, 2);

    t = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
    
    if (bPrintMsg)
      cout << selectorType << " with n = " << knnMatches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

    // Computed on k-nearest neighbors, not brute force [3]
    for (auto it = knnMatches.begin(); it != knnMatches.end(); ++it)
    {
      if ((*it)[0].distance / (*it)[1].distance < minDescDistanceRatio)
        matches.push_back((*it)[0]);
    }

    if (bPrintMsg)
      cout << "Number of keypoints removed = " << knnMatches.size() - matches.size() << endl;

    // MP.8: Keep track of the rejected outliers [4]
    rejected.push_back(knnMatches.size() - matches.size());
  }

  else
  {
    string errorMsg = "Invalid selector type " + selectorType + ". Choose between: SEL_NN, SEL_KNN";
    throw invalid_argument(errorMsg);
  }
}

/*********************************************************************************************************************
 * CUSTOM ADDITIONS
 *********************************************************************************************************************/

DataFrame loadImageIntoBuffer(const string filename, vector<DataFrame>& dataBuffer, const int dataBufferSize)
{
  // Load image from file and convert to grayscale
  cv::Mat img, imgGray;
  img = cv::imread(filename);
  cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

  // TASK MP.1: Implement ring buffer of size dataBufferSize

  DataFrame frame;
  frame.cameraImg = imgGray;

  if (dataBuffer.size() == dataBufferSize)

    // https://knowledge.udacity.com/questions/644337
    dataBuffer.erase(dataBuffer.begin());
  
  dataBuffer.push_back(frame);

  // End of TASK MP.1

  return frame;
}

vector<cv::KeyPoint> detectKeypoints(const string detectorType, cv::Mat& img, vector<double>& detTickCounts, 
  const bool bVis, const bool bPrintMsg)
{
  // To hold 2D keypoints extracted from current image
  vector<cv::KeyPoint> keypoints;

  // TASK MP.2: Add keypoints HARRIS, FAST, BRISK, ORB, AKAZE, SIFT and enable string-based selection on detectorType

  if (detectorType.compare("SHITOMASI") == 0)
    detKeypointsShiTomasi(keypoints, img, detTickCounts, bVis, bPrintMsg);

  else if (detectorType.compare("HARRIS") == 0)
    detKeypointsHarris(keypoints, img, detTickCounts, bVis, bPrintMsg);

  else
    detKeypointsModern(keypoints, img, detectorType, detTickCounts, bVis, bPrintMsg);

  // End of TASK MP.2

  return keypoints;
}

void focusOnArea(vector<cv::KeyPoint>& keypoints, const bool bFocusOnVehicle, const bool bPrintMsg)
{
  // TASK MP.3: Only keep keypoints in a rectangle centered on the preceding vehicle

  // Will also include side mirror of left vehicle and partial shadow of the car in front
  cv::Rect vehicleRect(535, 180, 180, 150);

  if (bFocusOnVehicle)
  {
    vector<cv::KeyPoint> rectKeyPoints;

    for (const auto& keypoint : keypoints)
    {
      if (vehicleRect.contains(keypoint.pt))
        rectKeyPoints.push_back(keypoint);
    }

    keypoints = rectKeyPoints;

    if (bPrintMsg)
      cout << "Number of keypoints in the region of interest = " << keypoints.size() << endl;
  }

  // End of TASK MP.3
}

void limitKeypoints(vector<cv::KeyPoint>& keypoints, const string detector, const int maxKeypoints)
{
  if (detector.compare("SHITOMASI") == 0)

    // There is no response info, so keep the first 50 as they are sorted in descending quality order
    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());

  cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
}

void extractNeighborhoodSizes(const vector<cv::KeyPoint>& keypoints, vector<double>& neighborhoodSizes)  // MP.7
{
  for (const auto& keypoint : keypoints)
    neighborhoodSizes.push_back(keypoint.size);
}

double percentile(const vector<double>& vec, const float p)  // MP.7
{
  int vec_size = vec.size();
  double res;

  switch(vec_size % 2)
  {
    case 0:
      res = vec[vec_size * p];
      break;

    case 1:
      res = 0.5 * (vec[floor(vec_size * p)] + vec[floor(vec_size * p) + 1]);
      break;
  }

  return res;
}

double computeMode(const vector<double>& vec)  // MP.7
{
  double mode;
  double counter = 1;
  double running_max = 1;

  for (size_t i=0; i < (vec.size()-1); i++)
  {
    if (vec[i+1] == vec[i])
    {
      counter++;
      running_max = (counter > running_max) ? counter : running_max;
      mode = vec[i];
    }
    else
      counter = 1;
  }

  return mode;
}

void computeStatistics(Stats& s, vector<double>& vec)  // MP.7
{
  // Ensure input distribution is sorted before computing statistics
  sort(vec.begin(), vec.end());

  double mean = accumulate(vec.begin(), vec.end(), 0.) / vec.size();

  double squared_dev = accumulate(vec.begin(), vec.end(), 0., 
    [mean](double sum, const double x) { return sum + pow(x - mean, 2); });

  double mode = computeMode(vec);

  // Append detector's statistics to passed Stats struct
  s.vec_size.push_back(vec.size());
  s.kpt_img.push_back(vec.size() / s.img_count);

  s.mean.push_back(mean);
  s.median.push_back(percentile(vec, .50));
  s.mode.push_back(mode);

  s.std_dev.push_back(sqrt(squared_dev / (vec.size() - 1)));  // Sample standard deviation

  s.min.push_back(vec[0]);
  s.max.push_back(vec.back());
  s.range.push_back(vec.back() - vec[0]);

  double p25 = percentile(vec, .25);
  double p75 = percentile(vec, .75);

  s.p25.push_back(p25);
  s.p75.push_back(p75);
  s.iqr.push_back(p75 - p25);
}

template<typename T>
void printLine(const pair<const string, vector<T>>& p)  // MP.7
{
  cout << left << setw(30) << p.first;

  for (const auto& value : p.second)
    cout << right << setw(12) <<  value;

  cout << endl;
}

template<typename T>
void printLine(const vector<T> v, const int& imgEndIndex)  // MP.8-9
{
  int counter = 0;
  for (const auto& value : v)
  {
    cout << right << ((counter < imgEndIndex) ? setw(6) : setw(12)) << value;
    counter++;
  }
  cout << endl;
}

void printStatistics(const vector<string>& detectors, const Stats& s)  // MP.7
{
  // Populate pairs
  pair<string, vector<string>> header("DETECTOR", detectors);

  vector<pair<string, vector<double>>> v {

    pair<string, vector<double>>("Average # keypoints/image", s.kpt_img),
    pair<string, vector<double>>("Average detection time (ms)", s.avg_det_time),

    pair<string, vector<double>>("Mean", s.mean),
    pair<string, vector<double>>("Median", s.median),
    pair<string, vector<double>>("Mode", s.mode),

    pair<string, vector<double>>("Standard deviation", s.std_dev),

    pair<string, vector<double>>("Min", s.min),
    pair<string, vector<double>>("Max", s.max),
    pair<string, vector<double>>("Range", s.range),

    pair<string, vector<double>>("25th percentile", s.p25),
    pair<string, vector<double>>("75th percentile", s.p75),
    pair<string, vector<double>>("IQR", s.iqr)
  };

  // Custom length based on the number of detectors
  cout << string(30 + 12*detectors.size(), '*') << endl;

  printLine<string>(header);

  cout << string(30 + 12*detectors.size(), '*') << endl;

  printLine<int>(pair<string, vector<int>>("Total # of keypoints", s.vec_size));

  for (const auto& pair : v)
    printLine<double>(pair);
}

void printStatistics(const map<string, vector<double>>& m, const int& imgEndIndex, const string sep)  // MP.8-9
{
  // Print header
  vector<string> header {"0-1", "1-2", "2-3", "3-4", "4-5", "5-6", "6-7", "7-8", "8-9", "# MATCHES", "REJECTED", 
    "DET.TIME", "DESC.TIME", "TOTAL TIME"};

  cout << string(6 * (14 + imgEndIndex), '*') << endl;
  cout << left << setw(12) << "DETECTOR" << left << setw(12) << "DESCRIPTOR";

  printLine(header, imgEndIndex);
  cout << string(6 * (14 + imgEndIndex), '*') << endl;

  for (const auto& pair : m)
  {
    // Source: Udacity GPT
    string s = pair.first;
    size_t pos = s.find(sep);

    // Detector, descriptor
    cout << left << setw(12) << s.substr(0, pos) << left << setw(12) << s.substr(pos + sep.length());
    
    // Breakdown of matched keypoints and total time spent
    printLine(pair.second, imgEndIndex);
  }

  cout << endl;
}
