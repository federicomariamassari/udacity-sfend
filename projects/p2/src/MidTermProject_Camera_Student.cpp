#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;


/**
 * Customise main output
 */
struct Options
{
  string detectorType = "FAST";  // HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, SIFT, SURF
  string descriptorType = "BRIEF";  // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT, SURF
  string descriptorGroup = "DES_HOG";  // DES_HOG, DES_BINARY

  string matcherType = "MAT_BF";  // MAT_BF, MAT_FLANN
  string selectorType = "SEL_KNN";  // SEL_NN, SEL_KNN

  float minDescDistanceRatio = 0.8;  // Minimum descriptor distance ratio test, 0. <= x <= 1.
  
  bool bCompareDetectors = true;  // true for MP.7, false to print statistics on a single detector
  bool bCompareDescriptors = true; // true for MP.8-9, false to use a single descriptor

  bool bVis = false;  // true to display the full array of keypoints found in each image
  bool bVisMatches = true;  // true to view matched keypoints among image pairs (single detector-descriptor only,
                            // adds a significant overhead to tick counts)

  bool bSaveImagePairs = false;  // true to write image pairs in the current working directory
  bool bLimitKpts = false;  // true to limit the number of keypoints (helpful for debugging and learning)
};

int main(int argc, const char *argv[])
{
  Options options = Options();

  if (options.bCompareDescriptors)  // MP.8: Ensure task is performed with the required parameters
  {
    options.matcherType = "MAT_BF";
    options.selectorType = "SEL_KNN";
    options.minDescDistanceRatio = 0.8;

    string msg = "Setting matcherType='" + options.matcherType + "' and selectorType='" + options.selectorType + "', "
     + "minimum descriptor distance ratio: " + to_string(options.minDescDistanceRatio);

    cerr << endl << "(!) WARNING: Descriptor comparison is ON. " << msg << endl;
  }

  if (options.bLimitKpts)
    cerr << endl << "(!) WARNING: Limit Keypoints is ON. Keypoints will be limited!" << endl;

  cout << endl;
  cout << "Keypoint detector: " << options.detectorType << endl;
  cout << "Keypoint descriptor: " << options.descriptorType << endl;
  cout << "Descriptor group: " << options.descriptorGroup << endl;
  cout << "Feature matcher: " << options.matcherType << endl;
  cout << "Selector type: " << options.selectorType << endl;
  
  // Data location
  string dataPath = "../";
  string sep = string(80, '*');

  // Camera
  string imgBasePath = dataPath + "images/";
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";  // Left camera, color
  string imgFileType = ".png";
  
  // Assume LiDAR and camera have same naming convention
  int imgStartIndex = 0;
  int imgEndIndex = 9;
  int imgFillWidth = 4;  // Number of digits which make up file index (e.g., img-0001.png)

  int imgCount = (imgEndIndex - imgStartIndex) + 1;  // MP.7, MP.9: To compute average statistics

  // Number of images held in memory at the same time (ring buffer)
  int dataBufferSize = 2;

  // Do not clutter display when performing detector-descriptor comparison (MP.7, MP.8)
  bool bPrintLogs = !(options.bCompareDetectors || options.bCompareDescriptors);

  // Push detector and descriptor types to vectors to tabulate statistics output
  vector<string> detectorTypes, descriptorTypes;

  if (options.bCompareDetectors)  // MP.7
    detectorTypes = {"HARRIS", "SHITOMASI", "FAST", "BRISK", "ORB", "AKAZE", "SIFT", "SURF"};
  else
    detectorTypes.push_back(options.detectorType);

  if (options.bCompareDescriptors)  // MP.8
    descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT", "SURF"};
  else
    descriptorTypes.push_back(options.descriptorType);

  // MP.8-9: To hold vectors of image-pairs matched keypoints and total time for all detector-descriptor combinations
  map<string, vector<double>> featureTrackingResMap;

  bool isTableDisplayed = false; // Display MP.7 table only once when also performing MP.8

  for (const string descriptorType : descriptorTypes)  // MP.8
  {
    // To hold average keypoint detection and description time (one entry per detector/descriptor combination)
    vector<double> avgDetTickCounts, avgDescTickCounts;

    // To hold the distributions of neighborhood sizes (one row per detector)
    vector<vector<double>> neighborhoodSizesMatrix;

    // To hold statistics for each detector (one struct per detector)
    vector<Stats> detectorsStats;

    for (const string detectorType : detectorTypes)  // MP.7
    {
      // List of data frames held in memory simultaneously
      vector<DataFrame> dataBuffer;

      // MP.7: Holds the full set of neighborhood sizes across images to calculate average distribution statistics
      vector<double> neighborhoodSizes;

      // MP.7, MP.9: Hold full sets of detection and description tick counts, to compute average time spent per image
      vector<double> detTickCounts, descTickCounts;

      // MP.8-9: Vector of matched keypoints and total time taken for the current detector-descriptor combination
      vector<double> featureTrackingRes;

      // MP.8: To keep track of the rejected outliers
      vector<double> rejected;

      // To display a particular warning or info message only once
      int warningCounter = 0;
      int infoCounter = 0;

      for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
      {
        // Assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        if (bPrintLogs)
          cout << endl << sep << endl << "Loading image: " << imgIndex << endl << sep << endl;

        // MP.1
        DataFrame frame = loadImageIntoBuffer(imgFullFilename, dataBuffer, dataBufferSize);

        if (bPrintLogs)
          cout << "#1: LOAD IMAGE INTO BUFFER done" << endl << endl;

        /* DETECT IMAGE KEYPOINTS */

        // MP.2: Wrapper function that includes all possible detector choices
        vector<cv::KeyPoint> keypoints = detectKeypoints(detectorType, frame.cameraImg, detTickCounts, options.bVis, 
          bPrintLogs);

        // MP.3: Restrict the focus on the preceding vehicle
        focusOnArea(keypoints, true, bPrintLogs);

        if (options.bLimitKpts)
          // Note: FAST, SIFT may occasionally include 51 or 52 keypoints despite the cap, as some images have
          // particularly rich features the algorithm considers distinctive (source: Udacity GPT)
          limitKeypoints(keypoints, detectorType, 50);

        // Push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;

        if (bPrintLogs)
          cout << "#2: DETECT KEYPOINTS done" << endl;

        // MP.7: Cumulate neighborhood sizes for average statistics
        // Diameter is constant for Harris, Shi-Tomasi, and FAST (https://knowledge.udacity.com/questions/1021073)
        extractNeighborhoodSizes(keypoints, neighborhoodSizes);

        /* EXTRACT KEYPOINT DESCRIPTORS */

        // MP.4: Add descriptors BRIEF, ORB, FREAK, AKAZE, SIFT in file matching2D.cpp and enable string-based 
        // selection based on descriptorType
        cv::Mat descriptors;

        try {
          descKeypoints((dataBuffer.end()-1)->keypoints, (dataBuffer.end()-1)->cameraImg, descriptors, detectorType, 
            descriptorType, descTickCounts, bPrintLogs);
        }

        catch (invalid_argument& e)
        {
          if (warningCounter == 0)
          {
            cout << "(!) WARNING: " << e.what() << " Skipping for " << detectorType << "." << endl;
            warningCounter++;
          }

          continue;
        }

        // End of TASK MP.4

        // Push descriptors for current frame to end of data buffer
        (dataBuffer.end()-1)->descriptors = descriptors;

        if (bPrintLogs)
          cout << "#3: EXTRACT DESCRIPTORS done" << endl << endl;

        if (dataBuffer.size() > 1)  // Wait until at least two images have been processed
        {
          /* MATCH KEYPOINT DESCRIPTORS */

          vector<cv::DMatch> matches;

          if (dataBuffer.size() == dataBufferSize && bPrintLogs)
            cout << "Comparing images: " << imgIndex-1 << " and " << imgIndex << endl;          

          // MP.5: Add FLANN matching in file matching2D.cpp
          // MP.6: Add KNN match selection and descriptor distance ratio filtering with t=0.8 in matching2D.cpp

          matchDescriptors((dataBuffer.end()-2)->keypoints, (dataBuffer.end()-1)->keypoints, 
            (dataBuffer.end()-2)->descriptors, (dataBuffer.end()-1)->descriptors, matches, descriptorType, 
            options.descriptorGroup, options.matcherType, options.selectorType, rejected, infoCounter, 
            options.minDescDistanceRatio, bPrintLogs);

          // End of tasks MP.5-MP.6 (see matching2D.cpp)

          featureTrackingRes.push_back(matches.size());  // MP.8

          // Store matches in current data frame
          (dataBuffer.end() - 1)->kptMatches = matches;

          if (bPrintLogs)
            cout << "#4: MATCH KEYPOINT DESCRIPTORS done" << endl << endl;

          // Do not display image pairs unless performing single detector-descriptor analysis
          if(!(options.bCompareDetectors || options.bCompareDescriptors))
          {
            // Visualize matches between current and previous image
            if (options.bVisMatches)
            {
              cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
              cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints, 
                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints, matches, matchImg, 
                cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

              string windowName = "Matching keypoints between two camera images";
              cv::namedWindow(windowName, 7);
              imshow(windowName, matchImg);

              if (options.bSaveImagePairs)  // Save matched keypoints image pairs in current working directory
              {
                string saveAs = detectorType + "_" + descriptorType + "_" + imgNumber.str() + imgFileType;
                imwrite(saveAs, matchImg);
              }

              cout << "Press any key to continue to next image" << endl;
              cv::waitKey(0);  // Wait for key to be pressed
            }
          }
        }

        else
          if (bPrintLogs)
            cout << "#4: MATCH KEYPOINT DESCRIPTORS skipped (no image to compare to)" << endl;

      }  // end for loop images

      neighborhoodSizesMatrix.push_back(neighborhoodSizes);

      // MP.9: Total time taken for keypoint detection and descriptor for the chosen combination
      double sumDetTickCounts = accumulate(detTickCounts.begin(), detTickCounts.end(), 0.) * 1000;  // in ms
      double sumDescTickCounts = accumulate(descTickCounts.begin(), descTickCounts.end(), 0.) * 1000;

      avgDetTickCounts.push_back(sumDetTickCounts / detTickCounts.size());

      // Disregard invalid detector-descriptor combinations
      if (!featureTrackingRes.empty())
      {
        // MP.8: Sum the number of matched keypoints and rejected outliers across image pairs
        double totalMatches = accumulate(featureTrackingRes.begin(), featureTrackingRes.end(), 0.);
        featureTrackingRes.push_back(totalMatches);

        double totalRejected = accumulate(rejected.begin(), rejected.end(), 0.);
        featureTrackingRes.push_back(totalRejected);

        // Push back matching ratio as a percentage
        featureTrackingRes.push_back(totalMatches / (totalMatches + totalRejected) * 100);

        // MP.9: Also push back total time spent on detection, description, and combined
        featureTrackingRes.push_back(sumDetTickCounts);
        featureTrackingRes.push_back(sumDescTickCounts);
        featureTrackingRes.push_back(sumDetTickCounts + sumDescTickCounts);

        featureTrackingResMap[detectorType + "-" + descriptorType] = featureTrackingRes;
      }
    }  // end for loop detectors

    if (!isTableDisplayed)
    {
      // MP.7: Create structure to hold statistics on keypoints' neighborhood distribution(s)
      Stats stats;
      stats.img_count = imgCount;
      stats.avg_det_time = avgDetTickCounts;  // Will use averages from last detector-descriptor combination

      cout << endl;
      cout << "(MP.7) Performance Evaluation 1: Distribution of Keypoints' Neighborhood Sizes" << endl;
      cout << endl;

      for (size_t i=0; i < detectorTypes.size(); i++)
        computeStatistics(stats, neighborhoodSizesMatrix[i]);

      // MP.7: Pretty-print detectors' statistics
      printStatistics(detectorTypes, stats);

      cout << endl;
    }

    isTableDisplayed = true;  // Display MP.7 table only once when also performing MP.8

  }  // end for loop descriptors

  // https://knowledge.udacity.com/questions/118373
  cout << endl;
  cout << "(MP.8) Performance Evaluation 2: Matched(*) keypoints for all detector-descriptor combinations" << endl;
  cout << "(MP.9) Performance Evaluation 3: Total time taken for keypoint detection and descriptor extraction" << endl;
  cout << endl;

  printStatistics(featureTrackingResMap, imgEndIndex);

  cout << "(*) Brute force matching, k-nearest neighbors (k=2), descriptor distance ratio: 0.8. " << endl << endl;

  return 0;
}