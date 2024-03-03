#include "camFusion.hpp"
#include "lidarData.hpp"

using namespace std;


void clusterLidarWithROI(vector<BoundingBox>& boundingBoxes, vector<LidarPoint>& lidarPoints, float shrinkFactor, 
  cv::Mat& P_rect_xx, cv::Mat& R_rect_xx, cv::Mat& RT)
{
  // Time the LiDAR-ROI clustering process
  auto startTime = chrono::steady_clock::now();

  // Loop over all LiDAR points and associate them to a 2D bounding box
  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
  {
    // Assemble vector for matrix-vector multiplication (homogeneous coordinates)
    X.at<double>(0, 0) = it1->x;
    X.at<double>(1, 0) = it1->y;
    X.at<double>(2, 0) = it1->z;
    X.at<double>(3, 0) = 1;

    // Project LiDAR point into camera
    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;

    // Pixel coordinates
    pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
    pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

    // Pointers to all bounding boxes which enclose the current LiDAR point
    vector<vector<BoundingBox>::iterator> enclosingBoxes;

    for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
    {
      // Shrink current bounding box slightly to avoid having too many outlier points around the edges
      cv::Rect smallerBox;
      smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
      smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
      smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
      smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

      // Check whether point is within current bounding box
      if (smallerBox.contains(pt))
        enclosingBoxes.push_back(it2);

    }  // eof loop over all bounding boxes

    // Check whether point has been enclosed by one or by multiple boxes
    if (enclosingBoxes.size() == 1)

      // Add LiDAR point to bounding box (discard otherwise)
      enclosingBoxes[0]->lidarPoints.push_back(*it1);

  }  // eof loop over all LiDAR points

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "LiDAR point cloud clustering took: " << elapsedTime.count() / 1000. << " ms" << endl;
}

void show3DObjects(vector<BoundingBox>& boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bSaveLidarTopView, 
  string saveAs, bool bWait)
{
  // Create top-view image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));  // Image is white by default

  for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1)
  {
    // Create randomized color for current 3D object
    cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

    // Plot LiDAR points into top-view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0; 
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;

    for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
    {
      // World coordinates
      float xw = (*it2).x; // World position in m with x facing forward from sensor
      float yw = (*it2).y; // World position in m with y facing left from sensor
      xwmin = (xwmin < xw) ? xwmin : xw;
      ywmin = (ywmin < yw) ? ywmin : yw;
      ywmax = (ywmax > yw) ? ywmax : yw;

      // Top-view coordinates (conversion between coordinate systems, sensor-to-image or meter-to-pixel)
      int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
      int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

      // Find enclosing rectangle
      top = (top < y) ? top : y;
      left = (left < x) ? left : x;
      bottom = (bottom > y) ? bottom : y;
      right = (right > x) ? right : x;

      // Draw individual point
      cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
    }

    // Draw enclosing rectangle
    cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0, 0, 0), 2);

    // Augment object with some key data
    char str1[200], str2[200];
    sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int) it1->lidarPoints.size());
    putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 1, currColor);
    sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
    putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 100), cv::FONT_ITALIC, 1, currColor);  
  }

  // Plot distance markers
  float lineSpacing = 2.0;  // Gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);

  for (size_t i = 0; i < nMarkers; ++i)
  {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
  }

  // Display image
  string windowName = "3D Objects";
  cv::namedWindow(windowName, 1);
  cv::imshow(windowName, topviewImg);

  if (bSaveLidarTopView)
  {
    imwrite(saveAs, topviewImg);
    cout << "Saved image: " << saveAs << endl;
  }

  if (bWait)
    cv::waitKey(0);  // Wait for key to be pressed
}

// FP.1
void matchBoundingBoxes(vector<cv::DMatch>& matches, map<int, int>& bbBestMatches, DataFrame& prevFrame, 
  DataFrame& currFrame)
{
  // Time the bounding box matching process
  auto startTime = chrono::steady_clock::now();

  map<pair<int, int>, int> idPairsCount;  // [1]

  for (const auto& match : matches)
  {
    cv::KeyPoint prevPoint = prevFrame.keypoints[match.queryIdx];  // [2] [3]
    cv::KeyPoint currPoint = currFrame.keypoints[match.trainIdx];

    // Early termination once a keypoint correspondence is found is not recommended because nmsThreshold = 0.4
    // does not guarantee that a keypoint is contained in one and only one bounding box
    for (const BoundingBox& prevBB : prevFrame.boundingBoxes)
    {
      // Keypoint is in the previous frame bounding box
      if (prevBB.roi.contains(prevPoint.pt))
      {
        for (const BoundingBox& currBB : currFrame.boundingBoxes)
        {
          // Keypoint is also in the current frame bounding box
          if (currBB.roi.contains(currPoint.pt))
            idPairsCount[make_pair(prevBB.boxID, currBB.boxID)]++;  // We have a correspondence
        }
      }
    }
  }

  map<int, int> highestCounts;

  for (const auto& item : idPairsCount)
  {
    int prevId = item.first.first, currId = item.first.second, pairCount = item.second;

    if (pairCount > highestCounts[prevId])  // For same previous bounding box, has the new pair a higher count?
    {
      highestCounts[prevId] = pairCount;  // If so, (temporarily) set as best match for that previous bounding box
      bbBestMatches[prevId] = currId;
    }
  }

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "Bounding box matching took: " << elapsedTime.count() / 1000. << " ms" << endl;
}

// FP.2, FP.3, FP.4
double percentile(const vector<double>& vec, const float q)
{
  // https://axibase.com/use-cases/workshop/percentiles.html#r7-linear-interpolation

  int vecSize = vec.size();

  if (vecSize == 0)
    throw length_error("Input vector is empty.");

  if (q < 0. || q > 1.)
    throw out_of_range("Percentile must be a real number in [0; 1].");

  if (q == 0.)  // Fast bounds calculation
    return vec.front();

  if (q == 1.)
    return vec[vecSize - 1];

  if (q == 0.5)  // Fast median calculation
  {
    if (vecSize % 2 == 1)
      return vec[vecSize * q];
    else
      return 0.5 * (vec[vecSize * q - 1] + vec[vecSize * q]);
  }

  double index = (vecSize - 1) * q;
  return vec[floor(index)] + (index - floor(index)) * (vec[ceil(index)] - vec[floor(index)]);
}

// FP.2, FP.5
double mean(const vector<double>& vec)
{
  return accumulate(vec.begin(), vec.end(), 0.) / vec.size();
}

// FP.5
double variance(const vector<double>& vec)
{
  double mu = mean(vec);

  return accumulate(vec.begin(), vec.end(), 0., 
    [mu](double sum, const double x) { return sum + pow(x - mu, 2); }) / (vec.size() - 1);
}

// FP.5
double skewness(const vector<double>& vec)
{
  double mu = mean(vec);

  double numerator = accumulate(vec.begin(), vec.end(), 0., 
    [mu](double sum, const double x) { return sum + pow(x - mu, 3); }) / vec.size();

  double denominator = pow(variance(vec), 1.5);

  return numerator / denominator;
}

// FP.5
double excess_kurtosis(const vector<double>& vec)
{
  double mu = mean(vec);

  double numerator = accumulate(vec.begin(), vec.end(), 0., 
    [mu](double sum, const double x) { return sum + pow(x - mu, 4); }) / vec.size();

  double denominator = pow(variance(vec), 2);

  return numerator / denominator - 3;  // As 3 is the kurtosis of the symmetric Gaussian distribution
}

// FP.2
void renderClusters(const vector<vector<LidarPoint>>& clusters, const vector<vector<LidarPoint>>& removed, 
  bool bShowRemoved)
{
  cv::viz::Viz3d window("Filtered LiDAR point cloud");
  window.setBackgroundColor(cv::viz::Color::black());

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    vector<cv::Point3f> points;

    for (const auto& point : clusters[i])
      points.push_back(cv::Point3f(point.x, point.y, point.z));

    cv::viz::WCloud cloud(points);
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6.0);

    // Rotate color among clusters (to extend: i % n == k, for all k = 0, ..., n-1)
    cv::viz::Color color;

    if (i % 3 == 0)
      color = cv::viz::Color::red();  // [1]
    
    else if (i % 3 == 1)
      color = cv::viz::Color::blue();
    
    else
      color = cv::viz::Color::green();

    cloud.setColor(color);

    string cloudName = "cloud" + to_string(i);
    window.showWidget(cloudName, cloud);
  }

  if (bShowRemoved)  // Removed clusters will be colorless  
  {
    vector<cv::Point3f> discarded;

    for (size_t i = 0; i < removed.size(); ++i)
    {
      for (const auto& point : removed[i])
        discarded.push_back(cv::Point3f(point.x, point.y, point.z));
    }

    cv::viz::WCloud cloud(discarded);
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6.0);
    cloud.setColor(cv::viz::Color::white());
    
    window.showWidget("discarded", cloud);
  }

  window.spin();  // Trigger event loop
}

// FP.2
template<typename T>
void clusterHelper(int index, const vector<LidarPoint>& src, const cv::Mat& srcMat, vector<LidarPoint>& cluster, 
  vector<bool>& processed, cv::flann::GenericIndex<T>& tree, float radius, int knn, int minSize, int maxSize)
{
  processed[index] = true;
  cluster.push_back(src[index]);

  // Will contain (sorted) ids and distances of points close enough to query point, -1 elsewhere
  cv::Mat nearest (1, knn, cv::DataType<int>::type, cv::Scalar::all(-1));
  cv::Mat distances (1, knn, cv::DataType<float>::type, cv::Scalar::all(-1));

  // Square radius as we are using L2-norm [4]; for cvflann::SearchParams see [5]
  tree.radiusSearch(srcMat.row(index), nearest, distances, radius*radius, cvflann::SearchParams());

  for (size_t j = 0; j < nearest.cols; ++j)
  {
    int id = nearest.at<int>(j);

    if (id == -1)  // No more close-enough points
      break;

    if (!processed[id])

      // Insert all points close enough to query point into the cluster
      clusterHelper(id, src, srcMat, cluster, processed, tree, radius, knn, minSize, maxSize);
  }
}

// FP.2
void euclideanClustering(const vector<LidarPoint>& src, vector<vector<LidarPoint>>& clusters, 
  vector<vector<LidarPoint>>& removed, float radius, int knn, int minSize, int maxSize)
{
  // Convert src to matrix for nearest neighbor search [2]. Downcast to float for faster processing with minimal 
  // change in output, if any

  cv::Mat srcMat(src.size(), 3, CV_32F);

  for (size_t i = 0; i < src.size(); ++i)
  {
    srcMat.at<float>(i, 0) = src[i].x;
    srcMat.at<float>(i, 1) = src[i].y;
    srcMat.at<float>(i, 2) = src[i].z;
  }

  // Build a KD-Tree structure [2] [3] with arguments optimized for 3D-point search
  cv::flann::GenericIndex<cvflann::L2_Simple<float>> kdtree(srcMat, cvflann::KDTreeSingleIndexParams());

  vector<bool> processed(src.size(), false);

  for (size_t i = 0; i < src.size(); ++i)  // Point at index i is query point
  {
    if (processed[i])  // Skip
      continue;

    vector<LidarPoint> cluster;

    clusterHelper(i, src, srcMat, cluster, processed, kdtree, radius, knn, minSize, maxSize);

    if (cluster.size() >= minSize && cluster.size() <= maxSize)
      clusters.push_back(cluster);
    
    else
      removed.push_back(cluster);
  }
}

// FP.2
void removeOutliers(vector<LidarPoint>& src, vector<LidarPoint>& dst, FilteringMethod method, float radius, int knn,
  int minSize, int maxSize, bool bRenderClusters, bool bShowRemoved, bool bPrintStats)
{
  switch (method)
  {
    case FilteringMethod::TUKEY:  // Tukey's fences [1]
    {  
      // Sort by x, the most important dimension to compute time-to-collision
      sort(src.begin(), src.end(), [](const LidarPoint& p1, const LidarPoint& p2) { return p1.x < p2.x; });

      // Extract x-coordinates for easier percentile calculation
      vector<double> src_x (src.size(), 0.);

      for (size_t i = 0; i < src.size(); ++i)
        src_x[i] = src[i].x;

      double q1 = percentile(src_x, .25);
      double q3 = percentile(src_x, .75);
      double IQR = (q3 - q1);

      double lowerBound = q1 - 1.5 * IQR;
      double upperBound = q3 + 1.5 * IQR;

      vector<LidarPoint> removed;

      for (const auto& point : src)
      {
        // Only keep points inside Tukey's fences
        if (point.x >= lowerBound && point.x <= upperBound)
          dst.push_back(point);

        else
          removed.push_back(point);
      }

      if (bPrintStats)
      {
        printFilteringStats(src, dst, removed, lowerBound, upperBound);

        vector<double> dst_x (dst.size(), 0.);
        for (size_t i = 0; i < dst.size(); ++i)
          dst_x[i] = dst[i].x;

        printDistributionStats(dst_x);
      }
      
      if (bRenderClusters)
      {
        vector<vector<LidarPoint>> kept { dst };
        vector<vector<LidarPoint>> discarded { removed };

        renderClusters(kept, discarded, bShowRemoved);
      }

      break;
    }

    case FilteringMethod::EUCLIDEAN_CLUSTERING:  // [2]
    {
      vector<vector<LidarPoint>> clusters, removed;

      euclideanClustering(src, clusters, removed, radius, knn, minSize, maxSize);

      for (auto& cluster : clusters)
      {
        for (const auto& point : cluster)
          dst.push_back(point);  // Will compute time-to-collision with remaining points
      }

      sort(dst.begin(), dst.end(), [](const LidarPoint& p1, const LidarPoint& p2) { return p1.x < p2.x; });

      if (bPrintStats)
      { 
        printFilteringStats(src, clusters, removed, radius, minSize, maxSize);

        vector<double> dst_x (dst.size(), 0.);
        for (size_t i = 0; i < dst.size(); ++i)
          dst_x[i] = dst[i].x;

        printDistributionStats(dst_x);
      }

      if (bRenderClusters)
        renderClusters(clusters, removed, bShowRemoved);

      break;
    }
  }
}

// FP.2
void computeTTCLidar(vector<LidarPoint>& lidarPointsPrev, vector<LidarPoint>& lidarPointsCurr, double frameRate, 
  double& TTC, FilteringMethod& filteringMethod, float radius, int knn, int minSize, int maxSize, bool bRenderClusters, 
  bool bShowRemoved)
{
  // Time between two measurements, in seconds
  double dT = 1 / frameRate;

  vector<LidarPoint> filteredPrev, filteredCurr;

  // Time the outlier removal process
  auto startTime = chrono::steady_clock::now();

  removeOutliers(lidarPointsPrev, filteredPrev, filteringMethod, radius, knn, minSize, maxSize, bRenderClusters, 
    bShowRemoved, false);
  
  removeOutliers(lidarPointsCurr, filteredCurr, filteringMethod, radius, knn, minSize, maxSize, bRenderClusters, 
    bShowRemoved, true);

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "Outlier detection took: " << elapsedTime.count() / 1000. << " ms" << endl;

  // Time the LiDAR time-to-collision calculation process
  startTime = chrono::steady_clock::now();

  vector<double> filteredXPrev (filteredPrev.size(), 0.);

  for (size_t i = 0; i < filteredPrev.size(); ++i)
    filteredXPrev[i] = filteredPrev[i].x;

  vector<double> filteredXCurr (filteredCurr.size(), 0.);

  for (size_t i = 0; i < filteredCurr.size(); ++i)
    filteredXCurr[i] = filteredCurr[i].x;

  double medianXPrev = percentile(filteredXPrev, 0.5);
  double medianXCurr = percentile(filteredXCurr, 0.5);

  // Compute stable time-to-collision assuming a constant velocity model
  TTC = medianXCurr * dT / (medianXPrev - medianXCurr);

  printTTCStats(medianXPrev, medianXCurr, TTC);

  endTime = chrono::steady_clock::now();
  elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "LiDAR time-to-collision calculation took: " << elapsedTime.count() / 1000. << " ms" << endl;
}

// FP.3
void clusterKptMatchesWithROI(BoundingBox& boundingBox, vector<cv::KeyPoint>& kptsPrev, vector<cv::KeyPoint>& kptsCurr, 
  vector<cv::DMatch>& kptMatches)
{
  // Time bounding-box-to-keypoint-matches association step
  auto startTime = chrono::steady_clock::now();

  vector<double> euclideanDistances;

  vector<cv::KeyPoint> keypointsInRoi, tempKeypoints;
  vector<cv::DMatch> matchesInRoi, tempMatches;

  for (const auto& match : kptMatches)
  {
    cv::KeyPoint prevPoint = kptsPrev.at(match.queryIdx);  // Extract keypoint descriptors of current match
    cv::KeyPoint currPoint = kptsCurr.at(match.trainIdx);

    if (boundingBox.roi.contains(currPoint.pt))  // As we focus on the current bounding box ROI [1]
    {
      double distance = cv::norm(currPoint.pt - prevPoint.pt);  // Defaults to L2-norm (Euclidean distance) [2]
      euclideanDistances.push_back(distance);

      tempKeypoints.push_back(currPoint);  // To avoid code redundancy
      tempMatches.push_back(match);
    }
  }

  sort(euclideanDistances.begin(), euclideanDistances.end());

  double medianDistance = percentile(euclideanDistances, 0.5);  // As mean is not robust enough [3]
  
  int i = 0;
  while (euclideanDistances[i] <= 1.5 * medianDistance && i < euclideanDistances.size())
  {
    // As we expect a rigid transform of preceding vehicle [4]
    keypointsInRoi.push_back(tempKeypoints[i]);
    matchesInRoi.push_back(tempMatches[i]);

    i++;  // Early termination since the vector is sorted in ascending order
  }

  boundingBox.keypoints = keypointsInRoi;
  boundingBox.kptMatches = matchesInRoi;

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "Bounding-box-to-keypoints association took: " << elapsedTime.count() / 1000. << " ms" << endl;
}

// FP.4
void computeTTCCamera(vector<cv::KeyPoint>& kptsPrev, vector<cv::KeyPoint>& kptsCurr, vector<cv::DMatch> kptMatches, 
  double frameRate, double& TTC, cv::Mat* visImg)
{
  // Time the camera time-to-collision calculation process
  auto startTime = chrono::steady_clock::now();

  // This will actually store the heights' ratios, which allow to estimate the distance to the preceding vehicle
  // without measuring distance directly (h1/h0 = d0/d1) (source: Udacity GPT)
  vector<double> distRatios;

  // Compute the heights' ratios
  for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)  // Outer keypoint loop
  {
    cv::KeyPoint kptOuterPrev = kptsPrev.at(it1->queryIdx);
    cv::KeyPoint kptOuterCurr = kptsCurr.at(it1->trainIdx);

    // Filter out outliers or mismatches
    for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)  // Inner keypoint loop
    {
      // To avoid ambiguous matches when keypoints are too close with too similar descriptors
      double minDistance = 100.0;  // [1]

      cv::KeyPoint kptInnerPrev = kptsPrev.at(it2->queryIdx);
      cv::KeyPoint kptInnerCurr = kptsCurr.at(it2->trainIdx);

      double distPrev = cv::norm(kptOuterPrev.pt - kptInnerPrev.pt);
      double distCurr = cv::norm(kptOuterCurr.pt - kptInnerCurr.pt);

      if (distPrev > numeric_limits<double>::epsilon() && distCurr > minDistance)
      {
        // Avoid division by zero
        double distRatio = distCurr / distPrev;  // h1/h0
        distRatios.push_back(distRatio);
      }
    }  // eof inner loop over all matched keypoints
  }  // eof outer loop over all matched keypoints

  if (distRatios.size() == 0)  // Only continue if list of distance ratios is not empty
  {
    cerr << endl << "(!) WARNING: No usable distance ratio to compute TTC. Skipping..." << endl << endl;

    TTC = NAN;
    return;
  }

  // Compute camera-based TTC from distance ratios
  sort(distRatios.begin(), distRatios.end());

  // Apply Tukey's fences to the distance ratios
  vector<double> filteredDistRatios;

  double q1 = percentile(distRatios, .25);
  double q3 = percentile(distRatios, .75);
  double IQR = (q3 - q1);

  double lowerBound = q1 - 1.5 * IQR;
  double upperBound = q3 + 1.5 * IQR;

  for (const auto& ratio : distRatios)
  {
    // Only keep points inside Tukey's fences
    if (ratio >= lowerBound && ratio <= upperBound)
      filteredDistRatios.push_back(ratio);
  }

  // Calculate median after filtering
  double medianDistRatio = percentile(filteredDistRatios, 0.5);  // As robust to outliers

  printFilteringStats(distRatios, filteredDistRatios, medianDistRatio);
  printDistributionStats(filteredDistRatios);

  double dT = 1 / frameRate;
  TTC = -dT / (1 - medianDistRatio);

  printTTCStats(TTC);

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "Camera time-to-collision calculation took: " << elapsedTime.count() / 1000. << " ms" << endl << endl;
}

/*********************************************************************************************************************
 * PRINT FUNCTIONS
 *********************************************************************************************************************/

// FP.2: Print Tukey's fences statistics
void printFilteringStats(const vector<LidarPoint>& src, const vector<LidarPoint>& kept, 
  const vector<LidarPoint>& removed, double lowerBound, double upperBound)
{
  cout << "OUTLIER DETECTION AND FILTERING (LIDAR)" << endl;
  cout << string(50, '-') << endl;

  cout << "Source: " << src.size() << " points" << endl;
  cout << "Filtering method: Tukey's fences" << endl << endl;

  cout << "Lower Bound: " << lowerBound << endl;
  cout << "Upper Bound: " << upperBound << endl << endl;

  cout << left << setw(22) << "Total points kept:" << right << setw(10) << kept.size() << endl;
  cout << left << setw(22) << "Total points removed:" << right << setw(10) << removed.size() << endl;
}

// FP.2: Print Euclidean clustering statistics
void printFilteringStats(const vector<LidarPoint>& src, const vector<vector<LidarPoint>>& clusters, 
  const vector<vector<LidarPoint>>& removed, float radius, int minSize, int maxSize)
{
  cout << "OUTLIER DETECTION AND FILTERING (LIDAR)" << endl;
  cout << string(50, '-') << endl;

  cout << "Source: " << src.size() << " points" << endl;
  cout << "Filtering method: Euclidean clustering" << endl << endl;

  cout << "Radius: " << radius << " (sq.: " << radius*radius << ")" << endl << endl;

  cout << "Min cluster size: " << right << setw(4) << minSize << endl;
  cout << "Max cluster size: " << right << setw(4) << maxSize << endl << endl;

  int pointsKept = 0, pointsRemoved = 0;

  for (auto& cluster : clusters)
  {
    cout << left << setw(25) << "Found cluster of size: " << right << setw(4) << cluster.size() << endl;
    pointsKept += cluster.size();
  }

  cout << string(30, '-') << endl;
  cout << left << setw(25) << "Total points kept: " << right << setw(4) << pointsKept << endl;
  cout << endl;

  for (auto& cluster : removed)
  {
    cout << left << setw(25) << "Removed cluster of size: " << right << setw(4) << cluster.size() << endl;
    pointsRemoved += cluster.size();
  }

  cout << string(30, '-') << endl;
  cout << left << setw(25) << "Total points removed: " << right << setw(4) << pointsRemoved << endl;
  cout << endl;
}

// FP.6: Print camera-based filtering statistics
void printFilteringStats(const vector<double> distRatios, const vector<double> filteredDistRatios, 
  double medianDistRatio)
{
  cout << endl;
  cout << "OUTLIER DETECTION AND FILTERING (CAMERA)" << endl;
  cout << string(50, '-') << endl;

  vector<string> header = {"Candidate distance ratios:", "Available ratios post filtering:", "Median distance ratio:"};

  if (filteredDistRatios.empty())
    cerr << endl << "(!) WARNING: No distance ratio matches the filtering criteria. Skipping..." << endl << endl;
  else
  {
    cout << left << setw(35) << header[0] << right << setw(15) << distRatios.size() << endl;
    cout << left << setw(35) << header[1] << right << setw(15) << filteredDistRatios.size() << endl;
    cout << left << setw(35) << header[2] << right << setw(15) << medianDistRatio << endl << endl;
  }

  if (medianDistRatio == 1)
    cerr << "(!) WARNING: Median distance ratio equal to 1. TTC estimate will be -inf." << endl << endl;
}

// FP.5, FP.6
void printDistributionStats(const vector<double>& vec)
{
  double csi = skewness(vec);
  double kappa = excess_kurtosis(vec);

  cout << endl;
  cout << "Distribution statistics (kept)" << endl << string(32, '-') << endl;

  cout << left << setw(22) << "Mean:" << right << setw(10) << mean(vec) << endl;
  cout << left << setw(22) << "Standard deviation:" << right << setw(10) << sqrt(variance(vec)) << endl;
  cout << left << setw(22) << "Skewness:" << right << setw(10) << csi << endl;
  cout << left << setw(22) << "Excess kurtosis:" << right << setw(10) << kappa << endl << endl;

  string description = (csi < 0) ? "Negatively-skewed" : "Positively-skewed";
  description += ", ";
  description += (kappa < 0) ? "platykurtic" : "leptokurtic";

  cout << "Distribution shape: " << description << endl << endl;
}

// FP.2: Print LiDAR time-to-collision statistics
void printTTCStats(const double medianXPrev, const double medianXCurr, const double TTC)
{
  cout << endl << "LIDAR TIME-TO-COLLISION STATISTICS" << endl;
  cout << string(50, '-') << endl;

  cout << left << setw(35) << "Previous median (x-coordinate):" << right << setw(15) << medianXPrev << endl;
  cout << left << setw(35) << "Current median (x-coordinate):" << right << setw(15) << medianXCurr << endl;
  cout << left << setw(35) << "Difference:" << right << setw(15) << (medianXPrev - medianXCurr) << endl;
  cout << left << setw(35) << "TTC:" << right << setw(15) << TTC << endl << endl;
}

// FP.6: Print camera time-to-collision statistics
void printTTCStats(const double TTC)
{
  cout << endl << "CAMERA TIME-TO-COLLISION STATISTICS" << endl;
  cout << string(50, '-') << endl;

  cout << left << setw(35) << "TTC:" << right << setw(15) << TTC << endl << endl;
}

// FP.5, FP.6: Print time-to-collision summary statistics
void printSummaryStats(const int imgStartIndex, const vector<vector<double>>& ttcStats)
{
  cout << string(40, '*') << endl;
  cout << setw(10) << "IMAGE PAIR" << right << setw(15) << "LIDAR TTC" << right << setw(15) << "CAMERA TTC" << endl;
  cout << string(40, '*') << endl;

  int index = (imgStartIndex);
  for (size_t i = 0; i < ttcStats.size(); ++i)
  {
    cout << right << setw(10) << to_string(index) + "-" + to_string(++index) << right << setw(15) << ttcStats[i][0] 
      << right << setw(15) << ttcStats[i][1] << endl;
  }
}