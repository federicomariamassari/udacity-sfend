#include "camFusion.hpp"
#include "lidarData.hpp"

using namespace std;


// Create groups of LiDAR points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(vector<BoundingBox>& boundingBoxes, vector<LidarPoint>& lidarPoints, float shrinkFactor, 
  cv::Mat& P_rect_xx, cv::Mat& R_rect_xx, cv::Mat& RT)
{
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
}

/** 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually 
* tuned to fit the 2000x2000 size. However, you can make this function work for other sizes too. For instance, to use 
* a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(vector<BoundingBox>& boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
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

  if (bWait)
    cv::waitKey(0);  // Wait for key to be pressed
}

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

double mean(const vector<double>& vec)
{
  return accumulate(vec.begin(), vec.end(), 0.) / vec.size();
}

double std_dev(const vector<double>& vec)  // TODO: Potentially unused
{
  double mu = mean(vec);

  double squared_dev = accumulate(vec.begin(), vec.end(), 0., 
    [mu](double sum, const double x) { return sum + pow(x - mu, 2); });

  return sqrt(squared_dev / (vec.size() - 1));  // Sample standard deviation
}

void renderClusters(const vector<LidarPoint>& src, const vector<set<int>>& clusters, bool bShowRemoved)
{
  cv::viz::Viz3d window("Filtered LiDAR point cloud");
  window.setBackgroundColor(cv::viz::Color::black());

  set<int> removedIds;

  if (bShowRemoved)
  {
    for (int i = 0; i < src.size(); ++i)
      removedIds.insert(i);
  }

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    std::vector<cv::Point3f> points;

    for (int id : clusters[i])
    {  
      points.push_back(cv::Point3f(src[id].x, src[id].y, src[id].z));

      if (bShowRemoved)
        removedIds.erase(id);  // Pop ids of kept point to only preserve removed ones
    }

    cv::viz::WCloud cloud(points);
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6.0);

    // Rotate color among clusters (to extend: i % n == k, for all k = 0, ..., n-1)
    cv::viz::Color color;

    if (i % 3 == 0) {
        color = cv::viz::Color::red();  // [1]
    } else if (i % 3 == 1) {
        color = cv::viz::Color::blue();
    } else {
        color = cv::viz::Color::green();
    }

    cloud.setColor(color);

    string cloudName = "cloud" + to_string(i);
    window.showWidget(cloudName, cloud);
  }

  if (bShowRemoved)  // Removed clusters will be colorless
  {
    vector<cv::Point3f> removed;

    for (int id : removedIds)
      removed.push_back(cv::Point3f(src[id].x, src[id].y, src[id].z));

    cv::viz::WCloud cloud(removed);
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 6.0);
    cloud.setColor(cv::viz::Color::white());
    
    window.showWidget("removed", cloud);

  }

  window.spin();  // Trigger event loop
}

void printStatistics(const vector<LidarPoint>& src, const vector<set<int>>& clusters, const vector<set<int>>& removed,
  float radius, int minSize, int maxSize)
{
  cout << endl;
  cout << "Source: " << src.size() << " points" << endl;
  cout << endl;

  cout << "Filtering method: Euclidean clustering" << endl;
  cout << "Radius: " << radius << " (sq.: " << radius*radius << ")" << endl;
  cout << endl;

  cout << "Min cluster size: " << right << setw(4) << minSize << endl;
  cout << "Max cluster size: " << right << setw(4) << maxSize << endl;
  cout << endl;

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

template<typename T>
void clusterHelper(int index, const cv::Mat& cloud, set<int>& cluster, vector<bool>& processed,
  cv::flann::GenericIndex<T>& tree, float radius, int knn, int minSize, int maxSize)
{
  processed[index] = true;
  cluster.insert(index);

  // Will contain (sorted) ids and distances of points close enough to query point, -1 elsewhere
  cv::Mat nearest (1, knn, cv::DataType<int>::type, cv::Scalar::all(-1));
  cv::Mat distances (1, knn, cv::DataType<float>::type, cv::Scalar::all(-1));

  // Square radius as we are using L2-norm [4]; for cvflann::SearchParams see [5]
  tree.radiusSearch(cloud.row(index), nearest, distances, radius*radius, cvflann::SearchParams());

  for (size_t j = 0; j < nearest.cols; ++j)
  {
    int id = nearest.at<int>(j);

    if (id == -1)  // No more close-enough points
      break;

    if (!processed[id])

      // Insert all points close enough to query point into the cluster
      clusterHelper(id, cloud, cluster, processed, tree, radius, knn, minSize, maxSize);
  }
}

void euclideanClustering(const vector<LidarPoint>& src, vector<set<int>>& clusters, vector<set<int>>& removed,
  float radius, int knn, int minSize, int maxSize)
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

    set<int> cluster;

    clusterHelper(i, srcMat, cluster, processed, kdtree, radius, knn, minSize, maxSize);

    if (cluster.size() >= minSize && cluster.size() <= maxSize)
      clusters.push_back(cluster);
    
    else
      removed.push_back(cluster);
  }
}

void removeOutliers(vector<LidarPoint>& src, vector<LidarPoint>& dst, FilteringMethod method, float radius, int knn,
  int minSize, int maxSize, bool bRenderClusters, bool bShowRemoved)
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

      for (const auto& point : src)
      {
        // Only keep points inside Tukey's fences
        if (point.x >= lowerBound && point.x <= upperBound)
          dst.push_back(point);
      }

      break;
    }

    case FilteringMethod::EUCLIDEAN_CLUSTERING:  // [2]
    {
      vector<set<int>> clusters, removed;

      euclideanClustering(src, clusters, removed, radius, knn, minSize, maxSize);

      for (auto& cluster : clusters)
      {
        for (int id : cluster)
          dst.push_back(src[id]);  // Will compute time-to-collision with remaining points
      }

      sort(dst.begin(), dst.end(), [](const LidarPoint& p1, const LidarPoint& p2) { return p1.x < p2.x; });

      if (bRenderClusters)
        renderClusters(src, clusters, bShowRemoved);
      
      printStatistics(src, clusters, removed, radius, minSize, maxSize);

      break;
    }
  }
}

void computeTTCLidar(vector<LidarPoint>& lidarPointsPrev, vector<LidarPoint>& lidarPointsCurr, double frameRate, 
  double& TTC, FilteringMethod& filteringMethod, float radius, int knn, int minSize, int maxSize, bool bRenderClusters, 
  bool bShowRemoved)
{
  // Time the LiDAR time-to-collision calculation process
  auto startTime = chrono::steady_clock::now();

  // Time between two measurements, in seconds
  double dT = 1 / frameRate;

  vector<LidarPoint> filteredPrev, filteredCurr;

  // TODO: Time outlier removal process as well!
  removeOutliers(lidarPointsPrev, filteredPrev, filteringMethod, radius, knn, minSize, maxSize, bRenderClusters, 
    bShowRemoved);
  
  removeOutliers(lidarPointsCurr, filteredCurr, filteringMethod, radius, knn, minSize, maxSize, bRenderClusters,
    bShowRemoved);

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

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "LiDAR time-to-collision calculation took: " << elapsedTime.count() / 1000. << " ms" << endl;
}

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

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(vector<cv::KeyPoint>& kptsPrev, vector<cv::KeyPoint>& kptsCurr, vector<cv::DMatch> kptMatches, 
  double frameRate, double& TTC, cv::Mat* visImg)
{
  // Time the camera time-to-collision calculation process
  auto startTime = chrono::steady_clock::now();

  vector<double> distRatios;

  for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)  // Outer keypoint loop
  {
    cv::KeyPoint kptOuterPrev = kptsPrev.at(it1->queryIdx);
    cv::KeyPoint kptOuterCurr = kptsCurr.at(it1->trainIdx);

    for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)  // Inner keypoint loop
    {
      double minDistance = 100.0;

      cv::KeyPoint kptInnerPrev = kptsPrev.at(it2->queryIdx);
      cv::KeyPoint kptInnerCurr = kptsCurr.at(it2->trainIdx);

      double distPrev = cv::norm(kptOuterPrev.pt - kptInnerPrev.pt);
      double distCurr = cv::norm(kptOuterCurr.pt - kptInnerCurr.pt);

      if (distPrev > numeric_limits<double>::epsilon() && distCurr > minDistance)
      {
        // Avoid division by zero
        double distRatio = distCurr / distPrev;
        distRatios.push_back(distRatio);
      }
    }  // eof inner loop over all matched keypoints
  }  // eof outer loop over all matched keypoints

  if (distRatios.size() == 0)  // Only continue if list of distance ratios is not empty
  {
    TTC = NAN;
    return;
  }

  // Compute camera-based TTC from distance ratios
  sort(distRatios.begin(), distRatios.end());

  double medianDistRatio = percentile(distRatios, 0.5);
  
  double dT = 1 / frameRate;
  TTC = -dT / (1 - medianDistRatio);

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "Camera time-to-collision calculation took: " << elapsedTime.count() / 1000. << " ms" << endl;

  // EXPERIMENTAL: Visualization (steps suggested by Udacity GPT) && project 2

  //cv::Mat kptsImg = visImg->clone(); //cv::Mat::zeros(visImg->rows, visImg->cols, CV_8UC3);

  //if (visImg != nullptr)
  //{
    // https://docs.opencv.org/4.2.0/d4/d5d/group__features2d__draw.html (suggested by Udacity GPT)

    // Draw previous, current keypoints and their distance ratios (or distances?)
    //cv::drawKeypoints(*visImg, kptsPrev, *visImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    //cv::drawKeypoints(*visImg, kptsCurr, *visImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    //string windowName = "Keypoints overlay";
    //cv::namedWindow(windowName, 6);
    //imshow(windowName, *visImg);
  //}

  // END EXPERIMENTAL


  // TODO: How to use *visImg
}