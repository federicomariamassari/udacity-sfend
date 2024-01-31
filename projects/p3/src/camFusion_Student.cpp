#include "camFusion.hpp"
#include "lidarData.hpp"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(vector<BoundingBox> &boundingBoxes, vector<LidarPoint> &lidarPoints, float shrinkFactor, 
  cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
  // loop over all Lidar points and associate them to a 2D bounding box
  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
  {
    // assemble vector for matrix-vector-multiplication
    X.at<double>(0, 0) = it1->x;
    X.at<double>(1, 0) = it1->y;
    X.at<double>(2, 0) = it1->z;
    X.at<double>(3, 0) = 1;

    // project Lidar point into camera
    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;
    // pixel coordinates
    pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
    pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

    vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
    for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
    {
      // shrink current bounding box slightly to avoid having too many outlier points around the edges
      cv::Rect smallerBox;
      smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
      smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
      smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
      smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

      // check wether point is within current bounding box
      if (smallerBox.contains(pt))
      {
        enclosingBoxes.push_back(it2);
      }

    } // eof loop over all bounding boxes

    // check wether point has been enclosed by one or by multiple boxes
    if (enclosingBoxes.size() == 1)
    { 
      // add Lidar point to bounding box
      enclosingBoxes[0]->lidarPoints.push_back(*it1);
    }

  } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

  for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
  {
    // create randomized color for current 3D object
    cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

    // plot Lidar points into top view image
    int top=1e8, left=1e8, bottom=0.0, right=0.0; 
    float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
    for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
    {
      // world coordinates
      float xw = (*it2).x; // world position in m with x facing forward from sensor
      float yw = (*it2).y; // world position in m with y facing left from sensor
      xwmin = (xwmin < xw) ? xwmin : xw;
      ywmin = (ywmin < yw) ? ywmin : yw;
      ywmax = (ywmax > yw) ? ywmax : yw;

      // top-view coordinates
      int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
      int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

      // find enclosing rectangle
      top = (top < y) ? top : y;
      left = (left < x) ? left : x;
      bottom = (bottom > y) ? bottom : y;
      right = (right > x) ? right : x;

      // draw individual point
      cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
    }

    // draw enclosing rectangle
    cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

    // augment object with some key data
    char str1[200], str2[200];
    sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
    putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
    sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
    putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
  }

  // plot distance markers
  float lineSpacing = 2.0; // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i)
  {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
  }

  // display image
  string windowName = "3D Objects";
  cv::namedWindow(windowName, 1);
  cv::imshow(windowName, topviewImg);

  if(bWait)
  {
    cv::waitKey(0); // wait for key to be pressed
  }
}

void matchBoundingBoxes(vector<cv::DMatch> &matches, map<int, int> &bbBestMatches, DataFrame &prevFrame, 
  DataFrame &currFrame)
{
  // Time the bounding box matching process
  auto startTime = chrono::steady_clock::now();

  // https://knowledge.udacity.com/questions/570553 (additional refactoring suggested by Udacity GPT)
  map<pair<int, int>, int> idPairsCount;

  for (const auto& match : matches)
  {
    // https://docs.opencv.org/4.2.0/d4/de0/classcv_1_1DMatch.html
    cv::KeyPoint prevPoint = prevFrame.keypoints[match.queryIdx];
    cv::KeyPoint currPoint = currFrame.keypoints[match.trainIdx];

    for (const BoundingBox& prevBB : prevFrame.boundingBoxes)
    {
      if (prevBB.roi.contains(prevPoint.pt))  // Keypoint is in the previous bounding box
      {
        for (const BoundingBox& currBB : currFrame.boundingBoxes)
        {
          if (currBB.roi.contains(currPoint.pt))  // Keypoint is also in the current bounding box
          {
            idPairsCount[make_pair(prevBB.boxID, currBB.boxID)]++;  // We have a correspondence
            break;  // No need to continue searching in the current bounding box
          }
        }

        break;  // No need to continue searching in the remaining bounding boxes
      }
    }
  }

  // Match bounding box pairs by highest keypoints' frequency
  map<int, int> highestCounts;

  for (const auto& item : idPairsCount)
  {
    int id = item.first.first;  // Previous bounding box id

    if (item.second > highestCounts[id])
    {
      highestCounts[id] = item.second;
      bbBestMatches[id] = item.first.second;  // Map previous-to-current box id best match
    }
  }

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "Bounding Boxes matching took: " << elapsedTime.count() / 1000. << " milliseconds." << endl;
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

double std_dev(const vector<double>& vec)
{
  double mu = mean(vec);

  double squared_dev = accumulate(vec.begin(), vec.end(), 0., 
    [mu](double sum, const double x) { return sum + pow(x - mu, 2); });

  return sqrt(squared_dev / (vec.size() - 1));  // Sample standard deviation
}

void renderCluster(const vector<LidarPoint> &src, const vector<set<int>> &clusters)
{
  cv::viz::Viz3d window("Filtered LiDAR point cloud");
  window.setBackgroundColor(cv::viz::Color::black());

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    std::vector<cv::Point3f> points;

    for (int id : clusters[i])
      points.push_back(cv::Point3f(src[id].x, src[id].y, src[id].z));

    cv::viz::WCloud cloud(points);
    cloud.setRenderingProperty(cv::viz::POINT_SIZE, 5.0);

    // Rotate color among clusters (to extend: i % n == k, with k = 0, ..., n-1)
    cv::viz::Color color;

    if (i % 3 == 0) {
        color = cv::viz::Color::red();  // [1]
    } else if (i % 3 == 1) {
        color = cv::viz::Color::blue();
    } else {
        color = cv::viz::Color::yellow();
    }

    cloud.setColor(color);

    std::string cloudName = "cloud" + std::to_string(i);
    window.showWidget(cloudName, cloud);
  }

  window.spin();  // Trigger event loop
}

template<typename T>
void clusterHelper(int index, const cv::Mat& cloud, set<int>& cluster, vector<bool>& processed,
  cv::flann::GenericIndex<T>& tree, float radius, int minSize, int maxSize)
{
  processed[index] = true;
  cluster.insert(index);

  int knn = 3;

  // Will contain (sorted) ids and distances of points close enough to query point, -1 elsewhere
  cv::Mat nearest (1, knn, CV_32S, cv::Scalar::all(-1));
  cv::Mat distances (1, knn, CV_32F, cv::Scalar::all(-1));

  // Square radius as we are using L2-norm [4]; for cvflann::SearchParams see [5]
  tree.radiusSearch(cloud.row(index), nearest, distances, radius*radius, cvflann::SearchParams());

  for (size_t j = 0; j < nearest.cols; ++j)
  {
    int id = nearest.at<int>(j);

    if (id == -1)  // No more close-enough points
      break;

    if (!processed[id])

      // Insert all points close enough to query point into the cluster
      clusterHelper(id, cloud, cluster, processed, tree, radius, minSize, maxSize);
  }
}

void euclideanClustering(const vector<LidarPoint> &src, vector<set<int>> &clusters, float radius, int minSize, 
  int maxSize)
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

    clusterHelper(i, srcMat, cluster, processed, kdtree, radius, minSize, maxSize);

    if (cluster.size() >= minSize && cluster.size() <= maxSize)
      clusters.push_back(cluster);
  }
}

void removeOutliers(vector<LidarPoint> &src, vector<LidarPoint> &dst, FilteringMethod method)
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
      float radius = 0.6;
      int minSize = 30, maxSize = 400;

      vector<set<int>> clusters;

      euclideanClustering(src, clusters, radius, minSize, maxSize);

      for (auto& cluster : clusters)
      {
        cout << "Source: " << src.size() << "\tCluster size: " << cluster.size() << endl;

        for (int id : cluster)
        {
          dst.push_back(src[id]);  // Will compute time-to-collision with remaining points
        }
      }

      sort(dst.begin(), dst.end(), [](const LidarPoint& p1, const LidarPoint& p2) { return p1.x < p2.x; });

      renderCluster(src, clusters);

      break;
    }
  }
}

void computeTTCLidar(vector<LidarPoint> &lidarPointsPrev, vector<LidarPoint> &lidarPointsCurr, double frameRate, 
  double &TTC)
{
  // Time the LiDAR time-to-collision calculation process
  auto startTime = chrono::steady_clock::now();

  // Time between two measurements, in seconds
  double dT = 1 / frameRate;

  vector<LidarPoint> filteredPrev, filteredCurr;

  removeOutliers(lidarPointsPrev, filteredPrev, FilteringMethod::EUCLIDEAN_CLUSTERING);
  removeOutliers(lidarPointsCurr, filteredCurr, FilteringMethod::EUCLIDEAN_CLUSTERING);  

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
  cout << "LiDAR time-to-collision calculation took: " << elapsedTime.count() / 1000. << " milliseconds." << endl;
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, vector<cv::KeyPoint> &kptsPrev, vector<cv::KeyPoint> &kptsCurr, 
  vector<cv::DMatch> &kptMatches)
{
  // ...
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(vector<cv::KeyPoint> &kptsPrev, vector<cv::KeyPoint> &kptsCurr, vector<cv::DMatch> kptMatches, 
  double frameRate, double &TTC, cv::Mat *visImg)
{
  // ...
}
