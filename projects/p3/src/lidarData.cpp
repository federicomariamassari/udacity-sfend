#include "lidarData.hpp"


using namespace std;

void cropLidarPoints(vector<LidarPoint>& lidarPoints, float minX, float maxX, float maxY, float minZ, float maxZ, 
  float minR)
{
  // Time LiDAR points cropping (custom addition)
  auto startTime = chrono::steady_clock::now();

  vector<LidarPoint> newLidarPts; 
  
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
  {  
    // Remove points which are positioned behind the LiDAR sensor (negative x-coordinate), too far ahead, outside 
    // the ego lane in the y-direction, too close to the road surface (negative z), and unreliable (low reflectivity)

    if((*it).x >= minX && (*it).x <= maxX && (*it).z >= minZ && (*it).z <= maxZ && (*it).z <= 0.0 && 
      abs((*it).y) <= maxY && (*it).r >= minR)
    {
       newLidarPts.push_back(*it);
    }
  }

  lidarPoints = newLidarPts;

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "LiDAR point cloud cropping took: " << elapsedTime.count() / 1000. << " ms" << endl;
}

void loadLidarFromFile(vector<LidarPoint> &lidarPoints, string filename)
{
  // Allocate 4 MB buffer (only ~130*4*4 KB are needed)
  unsigned long num = 1000000;
  float *data = (float*) malloc(num * sizeof(float));
  
  // Pointers
  float *px = data + 0;
  float *py = data + 1;
  float *pz = data + 2;
  float *pr = data + 3;
  
  // Load point cloud
  FILE *stream;
  stream = fopen(filename.c_str(), "rb");
  num = fread(data, sizeof(float), num, stream) / 4;

  for (int32_t i = 0; i < num; i++)
  {
    LidarPoint lpt;
    lpt.x = *px; lpt.y = *py; lpt.z = *pz; lpt.r = *pr;
    lidarPoints.push_back(lpt);
    px += 4; py += 4; pz += 4; pr += 4;
  }

  fclose(stream);
}

void showLidarTopview(vector<LidarPoint>& lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
  // Create top-view image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

  // Plot LiDAR points into image
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
  {
    float xw = (*it).x;  // World position in m with x facing forward from sensor
    float yw = (*it).y;  // World position in m with y facing left from sensor

    int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
    int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

    cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
  }

  // Plot distance markers
  float lineSpacing = 2.0; // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i)
  {
      int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
      cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
  }

  // Display image
  string windowName = "Top-View Perspective of LiDAR data";
  cv::namedWindow(windowName, 2);
  cv::imshow(windowName, topviewImg);

  if (bWait)
    cv::waitKey(0);  // Wait for key to be pressed
}

void showLidarImgOverlay(cv::Mat& img, vector<LidarPoint>& lidarPoints, cv::Mat& P_rect_xx, cv::Mat& R_rect_xx, 
  cv::Mat& RT, cv::Mat* extVisImg)
{
  // Init image for visualization
  cv::Mat visImg;

  if (extVisImg == nullptr)
    visImg = img.clone();

  else 
    visImg = *extVisImg;


  cv::Mat overlay = visImg.clone();

  // Find maximum x-value
  double maxVal = 0.0; 
  
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    maxVal = (maxVal < it->x) ? it->x : maxVal;

  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) 
  {
    X.at<double>(0, 0) = it->x;
    X.at<double>(1, 0) = it->y;
    X.at<double>(2, 0) = it->z;
    X.at<double>(3, 0) = 1;

    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;

    pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
    pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

    float val = it->x;
    int red = min(255, (int) (255 * abs((val - maxVal) / maxVal)));
    int green = min(255, (int) (255 * (1 - abs((val - maxVal) / maxVal))));
    
    cv::circle(overlay, pt, 5, cv::Scalar(0, green, red), -1);
  }

  float opacity = 0.6;
  cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

  // Return augmented image or wait if no image has been provided
  if (extVisImg == nullptr)
  {
    string windowName = "LiDAR data on image overlay";
    cv::namedWindow(windowName, 3);
    cv::imshow(windowName, visImg);
    cv::waitKey(0);  // Wait for key to be pressed
  }

  else
    extVisImg = &visImg;
}
