/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL for exploring self-driving car sensors

#include <unordered_set>
#include <vector>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

// Using templates for processPointClouds, so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  Car egoCar (Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
  Car car1 (Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
  Car car2 (Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2"); 
  Car car3 (Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene)
  {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // -----------------------------------------------------
  // ----- Open 3D viewer and display simple highway -----
  // -----------------------------------------------------
  
  // RENDER OPTIONS
  bool renderScene = false;  // true to display highway and cars
  bool renderLidarScans = false;  // true to render LiDAR scans
  bool renderDataPoints = false;  // true to render point cloud data points (non-colored)
  bool filterPointCloud = false;  // true to downsample point cloud using a Voxel Grid filter
  bool renderObstacleCloud = true;  // true to render obstacles in red (non-inliers)
  bool renderPlaneCloud = true;  // true to render inliers in green
  bool renderKdTree = false;  // true to render 3D KD-Tree in the Viewer (custom addition)
  bool renderClusters = true;  // true to render Euclidean Clustering on obstacle data
  bool renderBoxes = true;  // true to render bounding boxes around obstacle data

  std::vector<Car> cars = initHighway(renderScene, viewer);
  
  Lidar* lidar = new Lidar(cars, 0);
  
  // Generate scan from LiDAR object
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();

  // Visualize the generated LiDAR scans
  if (renderLidarScans)
    renderRays(viewer, lidar->position, pointCloud);

  if (renderDataPoints)
    renderPointCloud(viewer, pointCloud, "pointCloud");

  // Instantiating pointProcessor on the heap instead of the stack, so using pointers
  ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>;

  if (filterPointCloud)
  {
    float leafSize = 1.0f;  // Each voxel is a cube with side of 1m
    
    // Values suggested by Udacity GPT
    // Only keep points within range x: [-10; 10], y: [-5; 5], z: [-2; 2]
    Eigen::Vector4f minPoint (-10.0f, -5.0f, -2.0f, 1.0f);
    Eigen::Vector4f maxPoint (10.0f, 5.0f, 2.0f, 1.0f);

    // Create temporary variable to prevent segmentation fault
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud = pointProcessor->FilterCloud(pointCloud, leafSize, minPoint, maxPoint);
    pointCloud = filteredPointCloud;
  }

  // RANSAC 3D
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> 
    segmentCloud = pointProcessor->SegmentPlane(pointCloud, 100, 0.2);

  if (renderObstacleCloud)
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0));

  if (renderPlaneCloud)
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

  // KD-Tree 3D
  KdTree<pcl::PointXYZ>* tree = pointProcessor->CreateKdTree(segmentCloud.first);

  if (renderKdTree)
  {
    // Time the KD-Tree rendering process
    auto startTime = std::chrono::steady_clock::now();

    // Initialize the boundaries of the box enclosing 3D KD-Tree
    Box window = initKdTreeBox<pcl::PointXYZ>(segmentCloud.first);

    int it = 0;
    render3DTree(tree->root, viewer, window, it);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "KD-Tree rendering took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;
  }

  // Euclidean Clustering 3D
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> 
    cloudClusters = pointProcessor->Clustering(segmentCloud.first, tree, 2.0, 3, 30);

  // Reuse colors every three clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) 
  {
    if (renderClusters)
    {
      std::cout << "Cluster size: ";
      pointProcessor->numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colors[clusterId]);
    }

    if (renderBoxes)
    {
      // Add Bounding Boxes around the clusters
      Box box = pointProcessor->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
    }

      ++clusterId;
  }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  viewer->setBackgroundColor (0,0,0);

  // Set camera position and angle
  viewer->initCameraParameters();

  // Distance away in meters
  int distance = 16;

  switch(setAngle)
  {
    case XY: 
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); 
      break;
    
    case TopDown: 
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1); 
      break;
    
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); 
      break;
    
    case FPS: 
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if(setAngle != FPS)
    viewer->addCoordinateSystem (1.0);
}

int main (int argc, char** argv)
{
  std::cout << "Starting enviroment..." << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  simpleHighway(viewer);

  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
  } 
}