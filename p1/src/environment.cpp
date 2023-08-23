/* \author Aaron Brown
 * Create simple 3D highway enviroment using PCL for exploring self-driving car sensors.
 */
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

// Add custom options and parameters to run the project
#include "custom/options.h"

// Using templates for processPointClouds, so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  Car egoCar (Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1 (Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2 (Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2"); 
  Car car3 (Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

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

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  viewer->setBackgroundColor (0, 0, 0);

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

  if (setAngle != FPS)
    viewer->addCoordinateSystem (1.0);
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, const Options& options)
{
  // -----------------------------------------------------
  // ----- Open 3D viewer and display simple highway -----
  // -----------------------------------------------------

  std::vector<Car> cars = initHighway(options.renderScene, viewer);

  Lidar* lidar = new Lidar(cars, 0);

  // Generate scan from LiDAR object
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();

  // Visualize the generated LiDAR scans
  if (options.renderLidarScans)
    renderRays(viewer, lidar->position, pointCloud);

  if (options.renderDataPoints)
    renderPointCloud(viewer, pointCloud, "pointCloud");

  // Using pointers since pointProcessor is instantiated on the heap, not stack
  ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>;

  // RANSAC 3D
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> 
    segmentCloud = pointProcessor->SegmentPlane(pointCloud, options.maxIterations, options.distanceThreshold);

  if (options.renderPlaneCloud)
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

  if (options.renderObstacleCloud)
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0));

  // KD-Tree 3D
  KdTree<pcl::PointXYZ>* tree = pointProcessor->CreateKdTree(segmentCloud.first);

  if (options.renderKdTree)
  {
    // Time the KD-Tree rendering process
    auto startTime = std::chrono::steady_clock::now();

    // Initialize the boundaries of the box enclosing 3D KD-Tree
    Box window = initKdTreeBox<pcl::PointXYZ>(segmentCloud.first);

    int it = 0;
    render3DTree<pcl::PointXYZ>(tree->root, viewer, window, it);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "KD-Tree rendering took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;
  }

  // Euclidean Clustering 3D
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 
    tree, options.clusterTolerance, options.minSize, options.maxSize);

  // Reuse colors every three clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) 
  {
    if (options.renderClusters)
    {
      std::cout << "Cluster size: ";
      pointProcessor->numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
    }

    if (options.renderBoxes)
    {
      // Add Bounding Boxes around the clusters
      Box box = pointProcessor->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
    }

    if (options.renderMinimumXyAlignedBoxes)
    {
      // Experimental: Render minimum, XY-plane-aligned PCA bounding boxes around the clusters
      BoxQ boxQ = pointProcessor->MinimumXyAlignedBoundingBoxQ(cluster);
      renderBox(viewer, boxQ, clusterId);
    }

    ++clusterId;
  }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, 
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud, const Options& options)
{
  // -----------------------------------------------------
  // ------- Open 3D viewer and display city block -------
  // -----------------------------------------------------

  if (options.filterPointCloud)
  {
    // VOXEL GRID & REGION OF INTEREST
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredInputCloud = pointProcessor->FilterCloud(inputCloud, 
      options.filterRes, options.minPoint, options.maxPoint);

    inputCloud = filteredInputCloud;
  }

  if (options.renderDataPoints)
    renderPointCloud(viewer, inputCloud, "inputCloud");

  // RANSAC 3D
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> 
    segmentCloud = pointProcessor->SegmentPlane(inputCloud, options.maxIterations, options.distanceThreshold);

  if (options.renderPlaneCloud)
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

  if (options.renderObstacleCloud)
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0));

  // KD-TREE 3D
  KdTree<pcl::PointXYZI>* tree = pointProcessor->CreateKdTree(segmentCloud.first);

  if (options.renderKdTree)
  {
    // Time the KD-Tree rendering process
    auto startTime = std::chrono::steady_clock::now();

    // Initialize the boundaries of the box enclosing 3D KD-Tree
    Box window = initKdTreeBox<pcl::PointXYZI>(segmentCloud.first);

    int it = 0;
    render3DTree<pcl::PointXYZI>(tree->root, viewer, window, it);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "KD-Tree rendering took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;
  }

  // EUCLIDEAN CLUSTERING
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 
    tree, options.clusterTolerance, options.minSize, options.maxSize);

  // Reuse colors every three clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

  if (options.renderEgoCarBox)
  {
    // Render location of egoCar as a static purple box
    pcl::PointCloud<pcl::PointXYZI>::Ptr egoCarLocation (new pcl::PointCloud<pcl::PointXYZI>);

    // Detecting egoCar cluster and fitting a new box at each frame is computationally expensive and
    // leads to flickering behaviour, so keeping box size constant (y-limits slightly lower than default)
    pcl::PointXYZI egoCarMinPoint (-1.5, -1.6, -1.0, 1.0);
    pcl::PointXYZI egoCarMaxPoint (2.6, 1.6, 0.4, 1.0);

    egoCarLocation->points.push_back(egoCarMinPoint);
    egoCarLocation->points.push_back(egoCarMaxPoint);

    Box egoCarBox = pointProcessor->BoundingBox(egoCarLocation);
    renderBox(viewer, egoCarBox, 100, Color(1,0,1));
  }

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    std::cout << "Cluster size: ";
    pointProcessor->numPoints(cluster);

    if (options.renderClusters)
      renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

    if (options.renderBoxes)
    {
      // Add Bounding Boxes around the clusters
      Box box = pointProcessor->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
    }

    if (options.renderMinimumXyAlignedBoxes)
    {
      // Experimental: Render minimum, XY-plane-aligned PCA bounding boxes around the clusters
      BoxQ boxQ = pointProcessor->MinimumXyAlignedBoundingBoxQ(cluster);
      renderBox(viewer, boxQ, clusterId);
    }

    ++clusterId;
  }
}

/* Render all scenarios in "LiDAR Obstacle Detection" project by changing main options.
 * Fine-tune rendering options and parameters are in "custom/options.h".
 *
 * Main options:
 * - renderCityBlock: true to render "City Block", false to render "Simple Highway"
 * - streamCityBlock: true to stream multiple PCD files (data_1), false to display a single frame
 * - trackCyclist: true for highly non-linear tracking of a bicyclist and the surrounding objects (data_2)
 */
int main (int argc, char** argv)
{
  std::cout << "Starting enviroment..." << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  // ----------------------------------------------------------------------------------------------
  // MAIN OPTIONS TO RENDER ALL SCENARIOS
  // ----------------------------------------------------------------------------------------------
  bool renderCityBlock = true;
  bool streamCityBlock = true;
  bool trackCyclist = false;

  Options options(renderCityBlock, streamCityBlock, trackCyclist);

  if (renderCityBlock)
  {
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI> ();

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(options.filepath);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    if (!streamCityBlock)
    {
      // Load single-frame city block
      inputCloudI = pointProcessorI->loadPcd(options.filepath + "/0000000000.pcd");
      cityBlock(viewer, pointProcessorI, inputCloudI, options);
    }

    while (!viewer->wasStopped())
    {
      if (streamCityBlock)
      {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI, options);

        streamIterator++;
        if (streamIterator == stream.end())
          streamIterator = stream.begin();

        // Delay playback by 75 ms
        viewer->spinOnce(75);
      }

      else
        viewer->spinOnce();
    }
  }

  else
  {
    // Render Simple Highway
    simpleHighway(viewer, options);

    while (!viewer->wasStopped())
      viewer->spinOnce();
  }
}