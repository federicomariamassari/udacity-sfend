#ifndef OPTIONS_H
#define OPTIONS_H

/* Options to fine-tune rendering and parameters for all scenarios in Udacity's Sensor Fusion
 * "LiDAR Object Detection" project.
 * Called from environment.cpp "main" function: tweak its main options for changes to take effect.
 */
struct Options
{
  // Rendering options
  bool renderScene;
  bool renderLidarScans;
  bool renderDataPoints;
  bool renderPlaneCloud;
  bool renderObstacleCloud;
  bool renderKdTree;
  bool renderClusters;
  bool renderEgoCarBox;
  bool renderBoxes;
  bool renderMinimumXyAlignedBoxes;

  // Input
  std::string filepath;

  // Filtering options (voxel grid, region of interest)
  bool filterPointCloud;
  float filterRes;
  Eigen::Vector4f minPoint;
  Eigen::Vector4f maxPoint;

  // Plane segmentation options
  int maxIterations;
  float distanceThreshold;

  // Clustering options
  float clusterTolerance;
  float minSize;
  float maxSize;

  Options(bool renderCityBlock, bool streamCityBlock, bool trackCyclist)

    // --------------------------------------------------------------------------------------------
    // SHARED OPTIONS
    // --------------------------------------------------------------------------------------------
    : renderDataPoints(false),  // true to render colorless point cloud data points
      renderPlaneCloud(true),  // true to render inliers in green
      renderObstacleCloud(true),  // true to render obstacles (non-inliers) in red

      renderClusters(true)  // true to render Euclidean Clustering on obstacle data
  {
    if (renderCityBlock)
    {  
      if (trackCyclist)
      {
        // ----------------------------------------------------------------------------------------
        // CHALLENGE PROBLEM: TRACKING A BICYCLIST OPTIONS
        // ----------------------------------------------------------------------------------------
        filepath = "../src/sensors/data/pcd/data_2";

        renderBoxes = true;  // true to render bounding boxes around obstacle data
        renderMinimumXyAlignedBoxes = false;

        // RANSAC 3D
        maxIterations = 55;
        distanceThreshold = 0.22;  // 22 cm

        // Euclidean clustering
        clusterTolerance = 0.32;  // 32 cm
        minSize = 8;  // To track the guard rails
        maxSize = 1200;
      }

      else
      {
        filepath = "../src/sensors/data/pcd/data_1";

        renderBoxes = false;  // true to render bounding boxes around obstacle data
        renderMinimumXyAlignedBoxes = true;  // true to render XY-plane-aligned minimum bounding boxes

        // RANSAC 3D
        maxIterations = 50;
        distanceThreshold = 0.15;  // 15 cm

        // Euclidean clustering
        clusterTolerance = 0.85;  // 85 cm
        minSize = 14;  // To track the side pole
        maxSize = 650;
      }

      // ------------------------------------------------------------------------------------------
      // CITY BLOCK OPTIONS
      // ------------------------------------------------------------------------------------------
      renderEgoCarBox = true;  // true to render a box of the approximate egoCar location
      renderKdTree = false;  // true to render 3D KD-Tree in the Viewer (custom addition)

      // Voxel grid
      filterPointCloud = true;  // true to downsample point cloud using a voxel grid filter
      filterRes = 0.20;  // Downsample point cloud with cubic voxels of side 20 cm

      // Region of interest - a box with the dimensions (meters): X in [30 North; 10 South],
      // Y in [5 East; 6 West]; Z in [-2 down; 1 up] since LiDAR is mounted on top of egoCar
      minPoint = {-10.0, -5.0, -2.0, 1.0};
      maxPoint = {30.0, 6.0, 1.0, 1.0};
    }

    else
    {
      // ------------------------------------------------------------------------------------------
      // SIMPLE HIGHWAY OPTIONS
      // ------------------------------------------------------------------------------------------
      renderScene = false;  // true to display highway and cars
      renderLidarScans = false;  // true to render LiDAR scans
      renderKdTree = true;  // true to render 3D KD-Tree in the Viewer (custom addition)

      // RANSAC 3D
      maxIterations = 100;
      distanceThreshold = 0.2;  // 20 cm

      // Euclidean clustering
      clusterTolerance = 2.0;  // 2 meters
      minSize = 3;
      maxSize = 30;
    }

    if (renderBoxes && renderMinimumXyAlignedBoxes)
      // Cannot render all boxes at the same time due to clusterId clash
      renderMinimumXyAlignedBoxes = false;
  }

  ~Options() 
  {}
};

#endif /* OPTIONS_H */