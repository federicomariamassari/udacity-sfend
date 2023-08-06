// KD-Tree and Euclidean Clustering 3D
// Extended from cluster.cpp by Aaron Brown

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "headers/kdtree3d.h"

pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);

  // Add a semi-transparent black 10 x 10 cube
  viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, window.z_min, window.z_max, 0, 0, 0, "window");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "window");
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3d(std::vector<std::vector<float>> points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  for(int i = 0; i < points.size(); i++)
  {
    pcl::PointXYZ point;
    point.x = points[i][0];
    point.y = points[i][1];
    point.z = points[i][2];

    cloud->points.push_back(point);
  }

  cloud->width = cloud->points.size();

  // Data provided is a single-layer point cloud (single scan or measurement from LiDAR) [Source: Udacity GPT]
  cloud->height = 1;

  return cloud;
}

void render3DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{
  if (node != NULL)
  {
    Box upperWindow = window;
    Box lowerWindow = window;

    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZ>);

    // Cloud will always store the 4 vertices of a plane
    vertices->points.reserve(4);

    // Split on x-axis
    if (depth % 3 == 0)
    {
      // Draw a plane perpendicular to the x-axis
      // Point insertion order is important when drawing polygons
      vertices->points.push_back(pcl::PointXYZ(node->point[0], window.y_min, window.z_min));
      vertices->points.push_back(pcl::PointXYZ(node->point[0], window.y_max, window.z_min));
      vertices->points.push_back(pcl::PointXYZ(node->point[0], window.y_max, window.z_max));
      vertices->points.push_back(pcl::PointXYZ(node->point[0], window.y_min, window.z_max)); 

      // https://stackoverflow.com/questions/28296876/creating-a-polygon-with-point-cloud
      viewer->addPolygon<pcl::PointXYZ>(vertices, 0, 0, 1, "vertices" + std::to_string(iteration));

      lowerWindow.x_max = node->point[0];
      upperWindow.x_min = node->point[0];
    }

    // Split on y-axis
    else if (depth % 3 == 1)
    {
      // Draw a plane perpendicular to the y-axis
      vertices->points.push_back(pcl::PointXYZ(window.x_min, node->point[1], window.z_min));
      vertices->points.push_back(pcl::PointXYZ(window.x_max, node->point[1], window.z_min));
      vertices->points.push_back(pcl::PointXYZ(window.x_max, node->point[1], window.z_max)); 
      vertices->points.push_back(pcl::PointXYZ(window.x_min, node->point[1], window.z_max));

      viewer->addPolygon<pcl::PointXYZ>(vertices, 1, 0, 0, "vertices" + std::to_string(iteration));

      lowerWindow.y_max = node->point[1];
      upperWindow.y_min = node->point[1];
    }

    // Split on z-axis (depth % 3 == 2)
    else
    {
      // Draw a plane perpendicular to the z-axis
      vertices->points.push_back(pcl::PointXYZ(window.x_min, window.y_min, node->point[2]));
      vertices->points.push_back(pcl::PointXYZ(window.x_max, window.y_min, node->point[2]));
      vertices->points.push_back(pcl::PointXYZ(window.x_max, window.y_max, node->point[2]));
      vertices->points.push_back(pcl::PointXYZ(window.x_min, window.y_max, node->point[2]));

      viewer->addPolygon<pcl::PointXYZ>(vertices, 0, 1, 0, "vertices" + std::to_string(iteration));

      lowerWindow.z_max = node->point[2];
      upperWindow.z_min = node->point[2]; 
    }

    iteration++;

    render3DTree(node->left, viewer, lowerWindow, iteration, depth + 1);
    render3DTree(node->right, viewer, upperWindow, iteration, depth + 1);
  }
}

void clusterHelper(int indice, std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
  processed[indice] = true;
  cluster.push_back(indice);

  std::vector<int> nearest = tree->search(points[indice], distanceTol);

  for (int id : nearest)
  {
    if(!processed[id])
      clusterHelper(id, points, cluster, processed, tree, distanceTol);
  }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{
  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed (points.size(), false);

  int i = 0;
  while (i < points.size())
  {
    if (processed[i])
    {
      i++;
      continue;
    }

    std::vector<int> cluster;
    clusterHelper(i, points, cluster, processed, tree, distanceTol);
    clusters.push_back(cluster);
    i++;
  }
 
  return clusters;
}

int main ()
{
  // Create viewer
  Box window;
  window.x_min = -10;
  window.x_max = 10;
  window.y_min = -10;
  window.y_max = 10;
  window.z_min = -10;
  window.z_max = 10;

  pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 40);
  
  std::vector<std::vector<float>> points = { 
    {3.2, 4.5, -7.8}, {2.5, 3.9, -6.2}, {3.4, 4.1, -7.1}, {3.1, 4.2, -6.1}, {3.8, 2.7, -5.9},
    {-1.2, -3.5, 2.8}, {-0.8, -2.7, 3.9}, {-2.5, -3.9, 2.2},
    {1.1, -2.9, 0.9}, {0.9, -2.8, 0.7},
    {0.1, 1.4, 7.1}, {0.2, 1.1, 6.4},
    {8.5, 8.9, -3.2}
  };
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3d(points);

  KdTree* tree = new KdTree;
  
  for (int i=0; i < points.size(); i++) 
    tree->insert(points[i], i);

  int it = 0;
  render3DTree(tree->root, viewer, window, it);

  std::cout << "Test Search" << std::endl;
  std::vector<int> nearby = tree->search({3.5, 4.2, -5.5}, 3.0);

  for(int index : nearby)
    std::cout << index << ", ";

  std::cout << std::endl;

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  
  std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "Clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  // Render found clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(0,1,1), Color(1,1,0)};
  
  for(std::vector<int> cluster : clusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());

    for(int indice : cluster)
      clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0], points[indice][1], points[indice][2]));

    renderPointCloud(viewer, clusterCloud, "cluster" + std::to_string(clusterId), colors[clusterId % 5]);

    ++clusterId;
  }
  
  if(clusters.size() == 0)
    renderPointCloud(viewer, cloud, "data");

  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
  }
}