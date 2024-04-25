#include "process_point_cloud.h"


template<typename PointT>
void filterCloud(const typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr filtered, 
  float filterRes)
{
  // Create the filtering object
  pcl::VoxelGrid<PointT> voxelGrid;
  voxelGrid.setInputCloud(cloud);

  // Each voxel (volumetric pixel) is a cube with side length filterRes (0.01f = 1 cm)
  voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
  voxelGrid.filter(*filtered);
}

template<typename PointT>
void euclideanClustering(const typename pcl::PointCloud<PointT>::Ptr cloud, 
  vector<typename pcl::PointCloud<PointT>::Ptr>& clusters, float clusterTol, int minSize, int maxSize)
{
  // Create KD-Tree object for the search method of extraction [1]
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  // Indices extraction [1]
  vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTol);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  // Populate clusters [1] [2]
  for (const auto& cluster : clusterIndices)
  {
    typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

    for (const auto& index : cluster.indices)
      cloudCluster->push_back((*cloud)[index]);

    cloudCluster->width = cloudCluster->size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;

    clusters.push_back(cloudCluster);
  }
}

template <typename PointT>
void renderClusters(pcl::visualization::PCLVisualizer::Ptr& viewer, 
  const std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters)
{
  int clusterId = 0;
  
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
  
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
  {
    renderPointCloud(viewer, cluster, "cluster" + std::to_string(clusterId), colors[clusterId % colors.size()]);
    ++clusterId;
  }
}

template <typename PointT>
void boundingBox(const typename pcl::PointCloud<PointT>::Ptr cluster, Box& box)
{
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;

  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;
}

float norm(const Car& car, const Box& box)
{
  float x_mid = (box.x_min + box.x_max) * 0.5;
  float y_mid = (box.y_min + box.y_max) * 0.5;

  return sqrt(pow(car.position.x - x_mid, 2) + pow(car.position.y - y_mid, 2));
}

void sortBoundingBoxes(const std::vector<Car>& traffic, std::vector<Box>& boxes)
{
  std::vector<Box> sortedBoxes;

  for (const auto& car : traffic)
  {
    std::sort(boxes.begin(), boxes.end(), [&](const Box& box1, const Box& box2) 
    {
      return norm(car, box1) < norm(car, box2);
    });

    sortedBoxes.push_back(boxes[0]);
    boxes.erase(boxes.begin());
  }

  boxes = sortedBoxes;
}