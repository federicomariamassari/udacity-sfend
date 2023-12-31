/* 3-Dimensional Euclidean Clustering algorithm implementation
 * Extended from Aaron Brown's solution to Euclidean Clustering 2D (quiz/cluster.cpp) [1]
 *
 * Resources:
 * [1] - Euclidean Clustering (Clustering Obstacles: Lesson 8), Udacity Sensor Fusion Nanodegree
 */
#ifndef CLUSTERING_H
#define CLUSTERING_H

// Extended from Aaron Brown's solution [1]
template<typename PointT>
void proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr& cluster, 
	std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol)
{
  processed[index] = true;
  cluster->points.push_back(cloud->points[index]);

  std::vector<int> nearest = tree->search(cloud->points[index], distanceTol);

  for (int id : nearest)
  {
    if (!processed[id])
      proximity(id, cloud, cluster, processed, tree, distanceTol);
  }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, 
	KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  std::vector<bool> processed (cloud->points.size(), false);

  int i = 0;
  while (i < cloud->points.size())
  {
    if (processed[i])
    {
      i++;
      continue;
    }

    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

    proximity(i, cloud, cluster, processed, tree, distanceTol);

    // Reject clusters outside boundaries
    if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize)
      clusters.push_back(cluster);

    i++;
  }

  return clusters;
}

#endif  /* CLUSTERING_H */