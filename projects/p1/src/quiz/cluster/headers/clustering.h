// 3-dimensional Euclidean Clustering implementation using templates
// Extended from cluster.cpp by Aaron Brown

#ifndef CLUSTERING_H
#define CLUSTERING_H

template<typename PointT>
void clusterHelper(int index, typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr& cluster, 
	std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol)
{
  processed[index] = true;
  cluster->points.push_back(cloud->points[index]);

  std::vector<int> nearest = tree->search(cloud->points[index], distanceTol);

  for (int id : nearest)
  {
    if(!processed[id])
      clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
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
    
    clusterHelper(i, cloud, cluster, processed, tree, distanceTol);

    // Reject clusters outside boundaries
    if (cluster->points.size() >= minSize && cluster->points.size() <= maxSize)
		clusters.push_back(cluster);
    
    i++;
  }
 
  return clusters;
}

#endif  /* CLUSTERING_H */