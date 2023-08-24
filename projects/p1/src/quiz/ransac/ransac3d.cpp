// RANSAC 3D
// Extended from ransac.cpp by Aaron Brown

#include <unordered_set>
#include <vector>
#include "../../render/render.h"
#include "../../processPointClouds.h"
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3d()
{
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);

  return viewer;
}

std::unordered_set<int> Ransac3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  auto startTime = std::chrono::steady_clock::now();

  while (maxIterations--) {
  
    // Randomly select three distinct point indices from the cloud to define a plane
    std::unordered_set<int> inliers;

    while (inliers.size() < 3)
      inliers.insert(rand() % (cloud->points.size()));

    // Store the plane inlier points based on extracted indices
    std::vector<pcl::PointXYZ> p(3);

    auto itr = inliers.begin();

    for (int i=0; i < p.size(); i++)
    {
      p[i] = cloud->points[*itr];
      itr++;
    }

    // Find coefficients of general equation of a plane Ax + By + Cz + D = 0 via cross-product
    float A = (p[1].y-p[0].y) * (p[2].z-p[0].z) - (p[1].z-p[0].z)*(p[2].y-p[0].y);
    float B = (p[1].z-p[0].z) * (p[2].x-p[0].x) - (p[1].x-p[0].x)*(p[2].z-p[0].z);
    float C = (p[1].x-p[0].x) * (p[2].y-p[0].y) - (p[1].y-p[0].y)*(p[2].x-p[0].x);
    float D = -(A*p[0].x + B*p[0].y + C*p[0].z);

    for (int i=0; i < cloud->points.size(); i++) {

      // Ignore points which are already inliers to the plane
      if (inliers.count(i) > 0)
        continue;

      // Check if current point is inlier to the plane
      pcl::PointXYZ point = cloud->points[i];

      float dist = fabs(A*point.x + B*point.y + C*point.z + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));

      if (dist <= distanceTol)
        inliers.insert(i);
    }

    // The plane with the most inliers is the best model
    if (inliers.size() > inliersResult.size())
      inliersResult = inliers;
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
  std::cout << "Clustering took " << elapsedTime.count() / 1000. << " milliseconds" << std::endl;

  return inliersResult;
}

int main ()
{
  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3d();

  std::unordered_set<int> inliers = Ransac3d(cloud, 100, 0.20);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++)
  {
    pcl::PointXYZ point = cloud->points[index];

    if(inliers.count(index))
      cloudInliers->points.push_back(point);

    else
      cloudOutliers->points.push_back(point);
  }

  // Render 3D point cloud with inliers and outliers
  if (inliers.size())
  {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  }
  
  else
    renderPointCloud(viewer,cloud,"data");
  
  while (!viewer->wasStopped ())
    viewer->spinOnce ();
}