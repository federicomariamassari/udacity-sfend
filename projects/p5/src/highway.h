/* \author Aaron Brown */
// Handle logic for creating traffic on a highway and animating it.

#ifndef HIGHWAY_H
#define HIGHWAY_H

#include "render/render.h"
#include "sensors/lidar.h"
#include "tools.h"

#include "custom/process_point_cloud.h"  // Voxel grid filtering, Euclidean clustering, bounding boxes
#include "custom/process_point_cloud.cpp"  // To help linker


/**
 * Customise rendered output
 */
struct ProjectOptions
{
  CameraAngle setAngle = XY;

  // Set which cars to track with UKF
  std::vector<bool> trackCars = {true, true, true};  // Cars' initial positions from ego vehicle: SW, NE, S

  // Visualize sensor measurements
  bool visualize_lidar = true;  // true to display red orb
  bool visualize_radar = true;  // true to display radar metric
  bool visualize_pcd = true;  // true to display colorless LiDAR point clouds, false for stylised green car shapes

  // Predict path in the future using UKF
  double projectedTime = 2;
  int projectedSteps = 6;

  bool filterPointCloud = true;  // true to downsample input point cloud using voxel grid filtering
  float voxelSide = 0.18f;  // 0.01f = 1 cm

  bool cluster_pcd = true;

  // Rendering options
  bool renderBoxes = true;  // true to render simple (rectangular prism) bounding boxes around clusters
};


class Highway
{
  public:
    std::vector<Car> traffic;
    std::vector<Box> boxes;  // To store bounding boxes associated to traffic car clusters
    Car egoCar;
    Tools tools;

    bool pass = true;
    std::vector<double> rmseThreshold = {0.30, 0.16, 0.95, 0.70};
    std::vector<double> rmseFailLog = {0.0, 0.0, 0.0, 0.0};
    Lidar* lidar;

    Highway(pcl::visualization::PCLVisualizer::Ptr& viewer, ProjectOptions& options)
    {
      tools = Tools();

      egoCar = Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");

      Car car1(Vect3(-10, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");
      std::vector<accuation> car1_instructions;
      accuation a = accuation(0.5*1e6, 0.5, 0.0);
      car1_instructions.push_back(a);
      a = accuation(2.2 * 1e6, 0.0, -0.2);
      car1_instructions.push_back(a);
      a = accuation(3.3 * 1e6, 0.0, 0.2);
      car1_instructions.push_back(a);
      a = accuation(4.4 * 1e6, -2.0, 0.0);
      car1_instructions.push_back(a);
      car1.setInstructions(car1_instructions);

      if (options.trackCars[0])
      {
        UKF ukf1;
        car1.setUKF(ukf1);
      }

      traffic.push_back(car1);

      Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");
      std::vector<accuation> car2_instructions;
      a = accuation(4.0 * 1e6, 3.0, 0.0);
      car2_instructions.push_back(a);
      a = accuation(8.0 * 1e6, 0.0, 0.0);
      car2_instructions.push_back(a);
      car2.setInstructions(car2_instructions);

      if (options.trackCars[1])
      {
        UKF ukf2;
        car2.setUKF(ukf2);
      }

      traffic.push_back(car2);

      Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");
      std::vector<accuation> car3_instructions;
      a = accuation(0.5 * 1e6, 2.0, 1.0);
      car3_instructions.push_back(a);
      a = accuation(1.0 * 1e6, 2.5, 0.0);
      car3_instructions.push_back(a);
      a = accuation(3.2 * 1e6, 0.0, -1.0);
      car3_instructions.push_back(a);
      a = accuation(3.3 * 1e6, 2.0, 0.0);
      car3_instructions.push_back(a);
      a = accuation(4.5 * 1e6, 0.0, 0.0);
      car3_instructions.push_back(a);
      a = accuation(5.5 * 1e6, -2.0, 0.0);
      car3_instructions.push_back(a);
      a = accuation(7.5 * 1e6, 0.0, 0.0);
      car3_instructions.push_back(a);
      car3.setInstructions(car3_instructions);

      if (options.trackCars[2])
      {
        UKF ukf3;
        car3.setUKF(ukf3);
      }

      traffic.push_back(car3);

      lidar = new Lidar(traffic, 0);

      // Render environment
      renderHighway(0, viewer);
      egoCar.render(viewer);
      car1.render(viewer);
      car2.render(viewer);
      car3.render(viewer);
    }

    void stepHighway(double egoVelocity, long long timestamp, int frame_per_sec, 
      pcl::visualization::PCLVisualizer::Ptr& viewer, ProjectOptions& options)
    {
      if (options.visualize_pcd)  // Display and process point cloud LiDAR data
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = tools.loadPcd("../src/sensors/data/pcd/highway_" + 
          std::to_string(timestamp) + ".pcd");

        if (options.filterPointCloud)  // Voxel grid filtering
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);

          filterCloud<pcl::PointXYZ>(trafficCloud, filteredCloud, options.voxelSide);
          trafficCloud = filteredCloud;
        }

        float clusterTol = 1.2;
        int minSize = 50;
        int maxSize = 1000;

        if (options.cluster_pcd)  // Euclidean clustering
        {
          vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
          euclideanClustering<pcl::PointXYZ>(trafficCloud, clusters, clusterTol, minSize, maxSize);

          renderClusters<pcl::PointXYZ>(viewer, clusters);

          if (options.renderBoxes)  // Render simple bounding boxes
          {
            int clusterId = 0;
            for (const auto& cluster : clusters)
            {
              Box box;
              boundingBox<pcl::PointXYZ>(cluster, box);
              renderBox(viewer, box, clusterId);
              boxes.push_back(box);
              ++clusterId;
            }
          }
        }

        else
          renderPointCloud(viewer, trafficCloud, "trafficCloud", Color((float) 184/256, (float) 223/256, 
            (float) 252/256));
      }

      // Render highway environment with poles
      renderHighway(egoVelocity * timestamp / 1e6, viewer);
      egoCar.render(viewer);

      for (int i = 0; i < traffic.size(); i++)
      {
        traffic[i].move((double) 1 / frame_per_sec, timestamp);

        if (!options.visualize_pcd)
          traffic[i].render(viewer);

        // Sense surrounding cars with LiDAR and radar
        if (options.trackCars[i])
        {
          VectorXd gt(4);
          gt << traffic[i].position.x, traffic[i].position.y, traffic[i].velocity * cos(traffic[i].angle), 
            traffic[i].velocity * sin(traffic[i].angle);

          tools.ground_truth.push_back(gt);

          if (options.visualize_pcd && options.cluster_pcd && options.renderBoxes)
          {
            // Sort bounding boxes in accordance with traffic vector
            sortBoundingBoxes(traffic, boxes);
            tools.lidarSense(traffic[i], boxes[i], viewer, timestamp, options.visualize_lidar);
          }

          else
            tools.lidarSense(traffic[i], viewer, timestamp, options.visualize_lidar);

          // PCD is only relevant when we compute the laser estimates
          tools.radarSense(traffic[i], egoCar, viewer, timestamp, options.visualize_radar);

          tools.ukfResults(traffic[i], viewer, options.projectedTime, options.projectedSteps);
          VectorXd estimate(4);

          double v = traffic[i].ukf.x_(2);
          double yaw = traffic[i].ukf.x_(3);
          double v1 = cos(yaw) * v;
          double v2 = sin(yaw) * v;

          estimate << traffic[i].ukf.x_[0], traffic[i].ukf.x_[1], v1, v2;

          tools.estimations.push_back(estimate);
        }
      }

      boxes.clear();  // Avoid bounding boxes accumulation

      viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
      VectorXd rmse = tools.CalculateRMSE(tools.estimations, tools.ground_truth);
      viewer->addText("X: " + std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
      viewer->addText("Y: " + std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
      viewer->addText("Vx: " + std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
      viewer->addText("Vy: " + std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");

      if (timestamp > 1.0e6)
      {
        if (rmse[0] > rmseThreshold[0])
        {
          rmseFailLog[0] = rmse[0];
          pass = false;
        }

        if (rmse[1] > rmseThreshold[1])
        {
          rmseFailLog[1] = rmse[1];
          pass = false;
        }

        if (rmse[2] > rmseThreshold[2])
        {
          rmseFailLog[2] = rmse[2];
          pass = false;
        }

        if (rmse[3] > rmseThreshold[3])
        {
          rmseFailLog[3] = rmse[3];
          pass = false;
        }
      }

      if (!pass)
      {
        viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
        
        if(rmseFailLog[0] > 0)
          viewer->addText("X: " + std::to_string(rmseFailLog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
        
        if(rmseFailLog[1] > 0)
          viewer->addText("Y: " + std::to_string(rmseFailLog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
        
        if(rmseFailLog[2] > 0)
          viewer->addText("Vx: " + std::to_string(rmseFailLog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
        
        if(rmseFailLog[3] > 0)
          viewer->addText("Vy: " + std::to_string(rmseFailLog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
      } 
    }
};

#endif /* HIGHWAY_H */