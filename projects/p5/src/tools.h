#ifndef TOOLS_H_
#define TOOLS_H_

#include <iostream>
#include <random>
#include <vector>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>  // If Eigen is installed, else if in current working directory #include "Eigen/Dense"
#include "render/render.h"
#include "render/box.h"  // To compute LiDAR markers' position based on bounding box coordinates

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct lmarker
{
  double x, y;

  lmarker(double setX, double setY)
    : x(setX), y(setY)
  {}
};

struct rmarker
{
  double rho, phi, rho_dot;
  
  rmarker(double setRho, double setPhi, double setRhoDot)
    : rho(setRho), phi(setPhi), rho_dot(setRhoDot)
  {}
};

class Tools
{
  public:
    
    Tools();
    
    virtual ~Tools();
    
    // Members
    std::vector<VectorXd> estimations;
    std::vector<VectorXd> ground_truth;
    
    /**
     * @brief Gaussian random noise, to simulate imperfection of real-life measurements.
     * 
     * @param stddev Standard deviation of the random noise.
     * @param seedNum Random seed.
     */
    double noise(double stddev, long long seedNum);

    /**
     * @brief Sense where a car is located using LiDAR measurement.
     * 
     * @param car A car object.
     * @param viewer The PCL Visualizer object.
     * @param timestamp The current timestamp.
     * @param visualize Whether to add a red sphere on top of the tracked object.
     */
    lmarker lidarSense(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize);

    /**
     * @brief Compute LiDAR marker from bounding box (x,y) centerpoint (custom addition).
     * 
     * @param car A car object.
     * @param box A bounding box object, framing the point cloud cluster associated to the car.
     * @param viewer The PCL Visualizer object.
     * @param timestamp The current timestamp.
     * @param visualize Whether to add a red sphere on top of the tracked object.
     */
    lmarker lidarSense(Car& car, Box& box, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, 
      bool visualize);

    /**
     * @brief Sense where a car is located using radar measurement.
     * 
     * @param car A car object.
     * @param ego The ego car object.
     * @param viewer The PCL Visualizer object.
     * @param timestamp The current timestamp.
     * @param visualize Whether to add a red sphere on top of the tracked object.
     */
    rmarker radarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, 
      bool visualize);

    /**
     * @brief Show UKF tracking and also allow showing predicted future path.
     * 
     * @param car A car object.
     * @param viewer The PCL Visualizer object.
     * @param time Time ahead in the future to predict.
     * @param steps How many steps to show between present and time and future time.
     */
    void ukfResults(Car car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps);

    /**
    * @brief Helper method to calculate RMSE.
    * 
    * @param estimations Estimates from the Unscented Kálmán Filter.
    * @param ground_truth The ground truth measurements.
    */
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
     * @brief Save point cloud to .pcd file.
     * 
     * @param cloud The point cloud pointer object.
     * @param file Path to the file.
     */
    void savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file);

    /**
     * @brief Load point cloud from .pcd file.
     * 
     * @param file Path to the file.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(std::string file);
};

#endif /* TOOLS_H_ */