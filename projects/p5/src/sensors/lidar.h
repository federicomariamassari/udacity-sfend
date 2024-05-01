#ifndef LIDAR_H
#define LIDAR_H

#include "../render/render.h"
#include <ctime>
#include <chrono>

const double pi = 3.1415;

struct Ray
{
  Vect3 origin;
  double resolution;
  Vect3 direction;
  Vect3 castPosition;
  double castDistance;

  /**
   * @param setOrigin The starting position from where the ray is cast.
   * @param horizontalAngle The angle of direction the ray travels on the xy plane.
   * @param verticalAngle The angle of direction between xy plane and ray (for example, 0 radians is along the xy 
   *   plane and pi/2 radians is straight up).
   * @param resolution The magnitude of the ray's step, used for ray casting, the smaller the more accurate, but the 
   *   more expensive.
   */
  Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
    : origin(setOrigin), resolution(setResolution), direction(resolution * cos(verticalAngle) * cos(horizontalAngle), 
      resolution * cos(verticalAngle) * sin(horizontalAngle), resolution * sin(verticalAngle)), castPosition(origin), 
      castDistance(0)
  {}

  void rayCast(const std::vector<Car>& cars, double minDistance, double maxDistance, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double slopeAngle, double sderr)
  {
    // Reset ray
    castPosition = origin;
    castDistance = 0;

    bool collision = false;

    while (!collision && (castDistance < maxDistance) && (castPosition.y <= 6 && castPosition.y >= -6 && 
      castPosition.x <= 50 && castPosition.x >= -15))
    {

      castPosition = castPosition + direction;
      castDistance += resolution;

      // Check if there is any collisions with ground slope
      collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

      // Check if there is any collisions with cars
      if (!collision && castDistance < maxDistance)
      {
        for (Car car : cars)
        {
          collision |= car.checkCollision(castPosition);

          if (collision)
            break;
        }
      }
    }

    if ((castDistance >= minDistance) && (castDistance <= maxDistance) && (castPosition.y <= 6 && castPosition.y >= -6 
      && castPosition.x <= 50 && castPosition.x >= -15))
    {
      // Add noise based on standard deviation error
      double rx = ((double) rand() / (RAND_MAX));
      double ry = ((double) rand() / (RAND_MAX));
      double rz = ((double) rand() / (RAND_MAX));

      cloud->points.push_back(pcl::PointXYZ(castPosition.x + rx * sderr, castPosition.y + ry * sderr, 
        castPosition.z + rz * sderr));
    }
  }
};

struct Lidar
{
  std::vector<Ray> rays;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<Car> cars;
  Vect3 position;

  double groundSlope;
  double minDistance;
  double maxDistance;
  double resolution;
  double sderr;

  Lidar(std::vector<Car> setCars, double setGroundSlope)
    : cloud(new pcl::PointCloud<pcl::PointXYZ>()), position(0, 0, 3.0)
  {
    minDistance = 0;
    maxDistance = 120;
    resolution = 0.2;
    
    sderr = 0.02;
    cars = setCars;
    groundSlope = setGroundSlope;

    int numLayers = 64;
    
    // The steepest vertical angle
    double steepestAngle =  24.8 * (-pi/180);
    double angleRange = 26.8 * (pi/180);

    double horizontalAngleInc = pi / 2250;

    double angleIncrement = angleRange / numLayers;

    for (double angleVertical = steepestAngle; angleVertical < (steepestAngle + angleRange); 
      angleVertical += angleIncrement)
    {
      for (double angle = 0; angle <= 2*pi; angle += horizontalAngleInc)
      {
        Ray ray(position, angle, angleVertical, resolution);
        rays.push_back(ray);
      }
    }
  }

  ~Lidar()  // PCL uses boost smart pointers for cloud pointer so no need to manually free memory
  {}

  void updateCars(std::vector<Car> setCars)
  {
    cars = setCars;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan()
  {
    cloud->points.clear();
    auto startTime = std::chrono::steady_clock::now();

    for (Ray ray : rays)
      ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, sderr);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    cout << "Ray casting took " << elapsedTime.count() << " milliseconds" << endl;

    cloud->width = cloud->points.size();
    cloud->height = 1;  // One-dimensional unorganized point cloud dataset
    return cloud;
  }
};

#endif /* LIDAR_H */