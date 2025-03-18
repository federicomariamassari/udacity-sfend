/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL for exploring self-driving car sensors.

#include "highway.h"

void initCamera(CameraAngle& setAngle, pcl::visualization::PCLVisualizer::Ptr viewer)
{
  viewer->setBackgroundColor(0, 0, 0);

  // Set camera position and angle
  viewer->initCameraParameters();
  
  float x_pos = 0;
  int distance = 16;  // Distance away in meters

  switch (setAngle)
  {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);  // Custom addition
      break;

    case Default:
      viewer->setCameraPosition((x_pos - 26), 0, 15.0, (x_pos + 25), 0, 0, 0, 0, 1);  // From starter code
      break;
  }

  viewer->addCoordinateSystem(1.0);  
}


int main(int argc, char** argv)
{
  ProjectOptions options = ProjectOptions();  // See highway.h for a list of options to set

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  initCamera(options.setAngle, viewer);

  Highway highway(viewer, options);

  int frame_per_sec = 30;
  int sec_interval = 10;
  int frame_count = 0;
  int time_us = 0;

  double egoVelocity = 25;

  while (frame_count < (frame_per_sec * sec_interval))
  {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    highway.stepHighway(egoVelocity, time_us, frame_per_sec, viewer, options);
    viewer->spinOnce(1000 / frame_per_sec);
    frame_count++;
    time_us = 1000000 * frame_count / frame_per_sec;
  }
}