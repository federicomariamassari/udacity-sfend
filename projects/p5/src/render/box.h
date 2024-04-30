#ifndef BOX_H
#define BOX_H

#include <Eigen/Geometry> 


struct BoxQ
{
  // Translation and scaling of the bounding box (position, size)
  Eigen::Vector3f bboxTransform;

  // Rotation of the bounding box (rotation)
  Eigen::Quaternionf bboxQuaternion;

  float cube_length;
  float cube_width;
  float cube_height;
};

struct Box
{
  float x_min;
  float y_min;
  float z_min;
  float x_max;
  float y_max;
  float z_max;
};

#endif /* BOX_H */