// 3-dimensional KD-Tree implementation using templates
// Extended from kdtree.h by Aaron Brown

#ifndef KDTREE_3D_H
#define KDTREE_3D_H

#include "../render/render.h"

// Structure to represent node of KD-Tree
template<typename PointT>
struct Node
{
  PointT point;
  int id;
  Node<PointT>* left;
  Node<PointT>* right;

  Node(PointT arr, int setId)
  : point(arr), id(setId), left(NULL), right(NULL)
  {}

  ~Node()
  {
    delete left;
    delete right;
  }
};

// Structure to represent a 3-dimensional KD-Tree
template<typename PointT>
struct KdTree
{
  Node<PointT>* root;

  KdTree()
  : root(NULL)
  {}

  ~KdTree()
  {
    delete root;
  }

  void insertHelper(Node<PointT>** node, uint depth, PointT point, int id)
  {
    // Tree is empty
    if (*node == NULL)
      *node = new Node<PointT>(point, id);

    else
    {
      // Check which dimension to split by: x, y, z
      uint cd = depth % 3;
      float lhs, rhs;

      switch(cd)
      {
        case 0:
          // Split by x
          lhs = point.x;
          rhs = (*node)->point.x;
          break;

        case 1:
          // Split by y
          lhs = point.y;
          rhs = (*node)->point.y;
          break;

        case 2:
          // Split by z
          lhs = point.z;
          rhs = (*node)->point.z;
          break;
      }

      if (lhs < rhs)
        insertHelper(&((*node)->left), depth+1, point, id);
      else
        insertHelper(&((*node)->right), depth+1, point, id);
    }
  }

  void insert(PointT point, int id)
  {
    insertHelper(&root, 0, point, id);
  }

  void searchHelper(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
  {
    if (node != NULL)
    {
      // Box-check lazy evaluation: does the point lie within the cube centered on target?
      if (abs(node->point.x-target.x) <= distanceTol && abs(node->point.y-target.y) <= distanceTol && abs(node->point.z-target.z) <= distanceTol)
      {
        // Compare argument of norm to squared tolerance since square root is an expensive operation
        float sumOfSquares = pow(node->point.x-target.x, 2) + pow(node->point.y-target.y, 2) + pow(node->point.z-target.z, 2);

        if (sumOfSquares <= pow(distanceTol, 2))
          ids.push_back(node->id);
      }

      float lhs, rhs;

      // Where will the node flow?
      switch(depth % 3)
      {
        case 0:
          lhs = target.x;
          rhs = node->point.x;
          break;

        case 1:
          lhs = target.y;
          rhs = node->point.y;
          break;

        case 2:
          lhs = target.z;
          rhs = node->point.z;
          break;
      }

      if (lhs-distanceTol < rhs)
        searchHelper(target, node->left, depth+1, distanceTol, ids);

      if (lhs+distanceTol > rhs)
        searchHelper(target, node->right, depth+1, distanceTol, ids);
    }
  }

  // Return a list of point ids in the tree which are within distance of the target
  std::vector<int> search(PointT target, float distanceTol)
  {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);

    return ids;
  }
};

/* Initialize the boundaries of the box enclosing a 3D KD-Tree for rendering.
   This code in this method was suggested by Udacity GPT.
 */
template<typename PointT>
Box initKdTreeBox(typename pcl::PointCloud<PointT>::Ptr cloud)
{
  Box window;

  // These initial limits will always be updated in the first pass
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();

  // Iterate through points in the cluster
  for (const auto& point : cloud->points) {

    // Update minimum and maximum values of x, y, and z
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }

  window.x_min = min_x;
  window.x_max = max_x;
  window.y_min = min_y;
  window.y_max = max_y;
  window.z_min = min_z;
  window.z_max = max_z;

  return window;
}

/* Efficient 3D rendering of a KD-Tree.
 */
template<typename PointT>
void render3DTree(Node<PointT>* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{
  if (node != NULL)
  {
    Box upperWindow = window;
    Box lowerWindow = window;

    typename pcl::PointCloud<PointT>::Ptr vertices (new pcl::PointCloud<PointT>);

    // Cloud will always store the 4 vertices of a plane
    vertices->points.reserve(4);

    // Split on x-axis
    if (depth % 3 == 0)
    {
      // Draw a plane perpendicular to the x-axis; point insertion order is important when drawing polygons
      vertices->points.push_back(PointT(node->point.x, window.y_min, window.z_min));
      vertices->points.push_back(PointT(node->point.x, window.y_max, window.z_min));
      vertices->points.push_back(PointT(node->point.x, window.y_max, window.z_max));
      vertices->points.push_back(PointT(node->point.x, window.y_min, window.z_max)); 

      // https://stackoverflow.com/questions/28296876/creating-a-polygon-with-point-cloud
      viewer->addPolygon<pcl::PointXYZ>(vertices, 0, 0, 1, "vertices" + std::to_string(iteration));

      lowerWindow.x_max = node->point.x;
      upperWindow.x_min = node->point.x;
    }

    // Split on y-axis
    else if (depth % 3 == 1)
    {
      // Draw a plane perpendicular to the y-axis
      vertices->points.push_back(PointT(window.x_min, node->point.y, window.z_min));
      vertices->points.push_back(PointT(window.x_max, node->point.y, window.z_min));
      vertices->points.push_back(PointT(window.x_max, node->point.y, window.z_max)); 
      vertices->points.push_back(PointT(window.x_min, node->point.y, window.z_max));

      viewer->addPolygon<PointT>(vertices, 1, 0, 0, "vertices" + std::to_string(iteration));

      lowerWindow.y_max = node->point.y;
      upperWindow.y_min = node->point.y;
    }

    // Split on z-axis (depth % 3 == 2)
    else
    {
      // Draw a plane perpendicular to the z-axis
      vertices->points.push_back(PointT(window.x_min, window.y_min, node->point.z));
      vertices->points.push_back(PointT(window.x_max, window.y_min, node->point.z));
      vertices->points.push_back(PointT(window.x_max, window.y_max, node->point.z));
      vertices->points.push_back(PointT(window.x_min, window.y_max, node->point.z));

      viewer->addPolygon<PointT>(vertices, 0, 1, 0, "vertices" + std::to_string(iteration));

      lowerWindow.z_max = node->point.z;
      upperWindow.z_min = node->point.z; 
    }

    iteration++;

    render3DTree(node->left, viewer, lowerWindow, iteration, depth + 1);
    render3DTree(node->right, viewer, upperWindow, iteration, depth + 1);
  }
}

#endif  /* KDTREE_3D_H */