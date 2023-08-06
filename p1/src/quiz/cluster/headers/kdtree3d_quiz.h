// 3-dimensional KD-Tree implementation
// Extended from kdtree.h by Aaron Brown

#include "../../../render/render.h"

// Structure to represent node of KD Tree
struct Node
{
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int setId)
  : point(arr), id(setId), left(NULL), right(NULL)
  {}

  ~Node()
  {
    delete left;
    delete right;
  }
};


struct KdTree
{
  Node* root;

  KdTree()
  : root(NULL)
  {}

  ~KdTree()
  {
    delete root;
  }

  void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
  {
    // Tree is empty
    if (*node == NULL)
      *node = new Node(point, id);

    else
    {
      // Check which dimension to split by: x, y, z
      uint cd = depth % 3;

      if (point[cd] < (*node)->point[cd])
        insertHelper(&((*node)->left), depth+1, point, id);

      else
        insertHelper(&((*node)->right), depth+1, point, id);
    }
  }

  void insert(std::vector<float> point, int id)
  {
    insertHelper(&root, 0, point, id);
  }

  void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
  {
    if (node != NULL)
    {
      // Box-check: does the point lie within the cube centered on target?
      if (abs(node->point[0]-target[0]) <= distanceTol && abs(node->point[1]-target[1]) <= distanceTol && abs(node->point[2]-target[2]) <= distanceTol)
      {
        // Square root is an expensive operation; compare norm argument to squared tolerance instead
        float sumOfSquares = pow(node->point[0]-target[0], 2) + pow(node->point[1]-target[1], 2) + pow(node->point[2]-target[2], 2);

        if (sumOfSquares <= pow(distanceTol, 2))
          ids.push_back(node->id);
      }

      // Where will the node flow?
      if (target[depth % 3]-distanceTol < node->point[depth % 3])
        searchHelper(target, node->left, depth+1, distanceTol, ids);

      if (target[depth % 3]+distanceTol > node->point[depth % 3])
        searchHelper(target, node->right, depth+1, distanceTol, ids);
    }
  }

  // Return a list of point ids in the tree which are within distance of the target
  std::vector<int> search(std::vector<float> target, float distanceTol)
  {
    std::vector<int> ids;
    searchHelper(target, root, 0, distanceTol, ids);

    return ids;
  }
};