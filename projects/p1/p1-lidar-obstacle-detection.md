[Home](../../README.md) | Next: 

# Project 1: LiDAR Obstacle Detection

## Overview

The project is organized as follows. The cloud is initially filtered to shrink its size, so as to reduce computational processing burden across consecutive frames (using voxel grid and region-of-interest ROI techniques), then 3-dimensional RANSAC is used to separate points belonging to the road plane from those belonging to obstacles (be it incoming vehicles or other still obstacles, we do not know at this stage). Then, based on point proximity, we distinguish across various clusters of points. And finally, we bind the clusters within boxes.

Different degrees of obstacle detection (of increasing complexity) are presented in this project. 
1. Simple detection from a sample environment (3 cars). This serves as testing environment for RANSAC, clustering, and the bounding box algorithms.
2. Data from a real point cloud from Udacity's self-driving car, Carla: both static (one single frame) and dynamic (streaming of multiple frames).
3. Of the above point, there are two options: linear, and highly dynamic/non-linear.

Points 1 and 2 (static, linear) are generally easy to fit, but the highly non-linear case is requires advanced techniques which are not entirely explored in ths project for now. 

##

```bash
.
├── CMakeLists.txt
└── src
    ├── custom
    │   ├── clustering.h
    │   ├── kdtree3d.h
    │   └── options.h
    ├── environment.cpp
    ├── processPointClouds.cpp
    ├── processPointClouds.h
    ├── quiz
    │   ├── cluster
    │   │   ├── CMakeLists.txt
    │   │   ├── cluster.cpp
    │   │   ├── cluster3d.cpp
    │   │   └── headers
    │   │       ├── clustering.h
    │   │       ├── kdtree.h
    │   │       └── kdtree3d.h
    │   └── ransac
    │       ├── CMakeLists.txt
    │       ├── ransac2d.cpp
    │       └── ransac3d.cpp
    ├── render
    │   ├── box.h
    │   ├── render.cpp
    │   └── render.h
    └── sensors
        ├── data
        │   └── pcd
        │       ├── data_1
        │       │   ├── ...
        │       │   └── 0000000021.pcd
        │       ├── data_2
        │       │   ├── ...
        │       │   └── 0000000153.pcd
        │       └── simpleHighway.pcd
        └── lidar.h
```

### Voxel Grid

The original point cloud is filtered using voxel grid technique. A voxel (volumetric pixel) is a 3-dimensional [...]. The raw point cloud is subdivided into 3D cubes 20 centimeters in side, and all the points belonging to a particular cube are "approximated" by their _centroid_, a single point in 3D space which has as coordinates the means of all 3 coordinates of the enclosed points. Dimensionality reduction is thus achieved by ?? this cloud of points by the single centroid. 20 cm was found to be a good trade-off between dimensionality reduction and precision/details kept. Cloud achieved a ration of ~100k to ~5k points per frame. Dimension reduction is particularly important for self-driving cars or applications where fast processing of data is essential.

### Region Of Interest (ROI)

Region-based fitting consists, instead, in keeping only a certain section (rectangular prism) of the diving environment, discarding the edges of low significance for object detection and perception of the surroundings, and which introduce additional computational burden (they add little benefit in processing the area, add little information). Since LiDAR is mounted on top of the car (to be able to continuously rotate 360° and perceive the environment), in order to keep the road plane, Z <belongs> [-2m, 1m]. X, Y, instead, are approximately symmetric and in [ ; ] and [ ; ]. This way we detect the road, but no wall or parked cars.

### Bounding Boxes

Fitting bounding boxes to point cloud clusters is useful to visualize the edges to which the autonomous agent would expect to bump into the obstacle if these were touched. Two kinds of boxes are fitted as part of this project: "constant" bounding boxes, and minimum "PCA" bounding boxes.

#### Regular Bounding Boxes

For regular bounding boxes, the minimum and maximum coordinates across the $x$, $y$, and $z$ dimensions are used to determine the vertices of a rectangular prism fitting the point cloud. The conventions are: length extends along $x$, width along $y$, and height along $z$. The advantage of regular bounding boxes is that they are computationally inexpensive--find the minumum and maximum points in 3D, then enclose the point cloud inside the box obtained by connecting these vertices. The drawback, however, is that, if a point cloud extends diagonally, the box will fit too much empty space, so areas which are actually free to move across will be marked as occupied.

#### Minimum Bounding Boxes

An alternative is minimum bounding boxes. A technique to fit these uses Principal Component Analysis (PCA), while other, more advanced techniques such as convex hull [1] are also available but not covered in this project.

PCA boxes solve the problem of excessive fitting of diagonal point clouds, but are computationally expensive and might be unstable, depending on the detected point cloud across frames. PCA boxes also rotate and swing unpredictably, as the only requirement is ?? [2].

- Image of bounding box vs PCA bounding box for diagonal point clouds
- Image of cross

## Main issues

- Calibration of parameters: RANSAC inliers threshold, Euclidean clustering point-proximity tolerance level, minimum number of points in a cluster (minimum number of points to detect the poles on the road, but also detect spurious clusters which are instead part of another larger cloud).
- Using PCA boxes in point 2, non-linear helps, but it is not enough especially because, due to the high variability of points in the point cloud clusters, the principal axes are not always correctly aligned, so additional corrective steps are needed.
- Ego car: Instead of extracting the point cloud associated to ego car at each frame, to avoid flickering and reduce computational burden, the very first frame is analysed and the location kept constant for all subsequent stills.

1. https://en.wikipedia.org/wiki/Minimum_bounding_box_algorithms
2. Dimitrov, D., Knauer, C., Kriegel, K., Rote, G.: [On the Bounding Boxes Obtained by Principal Component Analysis](https://www.researchgate.net/publication/235758825_On_the_bounding_boxes_obtained_by_principal_component_analysis), 2014, ResearchGate.

[Home](../../README.md) | Next: 
