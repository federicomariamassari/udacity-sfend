[Home](../../README.md) | Next: 

# Project 1: LiDAR Obstacle Detection

## Overview

In autonomous systems such as robots and self-driving cars, LiDAR (Light Detection And Ranging) is commonly used as a way to accurately measure distances and create detailed 3-dimensional maps of the surrounding environment. LiDAR targets surfaces with laser beams (pulses of a few nanoseconds) and measures the time it takes for them to bounce back. By doing so, it generates Point Cloud Data (PCD).

In this project, my very first using Point Cloud Library (PCL), I filter, segment, and cluster raw data from LiDAR scans to detect incoming vehicles and obstacles within a driving environment. 

The project is organized as follows. The cloud is initially filtered to shrink its size, so as to reduce computational processing burden across consecutive frames (using voxel grid and region-of-interest ROI techniques), then 3-dimensional RANSAC is used to separate points belonging to the road plane from those belonging to obstacles (be it incoming vehicles or other still obstacles, we do not know at this stage). Then, based on point proximity, we distinguish across various clusters of points. And finally, we bind the clusters within boxes.

Different degrees of obstacle detection (of increasing complexity) are presented in this project.

1. Simple detection from a sample environment (3 cars). This serves as testing environment for RANSAC, clustering, and the bounding box algorithms.
2. Data from a real point cloud from Udacity's self-driving car, Carla: both static (one single frame) and dynamic (streaming of multiple frames).
3. Of the above point, there are two options: linear, and highly dynamic/non-linear.

Points 1 and 2 (static, linear) are generally easy to fit, but the highly non-linear case is requires advanced techniques which are not entirely explored in this project for now.

## Project Structure

The directory structure tree for the project appears in Figure 2. Three programs can be compiled: the main one, `environment`, inside `src`, plus two test implementations for RANSAC as well as KD-Trees and Euclidean clustering (2D/3D) inside `quiz`.

Figure 2 shows the directory structure tree for the projects. There are 3 `CMakeLists.txt` overall, which means it's possible to compile three different projects. The top level is the main project, and then there are two in the `quiz` folder. The quiz folder contains sample programs on RANSAC, KD-Trees, and Euclidean Clustering built during the course, as basis for the main file. `environment.cpp` is the main file, while `processPointClouds.cpp` contains all functions of the projects, and builds upon the header files in `custom`. `options.h` contains all rendering options, together with values for all the hyperparameters in the project. `kdtree.h` and `clustering.h` contain, respectively, the logic for KD-Trees and Euclidean Clustering in 3 dimensions. Voxel grid, region of interest, and RANSAC implementations are instead contained in `processPointClouds.cpp`. Quiz also has basic 2D implementations of the above. `render` contains helper functions for object rendering, while `sensors` contains, among the others, raw point cloud data for different scenarios.

__Figure 2: Directory Structure Tree__

```bash
.
├── build
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
    │   │   ├── build
    │   │   ├── CMakeLists.txt
    │   │   ├── cluster.cpp
    │   │   ├── cluster3d.cpp
    │   │   └── headers
    │   │       ├── clustering.h
    │   │       ├── kdtree.h
    │   │       └── kdtree3d.h
    │   └── ransac
    │       ├── build
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

### Rendering Options

<table>
    <thead>
        <tr>
            <th>Parameter</th>
            <th>Rationale</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><code>renderScene</code></td>
            <td><code>true</code> to display highway and cars</td>
        </tr>
        <tr>
            <td><code>renderLidarScans</code></td>
            <td><code>true</code> to render LiDAR scans</td>
        </tr>
        <tr>
            <td><code>filterPointCloud</code></td>
            <td><code>true</code> to downsample the point cloud using a voxel grid filter</td>
        </tr>
        <tr>
            <td><code>renderDataPoints</code></td>
            <td><code>true</code> to render colorless point cloud data points</td>
        </tr>
        <tr>
            <td><code>renderPlaneCloud</code></td>
            <td><code>true</code> to render inliers in green</td>
        </tr>
        <tr>
            <td><code>renderObstacleCloud</code></td>
            <td><code>true</code> to render obstacles (non-inliers) in red</td>
        </tr>
        <tr>
            <td><code>renderKdTree</code></td>
            <td><code>true</code> to render 3D KD-Tree in the Viewer (only recommended for Simple Highway)</td>
        </tr>
        <tr>
            <td><code>renderClusters</code></td>
            <td><code>true</code> to render Euclidean Clustering on obstacle data</td>
        </tr>
        <tr>
            <td><code>renderEgoCarBox</code></td>
            <td><code>true</code> to render a box of the approximate Ego Car location in magenta</td>
        </tr>
        <tr>
            <td><code>renderBoxes</code></td>
            <td><code>true</code> to render regular bounding boxes around obstacle data</td>
        </tr>
        <tr>
            <td><code>renderMinimumXyAlignedBoxes</code></td>
            <td><code>true</code> to render XY-plane-aligned minimum bounding boxes</td>
        </tr>
    </tbody>
</table>

### Voxel Grid

The original point cloud is filtered using voxel grid technique. A voxel (volumetric pixel) is a 3-dimensional [...]. The raw point cloud is subdivided into 3D cubes 20 centimeters in side, and all the points belonging to a particular cube are "approximated" by their _centroid_, a single point in 3D space which has as coordinates the means of all 3 coordinates of the enclosed points. Dimensionality reduction is thus achieved by ?? this cloud of points by the single centroid. 20 cm was found to be a good trade-off between dimensionality reduction and precision/details kept. Cloud achieved a ration of ~100k to ~5k points per frame. Dimension reduction is particularly important for self-driving cars or applications where fast processing of data is essential.

### Region Of Interest (ROI)

Region-based fitting consists, instead, in keeping only a certain section (rectangular prism) of the diving environment, discarding the edges of low significance for object detection and perception of the surroundings, and which introduce additional computational burden (they add little benefit in processing the area, add little information). Since LiDAR is mounted on top of the car (to be able to continuously rotate 360° and perceive the environment), in order to keep the road plane, Z <belongs> [-2m, 1m]. X, Y, instead, are approximately symmetric and in [ ; ] and [ ; ]. This way we detect the road, but no wall or parked cars.

<table>
  <tr>
  <td align="center"><b>Figure 3.A</b>: Raw (unfiltered) point cloud</td>
  <td align="center"><b>Figure 3.B</b>: Voxel grid (20 cm) and ROI -filtered point cloud</td>
  <tr>
  </tr>
  <tr>
    <td align="center"><img align="center" src="img/img2.png" width="475"/></td>
    <td align="center"><img align="center" src="img/img3.png" width="475"/></td>
  </tr>
</table>

### RANSAC

```math
\begin{align*}
&Ax + By + Cz + D = 0 \\
\\
&A = (y_2 - y_1)(z_3 - z_1) - (z_2 - z_1)(y_3 - y_1) \\
\\
&B = (z_2 - z_1)(x_3 - x_1) - (x_2 - x_1)(z_3 - z_1) \\
\\
&C = (x_2 - x_1)(y_3 - y_1) - (y_2 - y_1)(x_3 - x_1) \\
\\
&D = -(Ax_1 + By_1 + Cz_1) \\
\\
&distance = \frac{|Ax + By + Cz + D|}{\sqrt{A^2 + B^2 + C^2}}
\end{align*}
```

__Figure 3: RANSAC__
![RANSAC](./img/img4.png)

### Euclidean Clustering

__Figure 3: Euclidean Clustering__
![Euclidean Clustering](./img/img5.png)

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

<table>
  <tr>
  <td align="center"><b>Figure 3.A</b>: Regular Bounding Boxes</td>
  <td align="center"><b>Figure 3.B</b>: PCA Bounding Boxes</td>
  <tr>
  </tr>
  <tr>
    <td align="center"><img align="center" src="./img/mov2a.gif" width="475"/></td>
    <td align="center"><img align="center" src="./img/mov2b.gif" width="475"/></td>
  </tr>
</table>

1. https://en.wikipedia.org/wiki/Minimum_bounding_box_algorithms
2. Dimitrov, D., Knauer, C., Kriegel, K., Rote, G.: [On the Bounding Boxes Obtained by Principal Component Analysis](https://www.researchgate.net/publication/235758825_On_the_bounding_boxes_obtained_by_principal_component_analysis), 2014, ResearchGate.

[Home](../../README.md) | Next: 
