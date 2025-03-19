[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md) | Next: [Embedded Input Reader](https://github.com/federicomariamassari/udacity-esfnd/blob/main/projects/p1/p1-embedded-input-reader.md)

# Project 5: Unscented Kálmán Filter Highway Project

## Overview

In the capstone project of the Sensor Fusion Nanodegree, I implement an Unscented Kálmán Filter (UKF) algorithm to estimate the state of multiple target cars on a simulated highway, fusing noisy measurements from LiDAR and radar. For its predict-update cycle, the Unscented Kálmán Filter relies on a set of carefully-selected sigma points (representative points from a Gaussian distribution) which capture the mean and covariance matrix of the distribution of state variables, so it provides an accurate estimation of position and velocity in highly non-linear systems. The motion model used to predict the targets' state between sensor measurements is Constant Turn Rate and Velocity Magnitude (CTRV), which accounts for the natural turning and acceleration or deceleration behaviour of vehicles on a highway [Figure 1].

__Figure 1: UKF-CTRV Non-Linear Vehicle Tracking__
![UKF XY Output](./img/mov3.gif)

## Project Structure

The directory structure tree for the project appears in Figure 2. In particular:

- `highway.h` includes the environment rendering logic, the project options, calls to the point cloud processing functions, and the Root Mean Squared Error (RMSE) tests;
- `process_point_cloud.h`, a custom extension for PCD files, adds methods for voxel grid downsampling, Euclidean clustering, and bounding box encapsulation;
- `tools.h` contains auxiliary methods to display LiDAR and radar markers, inject random noise into simulated sensor measurements, and calculate RMSE, among the others;
- `ukf.h` holds the Unscented Kálmán Filter algorithm (prediction and update steps for both LiDAR and radar).

__Figure 2: Directory Structure Tree__

```bash
.
├── build
│   ├── ...
│   └── ukf_highway
├── CMakeLists.txt
└── src
    ├── custom
    │   ├── process_point_cloud.cpp
    │   └── process_point_cloud.h
    ├── highway.h
    ├── main.cpp
    ├── measurement_package.h
    ├── render
    │   ├── box.h
    │   ├── render.cpp
    │   └── render.h
    ├── sensors
    │   ├── data
    │   │   └── pcd
    │   │       ├── highway_0.pcd
    │   │       ├── ...
    │   │       └── highway_9966666.pcd
    │   └── lidar.h
    ├── tools.cpp
    ├── tools.h
    ├── ukf.cpp
    └── ukf.h
```

## Building and Running the Project

### Preliminary Configurations

This project was developed and tested on Ubuntu 20.04 (UTM VM QEMU 7.0 aarch64). Minor configuration changes were made:

1. Updated the starter `CMakeLists.txt` to compile with C++14 and PCL 1.11 rather than with the default C++11 and PCL 1.2
2. Disabled GPU acceleration [1] to correctly display `pcl::Visualization` point size property when point cloud data are used

### Build and Run

```bash
cd /home/$whoami/workspace/udacity-rsend/projects/p5
mkdir build && cd build
cmake ..
make
./ukf_highway
```

### Options

Project options can be set in [`highway.h`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/highway.h#L17-L45).

<table>
    <thead>
        <tr>
            <th>Type</th>
            <th>Parameter</th>
            <th>Default value</th>
            <th>Explanation</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td rowspan=1><b>Camera initialisation options</b></td>
            <td><code>setAngle</code></td>
            <td><code>XY</code></td>
            <td><code>XY</code> for lateral camera view, <code>Default</code> for starter code angle</td>
        </tr>
        <tr>
            <td rowspan=1><b>Target selection options</b></td>
            <td><code>trackCars</code></td>
            <td><code>{true, true, true}</code></td>
            <td><code>true</code> for each car to track as target (SW, NE, S)</td>
        </tr>
        <tr>
            <td rowspan=3><b>Sensor measurements visualisation options</b></td>
            <td><code>visualize_lidar</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to display LiDAR pings (red orbs) at the top of target vehicles</td>
        </tr>
        <tr>
            <td><code>visualize_radar</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to display magenta radar arrows tracking target vehicles</td>
        </tr>
        <tr>
            <td><code>visualize_pcd</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to display colorless LiDAR point clouds, <code>false</code> for stylised green car shapes</td>
        </tr>
        <tr>
            <td rowspan=2><b>UKF path prediction options</b></td>
            <td><code>projectedTime</code></td>
            <td>2</td>
            <td>Number of seconds into the future to predict the CTRV trajectory of targets</td>
        </tr>
        <tr>
            <td><code>projectedSteps</code></td>
            <td>6</td>
            <td>Number of green orbs to display at the top of target vehicles (+1 to describe the current state)</td>
        </tr>
        <tr>
            <td rowspan=2><b>Cloud filtering options</b></td>
            <td><code>filterPointCloud</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to downsample input point cloud using voxel grid filtering</td>
        </tr>
        <tr>
            <td><code>voxelSide</code></td>
            <td>0.25f</td>
            <td>Side length of the voxel cube (0.01f = 1 cm)</td>
        </tr>
        <tr>
            <td rowspan=4><b>Euclidean clustering options</b></td>
            <td><code>cluster_pcd</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to render Euclidean Clustering on obstacle data (rotating colors among red, yellow, blue)</td>
        </tr>
        <tr>
            <td><code>clusterTol</code></td>
            <td>1.2</td>
            <td>Maximum tolerance to keep points within a cluster (in meters)</td>
        </tr>
        <tr>
            <td><code>minSize</code></td>
            <td>50</td>
            <td>Minimum number of points in a cluster</td>
        </tr>
        <tr>
            <td><code>maxSize</code></td>
            <td>1000</td>
            <td>Maximum number of points in a cluster</td>
        </tr>
        <tr>
            <td rowspan=1><b>Rendering options</b></td>
            <td><code>renderBoxes</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to render axis-aligned bounding boxes around the clusters</td>
        </tr>
    </tbody>
</table>

## Code Logic

### Initialization

State vector $\bf{x}$ and state covariance matrix $\bf{P}$ are initialised in [`UKF::ProcessMeasurement()`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/ukf.cpp#L78), and their values depend on the [type of marker](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/tools.h#L17-L33) received (LiDAR or radar). Laser data contain info on the object's position $(p_x, p_y)$ while radar data contain info on radial distance, bearing, and range rate $(\rho, \phi, \dot{\rho})$. In this project, the state estimates are initialised with a LiDAR measurement, which is [always first by design](https://github.com/federicomariamassari/udacity-sfend/blob/92eb89f7cd22dce75865f76b58bdcb1f306a52e0/projects/p5/src/highway.h#L205-L212).

### Predict-Update Cycle

Once initialisation is complete, the algorithm enters a state prediction and measurement update cycle. __State prediction__ infers the current state of the objects on the road by incorporating the time elapsed since last measurement (by default, [1/3 of a second](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/main.cpp#L40-L47)) into a Constant Turn Rate and Velocity Magnitude (CTRV) model. __Measurement update__, instead, combines

__Figure 2: RMSE LiDAR Measurement X-Threshold Breach__
![RMSE X-dimension breach](./img/mov4.gif)

## Resources

1. https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/p1-preliminary-configs.md

[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md) | Next: [Embedded Input Reader](https://github.com/federicomariamassari/udacity-esfnd/blob/main/projects/p1/p1-embedded-input-reader.md)