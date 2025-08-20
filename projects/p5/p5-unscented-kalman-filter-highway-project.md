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

Post initialisation, the algorithm enters a state prediction and measurement update cycle. __State prediction__ estimates the future state of a tracked object by incorporating its most recent state and the time elapsed since the last measurement (by default, [1/3 of a second](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/main.cpp#L40-L47)) into a Constant Turn Rate and Velocity Magnitude (CTRV) process model. __Measurement update__ combines the above prediction with a new measurement (initially from LiDAR) to provide an updated, more accurate representation of the objects' location, assigning greater weight to the component (either state prediction or measurement update) with the lowest uncertainty.

### State Prediction

State prediction is implemented in [`UKF::Prediction()`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/ukf.cpp#L165-L175) and, at each time step $k$, consists of three stages:

1. __Augmented Sigma Point Generation:__ A set of augmented sigma points is created based on the current posterior state vector $x_{k|k}$ and covariance matrix $P_{k|k}$, incorporating process noise $\nu_k$. The augmentation ensures that both the system state and process uncertainties are accurately represented.
2. __Sigma Point Propagation:__ The augmented sigma points are propagated forward in time ($k+1$) using the CTRV motion model. This step forecasts how the system state will evolve, effectively capturing the non-linear dynamics of vehicle motion.
3. __State and Covariance Prediction:__ The predicted sigma points are used to compute the _a priori_ state vector $x_{k+1|k}$ and covariance matrix $P_{k+1|k}$. This yields the best estimate of the system state and its uncertainty prior to incorporating the next sensor measurement.

### Measurement Update

In the measurement update phase, the state estimate is refined by integrating the new measurement data from the sensor. The process involves three steps:

1. __Predicted State to Measurement Space Transformation:__ Each predicted sigma point is converted from the state space into the measurement space. This mapping, which allows to capture the uncertainty associated with the measurement, [varies based on the sensor](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/ukf.cpp#L147-L162) producing the data: __radar__ measurements are non-linear and provide data in _polar_ coordinates; by contrast, __LiDAR__ ones are more linear, producing distances in the _Cartesian_ plane.
2. __Predicted Measurement Mean and Covariance Calculation:__ The predicted measurement state vector $z_{k+1|k}$ is calculated as the weighted average of the transformed sigma points, while the predicted measurement covariance $S_{k+1|k}$ captures the spread of these points.
3. __Prediction and New Measurement Combination:__ The final step involves combining the predicted measurement with a new sensor measurement. This includes calculating the Kálmán gain, which determines the weight assigned to each quantity, resulting in a more accurate update of the object's state, $z_{k+1|k+1}$.

The implementation for LiDAR (which relies on a simple Kálmán filter due to its inherent linearity) and radar (which relies on the Unscented Kálmán filter) is provided, respectively, in [`UKF::UpdateLidar`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/ukf.cpp#L177-L204) and [`UKF::UpdateRadar`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/ukf.cpp#L206-L213).

## Performance Assessment

A metric for the accuracy of the implemented filter is the Root Mean Square Error (RMSE) [2], calculated for positions $p_x$, $p_y$, and velocity values $v_x$, $v_y$, and displayed at the bottom-left corner of the simulation screen. The algorithm is defined in the [`Tools::CalculateRMSE`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/tools.cpp#L128-L160) function and is called within the project's [main class](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/highway.h#L230-L279). The project is deemed successful if, after one second of simulation, the cumulative RMSE values do not exceed the designated threshold levels (i.e., $p_x$ <= 0.30, $p_y$ <= 0.16, $v_x$ <= 0.95, and $v_y$ <= 0.70). The task is manageable if the chosen anchor points for LiDAR are the [center points of the stylised blue cars](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/tools.cpp#L23), which are fixed in shape [Figure 1]. However, if the cars are replaced by their corresponding (box-bound) point cloud cluster scans the situation becomes more complex. Due to LiDAR's occasional inability to capture the full shape of a vehicle, particularly when the target is in the same lane as ego car, the RMSE threshold for $p_x$ is often exceeded, and the error accumulates the longer this situation persists [Figure 2]. Tracking the centroids of the point clouds, rather than the [center point of the enclosing bounding box](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/tools.cpp#L43-L47), does not alleviate the issue, as both approaches are affected by the same challenge of point cloud incompleteness.

__Figure 2: RMSE LiDAR Measurement X-Threshold Breach__
![RMSE X-dimension breach](./img/mov4.gif)

## Resources

1. https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/p1-preliminary-configs.md
2. [Wikipedia: Root mean square deviation](https://en.wikipedia.org/wiki/Root_mean_square_deviation)
3. [Udacity Knowledge: Under which conditions is it required to pass the RMSE thresholds?](https://knowledge.udacity.com/questions/1036196)

[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md) | Next: [Embedded Input Reader](https://github.com/federicomariamassari/udacity-esfnd/blob/main/projects/p1/p1-embedded-input-reader.md)