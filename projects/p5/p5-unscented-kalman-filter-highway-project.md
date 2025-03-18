[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md)

# Project 5: Unscented Kálmán Filter Highway Project

## Overview

## Building and Running the Project

### Preliminary Configurations

To build and run Udacity's starter code on Ubuntu 20.04 (UTM VM QEMU 7.0 aarch64), I updated the project's `CMakeLists.txt` as such:

1. I compiled using C++14 instead of the default C++11:

```c
add_definitions(-std=c++14)
```

2. To bypass a well-known assertion error [1], I included the following preprocessor directive [2], before adding the executables:

```c
add_definitions(-DEIGEN_MAX_STATIC_ALIGN_BYTES=0)
```

Simply switching to C++17, as [1] suggests, did not resolve the error because required dependency Point Cloud Library had already been built from source with C++14.

## Code Logic

### Initialization

<<<<<<< Updated upstream
State vector $\bf{x}$ and state covariance matrix $\bf{P}$ are initialized in [`UKF::ProcessMeasurement()`](), and their values depend on the [type of marker]() received (LiDAR or radar). Laser data contain info on the object's position $(p_x, p_y)$ while radar data contain info on radial distance, bearing, and range rate $(\rho, \phi, \dot{\rho})$.
=======
State vector $\bf{x}$ and state covariance matrix $\bf{P}$ are initialized in [`UKF::ProcessMeasurement()`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/ukf.cpp#L78), and their values depend on the [type of marker](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/tools.h#L17-L33) received (LiDAR or radar). Laser data contain info on the object's position $(p_x, p_y)$ while radar data contain info on radial distance, bearing, and range rate $(\rho, \phi, \dot{\rho})$. In this project, the state estimates are initialized with LiDAR measurement, which is [always first by design](https://github.com/federicomariamassari/udacity-sfend/blob/92eb89f7cd22dce75865f76b58bdcb1f306a52e0/projects/p5/src/highway.h#L205-L212).

__Figure 2: RMSE LiDAR Measurement X-Threshold Breach__
![RMSE X-dimension breach](./img/mov4.gif)
>>>>>>> Stashed changes

## Resources

1. https://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
2. https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html

[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md)