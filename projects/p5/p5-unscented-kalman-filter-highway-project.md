[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md)

# Project 5: Unscented Kálmán Filter Highway Project

## Overview

![UKF XY Output](./img/mov3.gif)

## Building and Running the Project

### Preliminary Configurations

This project was developed and tested on Ubuntu 20.04 (UTM VM QEMU 7.0 aarch64). Minor tweaks were needed:

1. Updating the starter `CMakeLists.txt` to compile with C++14 and PCL 1.11 rather than with the default C++11 and PCL 1.2
2. Disabling GPU acceleration [1] to correctly display `pcl::Visualization` point size property when point cloud data are used

## Code Logic

### Initialization

State vector $\bf{x}$ and state covariance matrix $\bf{P}$ are initialized in [`UKF::ProcessMeasurement()`](), and their values depend on the [type of marker]() received (LiDAR or radar). Laser data contain info on the object's position $(p_x, p_y)$ while radar data contain info on radial distance, bearing, and range rate $(\rho, \phi, \dot{\rho})$.

## Resources

1. https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/p1-preliminary-configs.md
2. https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html

[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md)