[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md) | Next: [Embedded Input Reader](https://github.com/federicomariamassari/udacity-esfnd/blob/main/projects/p1/p1-embedded-input-reader.md)

# Project 5: Unscented Kálmán Filter Highway Project

## Overview

In the capstone project of the Sensor Fusion Nanodegree, 

![UKF XY Output](./img/mov3.gif)

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

## Code Logic

### Initialization

State vector $\bf{x}$ and state covariance matrix $\bf{P}$ are initialized in [`UKF::ProcessMeasurement()`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/ukf.cpp#L78), and their values depend on the [type of marker](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p5/src/tools.h#L17-L33) received (LiDAR or radar). Laser data contain info on the object's position $(p_x, p_y)$ while radar data contain info on radial distance, bearing, and range rate $(\rho, \phi, \dot{\rho})$.

## Resources

1. https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/p1-preliminary-configs.md

[Home](../../README.md) | Previous: [Radar Target Generation and Detection](../p4/p4-radar-target-generation-and-detection.md) | Next: [Embedded Input Reader](https://github.com/federicomariamassari/udacity-esfnd/blob/main/projects/p1/p1-embedded-input-reader.md)