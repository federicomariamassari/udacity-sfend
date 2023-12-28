# Udacity Sensor Fusion Engineer Nanodegree

My Udacity Sensor Fusion Engineer Nanodegree projects, in C++.

# Core Projects

## Environment

* Ubuntu 20.04-5 LTS Focal Fossa running on UTM Virtual Machine on MacBook Pro M1 Max (aarch64)
* Point Cloud Library 1.11 ([Built from Source](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html#stable))

## [Project 1: LiDAR Obstacle Detection](projects/p1/p1-lidar-obstacle-detection.md)

__Acquired familiarity with:__ Point Cloud Library (PCL).

### Overview

_Filter, segment, and cluster raw LiDAR data to detect vehicles and obstacles on the road._

In this assignment I learn how to process point clouds from LiDAR scans in order to identify vehicles and other obstacles in a driving environment. I first reduce cloud size using voxel (volumetric pixel) grid and region of interest techniques, then separate road and obstacles via RANSAC (RANdom SAmple Consensus), group points belonging to the same objects using Euclidean clustering with 3-dimensional KD-Trees, and finally enclose the obtained clusters within either regular or minimum (PCA-based) bounding boxes.

__[Link to code](projects/p1/src)__ | [Starter Code from Udacity](https://github.com/udacity/SFND_Lidar_Obstacle_Detection)

<table>
  <tr>
    <td align="center"><img align="center" src="projects/p1/img/img2a.png" width="500"/></td>
    <td align="center"><img align="center" src="projects/p1/img/img1.png" width="500"/></td>
  </tr>
</table>

### How to Build and Run the Project

Clone the repository locally, for example inside `/home/$whoami/workspace` (with `$whoami` the username of the current user). Ensure PCL and associated Viewer are [installed correctly](projects/p1/p1-preliminary-configs.md), then build and run the main project as per below commands. To build and run `quiz` instead, see the project's README file.

```bash
cd /home/$whoami/workspace/udacity-rsend/projects/p1
mkdir build && cd build
cmake ..
make
./environment
```

### Output

A stream of incoming obstacles, encapsulated in PCA bounding boxes, is rendered in the city block scene below. Udacity's self-driving vehicle Carla is the purple block at the center of the screen, with LiDAR mounted on top.

![PCA Bounding Boxes](./projects/p1/img/mov1.gif)

## [Project 2: Camera-Based 2D Feature Tracking](projects/p2/p2-camera-based-2d-feature-tracking.md)

__Acquired familiarity with:__ OpenCV 4.x, Gnumeric.

### Overview

_Learn to detect, describe, and match features in 2D camera images._

In this computer vision application, I implement a two-dimensional feature tracking algorithm to monitor objects in a sequence of images using OpenCV. After progressively loading the pictures in a data ring buffer, I use classic and modern techniques to detect keypoints, calculate their descriptors, match the features between consecutive frames, and finally evaluate the performance of each detector-descriptor combination in terms of speed and accuracy.

__[Link to code](projects/p2/src)__ | [Starter Code from Udacity](https://github.com/udacity/SFND_2D_Feature_Tracking)

![SIFT Keypoint Detection](./projects/p2/img/mov1.gif)

### How to Build and Run the Project

As a prerequisite, build OpenCV 4.2.0 [from source](https://github.com/federicomariamassari/udacity-rsend/blob/main/projects/p4/p4-preliminary-config.md#3-rebuild-opencv-from-source-with-patented-modules) to enable patented algorithms SIFT and SURF. Then build and run as follows:

```bash
cd /home/$whoami/workspace/udacity-rsend/projects/p2
mkdir build && cd build
cmake ..
make
./2D_feature_tracking
```

### Output

Set both `bCompareDetectors` and `bCompareDescriptors` to `false` to visualize matched keypoints among image pairs for the chosen detector and descriptor. Set either (or both) to `true` to output tabulated statistics on the distribution of keypoints' neighborhood size (MP.7) and/or comparisons among detector-descriptor combinations (MP.8-9).

## [Project 3: Track an Object in 3D Space](projects/p3/p3-track-an-object-in-3d-space.md)

### Overview

__[Link to code](projects/p3/src)__ | [Starter Code from Udacity](https://github.com/udacity/SFND_3D_Object_Tracking)
