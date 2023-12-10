[Home](../../README.md) | Previous: [LiDAR Obstacle Detection](../p1/p1-lidar-obstacle-detection.md) | Next: 

# Project 2: Camera-Based 2D Feature Tracking

## Preliminary Considerations

This project was originally developed on a UTM QEMU 7.0 Virtual Machine running Ubuntu 20.04-5 LTS on Apple Silicon architecture (aarch64), and later ported to the Udacity workspace. It uses OpenCV 4.2.0, [built from source](https://github.com/federicomariamassari/udacity-rsend/blob/main/projects/p4/p4-preliminary-config.md#3-rebuild-opencv-from-source-with-patented-modules) to enable patented algorithms SIFT/SURF. For easier debugging and experimentation, the main parameters have been factored out in an [`Options` struct](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p2/src/MidTermProject_Camera_Student.cpp#L10) and can be changed for a range of outcomes.

__Figure 1: Directory Structure Tree__

```bash
.
├── build
│   ├── ...
│   └── 2D_feature_tracking
├── CMakeLists.txt
├── images
│   └── KITTI
│       └── 2011_09_26
│           └── image_00
│               └── data
│                   ├── 0000000000.png
│                   ├── ...
│                   └── 0000000009.png
├── analysis
│   └── Book1.xls
└── src
    ├── dataStructures.h
    ├── matching2D.hpp
    ├── matching2D_Student.cpp
    └── MidTermProject_Camera_Student.cpp
```

## Mid-Term Report

### MP.1: Data Buffer Optimization

A data ring buffer guarantees efficient memory management by limiting the number of images simultaneously present in the holding data structure, preventing the structure from growing excessively in size. Here, the buffer is a C++ vector with maximum size two; if the vector is full, we erase the earliest pushed-back image first (FIFO), then add the next element [1]. Time complexity is linear on the element erased and linear on moving the remaining element from position 1 to position 0 [2]; one can even say both operations occur in constant time, since the deleted object is always the first one with no search involved, and a one-place move is performed. The data buffer logic is encapsulated in a custom function [`loadBufferIntoFrame`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p2/src/matching2D_Student.cpp#L350), which is then [called in the main file](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p2/src/MidTermProject_Camera_Student.cpp#L141).

### MP.2: Keypoint Detection

Keypoint detection, description, matching, and selection are implemented in `matching2D_Student.cpp`. The Harris and Shi-Tomasi detectors have _ad hoc_ methods, while SIFT (and SURF), FAST, ORB, BRISK, and AKAZE are all included in `detKeypointsModern`. To provide a uniform interface, wrapper function [`detectKeypoints`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p2/src/matching2D_Student.cpp#L374) is also included and [called from `main`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p2/src/MidTermProject_Camera_Student.cpp#L149). Algorithm selection is performed via string comparison, using `std::string::compare` instead of `==` to both conform with the starter code and to allow for input case-insensitivity (source: Udacity GPT). Harris corner detection and non-maxima suppression are based on the Udacity solution [3]; for modern algorithms, which are plugged in the generic `cv::FeatureDetector` class, the main references are [4] and [5]. To improve readability, arguments are not explicitly passed to the function signatures if they use the default values; instead, reference to the official documentation is made in the code.

SURF is also included (but analysed separately), as it was the required method for project P4 [Map My World](https://github.com/federicomariamassari/udacity-rsend/blob/main/projects/p4/p4-map-my-world.md) of Udacity's Robotics Software Engineer Nanodegree and I was curious to understand how it would compare to the other algorithms.

### MP.3: Keypoint Removal

For this task, template class `cv::Rect` is used to remove all keypoints outside an area in pixels centered on the preceding vehicle (x=535, y=180, width=180, height=150). All keypoints whose coordinates belong to the rectangle are pushed back in a new vector, which is then reassigned to the original object. It is worth mentioning that the pre-defined area includes the side mirror of a vehicle on the left, as well as the shadow of the preceding car itself, with relevant implications for the analysis. The keypoint removal logic is placed in custom method [`focusOnArea`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p2/src/matching2D_Student.cpp#L396), which is then [called in the main file](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p2/src/MidTermProject_Camera_Student.cpp#L153).

### MP.4: Keypoint Descriptors

Descriptors SIFT (and SURF), BRIEF, ORB, FREAK, and AKAZE are implemented, with default arguments, from [4] and [5]. They complement the already available BRISK. Similarly to the detectors' case, the descriptors are plugged in the generic class `cv::DescriptorExtractor`, which provides a clean interface. Exceptions are raised in case of detector/descriptor incompatibilities, such as SIFT and ORB, or AKAZE and everything else [7].

### MP.5: Descriptor Matching

The main reference for matching is [8], with abstract class `cv::DescriptorMatcher` used as base for all matchers. Validation criteria are added to both Brute Force (part of the starter code) and FLANN: for Brute Force, by ensuring that the Hamming distance is only applied to binary algorithms, using instead the L2-norm (vector norm) for Histogram Of Gradients -based methods SIFT and SURF (having previously labelled them as such) [9]; for FLANN, by converting the input descriptor source and reference into 32-bit floating point numbers ahead of the processing step [10].

For the k-Nearest Neighbor selector, the main source is [11].

### MP.6: Description Distance Ratio

The implementation of the description distance ratio for k-Nearest Neighbors is taken from Udacity's solution to [12].

### MP.7: Performance Evaluation 1

__Figure 1: Distribution of Keypoints' Neighborhood Size__

| Metrics | Harris | Shi-Tomasi | FAST | BRISK | ORB | AKAZE | SIFT | SURF |
|:----------|------:|------:|------:|------:|------:|------:|------:|------:|
| Total no. of keypoints | 247 | 1179 | 2207 | 2762 | 1161 | 1670 | 1386 | 2150 |
| Keypoints/image (lower bound) | 24 | 117 | 220 | 276 | 116 | 167 | 138 | 215 |
| Avg keypoint detection time* (ms) | 5.13568 | 4.11741 | 0.862544 | 38.2281 | 4.34509 | 41.7808 | 40.887 | 17.0413 |
| Mean | 6 | 4 | 7 | 21.9444 | 55.9928 | 7.69915 | 5.03739 | 28.2967 |
| Median | 6 | 4 | 7 | 15.5217 | 44.64 | 5.70819 | 3.19932 | 21 |
| Mode | 6 | 4 | 7 | 72 | 111.079 | 22.8328 | 42.8487 | 128 |
| Standard deviation | 0 | 0 | 0 | 14.6207 | 25.1258 | 3.53831 | 5.98752 | 19.6608 |
| Min | 6 | 4 | 7 | 8.4 | 31 | 4.8 | 1.79669 | 10 |
| Max | 6 | 4 | 7 | 72 | 111.079 | 27.1529 | 51.7024 | 128 |
| Range | 0 | 0 | 0 | 63.6 | 80.0786 | 22.3529 | 49.9057 | 118 |
| 25th percentile | 6 | 4 | 7 | 12.3061 | 37.2 | 5.70819 | 2.1767 | 16 |
| 75th percentile | 6 | 4 | 7 | 27 | 77.1379 | 8.07261 | 4.83003 | 30 |
| IQR | 0 | 0 | 0 | 14.6939 | 39.9379 | 2.36441 | 2.65333 | 14 |

(*) Single experiment, [`bCompareDetectors = true`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p2/src/MidTermProject_Camera_Student.cpp#L21).

__Figure 1: FAST-BRIEF (Top), FAST-ORB (Middle), FAST-BRISK (Bottom) Keypoint Matching__
![FAST-BRIEF](./img/FAST_BRIEF_0007.png)
![FAST-ORB](./img/FAST_ORB_0007.png)
![FAST-BRISK](./img/FAST_BRISK_0007.png)

## Resources

1. https://knowledge.udacity.com/questions/644337
2. https://cplusplus.com/reference/vector/vector/erase/
3. Harris Corner Detection, Tracking Image Features Lesson 5, Udacity Sensor Fusion Nanodegree
4. https://docs.opencv.org/4.2.0/d5/d51/group__features2d__main.html
5. https://docs.opencv.org/4.2.0/d2/dca/group__xfeatures2d__nonfree.html
6. https://docs.opencv.org/4.2.0/d2/d44/classcv_1_1Rect__.html
7. https://knowledge.udacity.com/questions/105392
8. https://docs.opencv.org/4.2.0/d8/d9b/group__features2d__match.html
9. https://docs.opencv.org/4.2.0/d3/da1/classcv_1_1BFMatcher.html
10. https://knowledge.udacity.com/questions/211123
11. https://docs.opencv.org/4.2.0/db/d39/classcv_1_1DescriptorMatcher.html
12. Exercise - Descriptor Matching, Tracking Image Features Lesson 12, Udacity Sensor Fusion Nanodegree

[Home](../../README.md) | Previous: [LiDAR Obstacle Detection](../p1/p1-lidar-obstacle-detection.md) | Next: 