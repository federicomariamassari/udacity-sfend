[Home](../../README.md) | Previous: [LiDAR Obstacle Detection](../p1/p1-lidar-obstacle-detection.md) | Next: 

# Project 2: Camera-Based 2D Feature Tracking

## Preliminary Considerations

This project was originally developed on a UTM QEMU 7.0 Virtual Machine running Ubuntu 20.04-5 LTS on Apple Silicon architecture, and later ported to the Udacity workspace. It uses OpenCV 4.2.0, [built from source](https://github.com/federicomariamassari/udacity-rsend/blob/main/projects/p4/p4-preliminary-config.md#3-rebuild-opencv-from-source-with-patented-modules) to enable patented algorithms SIFT/SURF.

## Mid-Term Report

### MP.1 Data Buffer Optimization

A data ring buffer guarantees efficient memory management by limiting the number of images simultaneously present in the holding data structure, preventing the structure from growing excessively in size. Here, the buffer is a C++ vector with maximum size two; if the vector is full, we erase the earliest pushed-back image first (FIFO), then add the next element [1]. Time complexity is linear on the element erased (basically constant, since it's always the first one and no search is involved), and linear on moving the remaining element to position 0 [2].

### MP.2 Keypoint Detection

Keypoint detection, description, matching, and selection are implemented in `matching2D_Student.cpp`. The Harris and Shi-Tomasi detectors have _ad hoc_ methods, while SIFT (and SURF), FAST, ORB, BRISK, and AKAZE are all included in `detKeypointsModern`. Algorithm selection is performed via string comparison, using `std::string::compare` instead of `==` to both conform with the starter code and to allow for input case-insensitivity (source: Udacity GPT). Harris corner detection and non-maxima suppression are based on the Udacity solution [3]; for modern algorithms, which are plugged into the generic `cv::FeatureDetector` class, the main references are [4] and [5]. To improve readability, arguments are not explicitly passed to the function signatures if they use the default values. Instead, reference to the official documentation is provided in the code.

## Resources

1. https://knowledge.udacity.com/questions/644337
2. https://cplusplus.com/reference/vector/vector/erase/
3. Harris Corner Detection, Tracking Image Features Lesson 5, Udacity Sensor Fusion Nanodegree
4. https://docs.opencv.org/4.2.0/d5/d51/group__features2d__main.html
5. https://docs.opencv.org/4.2.0/d2/dca/group__xfeatures2d__nonfree.html

[Home](../../README.md) | Previous: [LiDAR Obstacle Detection](../p1/p1-lidar-obstacle-detection.md) | Next: 