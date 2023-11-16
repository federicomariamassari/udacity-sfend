[Home](../../README.md) | Previous: [LiDAR Obstacle Detection](../p1/p1-lidar-obstacle-detection.md) | Next: 

# Project 2: Camera-Based 2D Feature Tracking

## Preliminary Considerations

This project was originally developed on a UTM QEMU 7.0 Virtual Machine running Ubuntu 20.04-5 LTS on Apple Silicon architecture, and later ported to the Udacity workspace. It uses OpenCV 4.2.0, [built from source](https://github.com/federicomariamassari/udacity-rsend/blob/main/projects/p4/p4-preliminary-config.md#3-rebuild-opencv-from-source-with-patented-modules) to enable patented algorithms SIFT/SURF.

## Mid-Term Report

### MP.1: Data Buffer Optimization

A data ring buffer guarantees efficient memory management by limiting the number of images simultaneously present in the holding data structure, preventing the structure from growing excessively in size. Here, the buffer is a C++ vector with maximum size two; if the vector is full, we erase the earliest pushed-back image first (FIFO), then add the next element [1]. Time complexity is linear on the element erased (basically constant, since it's always the first one and no search is involved), and linear on moving the remaining element to position 0 [2].

### MP.2: Keypoint Detection

Keypoint detection, description, matching, and selection are implemented in `matching2D_Student.cpp`. The Harris and Shi-Tomasi detectors have _ad hoc_ methods, while SIFT (and SURF), FAST, ORB, BRISK, and AKAZE are all included in `detKeypointsModern`. Algorithm selection is performed via string comparison, using `std::string::compare` instead of `==` to both conform with the starter code and to allow for input case-insensitivity (source: Udacity GPT). Harris corner detection and non-maxima suppression are based on the Udacity solution [3]; for modern algorithms, which are plugged in the generic `cv::FeatureDetector` class, the main references are [4] and [5]. To improve readability, arguments are not explicitly passed to the function signatures if they use the default values; instead, reference to the official documentation is made in the code.

SURF is also included (but analysed separately) as it was the required method for project P4 [Map My World](https://github.com/federicomariamassari/udacity-rsend/blob/main/projects/p4/p4-map-my-world.md) of Udacity's Robotics Software Engineer Nanodegree, and I was curious to understand how it would fare against the competition.

### MP.3: Keypoint Removal

For this task, template class `cv::Rect` is used to remove all keypoints outside an area in pixels centered on the preceding vehicle (x=535, y=180, width=180, height=150). All keypoints whose coordinates belong to the rectangle are pushed back in a new vector, which is then reassigned to the original object. It is worth mentioning that the pre-defined area includes the side mirror of a vehicle on the left, as well as the shadow of the preceding car itself, with implications for the analysis.

### MP.4: Keypoint Descriptors

Descriptors SIFT (and SURF), BRIEF, ORB, FREAK, and AKAZE are implemented, with default arguments, from [4] and [5]. They complement the already available BRISK. Similarly to the detectors' case, the descriptors are plugged in the generic class `cv::DescriptorExtractor`, which provides a clean interface. Exceptions are raised in case of detector/descriptor incompatibilities, such as SIFT and ORB, or AKAZE and everything else [7].

### MP.5: Descriptor Matching

The main reference for matching is [8], with abstract class `cv::DescriptorMatcher` used as base for all matchers. Validation criteria are added to both Brute Force (part of the starter code) and FLANN: for Brute Force, by ensuring that the Hamming distance is only applied to binary algorithms, using instead the L2-norm (vector norm) for Histogram Of Gradients -based methods SIFT and SURF (having previously labelled them as such) [9]; for FLANN, by converting the input descriptor source and reference into 32-bit floating point numbers ahead of the processing step [10].

### MP.6: Description Distance Ratio

The implementation of the description distance ratio for k-Nearest Neighbors is taken from Udacity' solution to [11].

### MP.7: Performance Evaluation 1

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
11. Exercise - Descriptor Matching, Tracking Image Features Lesson 12, Udacity Sensor Fusion Nanodegree

[Home](../../README.md) | Previous: [LiDAR Obstacle Detection](../p1/p1-lidar-obstacle-detection.md) | Next: 