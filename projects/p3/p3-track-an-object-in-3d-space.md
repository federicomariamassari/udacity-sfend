[Home](../../README.md) | Previous: [Camera-Based 2D Feature Tracking](../p2/p2-camera-based-2d-feature-tracking.md) | Next: 

# Project 3: Track an Object in 3D Space

## Overview

Additionally, all tasks are timed to monitor their efficiency.

## Final Report

### FP.1: Match 3D Objects

The main reference for this task is [1]. To match bounding box pairs, I create a map object in which, for each element, the key will be a pair of bounding box indices (associated to the previous and current frames, respectively) and the value a counter of all occurrences of such key pair among the keypoint matches. For each match, after extracting the respective feature points in both frames [2], I first loop through all the bounding boxes in the query (previous) image to check which of them contains the related point; if a correspondence is found, I then run through all bounding boxes in the train (current) image to find the ones which also include the associated point. For those pairs where both conditions are met, the counter is incremented. Once the map is populated, the query-train index pairs for which the counter is largest (one for each query bounding box index) are kept. The object-matching logic is found in [`matchBoundingBoxes`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p3/src/camFusion_Student.cpp#L131) and called in the [main file](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p3/src/FinalProject_Camera.cpp#L221).

## FP.2: Compute LiDAR-based TTC

### Outliers Removal
Outlier detection and removal is provided in two flavours: Tukey's fences [3] and Euclidean clustering [4] [5].

## Resources

1. https://knowledge.udacity.com/questions/570553
2. https://docs.opencv.org/4.2.0/d4/de0/classcv_1_1DMatch.html
3. https://en.wikipedia.org/wiki/Outlier
4. https://knowledge.udacity.com/questions/296395
5. https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/src/custom/clustering.h
6. https://docs.opencv.org/4.2.0/db/d18/classcv_1_1flann_1_1GenericIndex.html

[Home](../../README.md) | Previous: [Camera-Based 2D Feature Tracking](../p2/p2-camera-based-2d-feature-tracking.md) | Next: 
