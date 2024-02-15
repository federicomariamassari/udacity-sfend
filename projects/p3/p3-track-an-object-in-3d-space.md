[Home](../../README.md) | Previous: [Camera-Based 2D Feature Tracking](../p2/p2-camera-based-2d-feature-tracking.md) | Next: 

# Project 3: Track an Object in 3D Space

## Overview

Additionally, all tasks are timed to monitor their efficiency.

## Final Report

### FP.1: Match 3D Objects

The main reference for this task is [1]. To match bounding box pairs, I create a map object in which, for each element, the key will be a pair of bounding box indices (associated to the previous and current frames, respectively) and the value a counter of all occurrences of such key pair among the keypoint matches. For each match, after extracting the respective feature points in both frames [2], I first loop through all the bounding boxes in the query (previous) image to check which of them contains the related point; if a correspondence is found, I then run through all bounding boxes in the train (current) image to find the ones which also include the associated point. For those pairs where both conditions are met, the counter is incremented. The default overlap threshold value of 0.4 for the YOLOv3 non-maxima suppression algorithm does not guarantee a keypoint is contained in one and only one bounding box, so I avoid early termination once a correspondence is found despite the increase in computational time.

Once the map is populated, the query-train index pairs for which the counter is largest (one for each query bounding box index) are kept. Many-to-one cases in which multiple previous bounding boxes are associated to the same current one are not removed, as they do not seem to distort the analysis (for example, they are not related to cases of "spurious bounding boxes" [3] mentioned further below). The object-matching logic is found in [`matchBoundingBoxes`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p3/src/camFusion_Student.cpp#L131) and called from the [main file](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p3/src/FinalProject_Camera.cpp#L221).

### FP.2: Compute LiDAR-based TTC

LiDAR time-to-collision logic is handled by [`computeTTCLidar`](). Extreme or otherwise unreliable data are discarded using one of the available filtering methods as input to function [`removeOutliers`](). TTC is then given by one of the below formulae (CVM, CAM), where $\tilde{x}$ is the median of all x-coordinates of the usable points (a proxy for distance $d$):

$$
\text{TTC}_ {\text{CVM}} = d_1 \times \frac{\Delta t}{d_0 - d_1} = \tilde{x}_{\text{curr}} \times \frac{\Delta t}{\tilde{x} _{\text{prev}} - \tilde{x} _{\text{curr}}}
$$

### Outlier Removal

Outlier detection and removal is provided in two flavours: Tukey's fences [4] and Euclidean clustering [5] [6]. Each option can be selected from the [`FilteringMethod`]() enum class. Custom rendering function [`renderClusters`](), valid for both methodologies, is also included for debugging and exploratory purposes, and can be enabled by setting `bRenderClusters = true`. (Colorless) outliers can be displayed with `bShowRemoved = true`.

#### Tukey's Fences

This option filters out as outliers all points whose x-coordinate (measuring the distance between ego and the preceding vehicle) is outside the interval $[Q_1 - 1.5\times IQR; Q_3 + 1.5\times IQR]$, where $Q_1$ and $Q_3$ are, respectively, the first and third quartiles (25th and 75th percentiles), computed via custom function [`percentile`](), and $IQR = Q_3 - Q_1$ is the interquartile range. This method produces a stable time-to-collision estimate fast and effectively, and is set as the default option.

#### Euclidean Clustering

An alternative option, which also considers dimensions $y$ and $z$ in the outlier detection phase, is Euclidean clustering [5]. The main reference for implementation (via Point Cloud Library instead of OpenCV), is [6]. Instead of choosing the cluster with the largest number of points to compute TTC, as [5] suggests, I just remove those clusters whose size is smaller than a predefined threshold `minSize` (main options) and use the remainder. Euclidean clustering is considerably slower than Tukey's fences (~10x with default values), hence harder to justify, in the current implementation, for real-time applications. The below parameters play a role in fine-tuning the algorithm:

| Parameter  | Default value | Explanation |
| :--------- | :------------ | :-----------|
| `knn`      | 5             | Number $k$ of neighbors to include at each radius search. Set $k > 3$ to ensure at least one new point is considered at each iteration (there will be duplicates), but not too large to avoid excessive increase in computational time. |
| `radius`   | 0.12          | Distance tolerance to query point for the neighborhood search. This value will be squared (L2-norm) [7]. |
| `minSize`  | 15            | Minimum cluster size. Clusters smaller than this threshold will be discarded as outliers. |
| `maxSize`  | 600           | Maximum cluster size. Clusters larger than this value will be broken down into smaller ones. |

### FP.3: Associate Keypoint Correspondences with Bounding Boxes

To establish a connection between the YOLOv3 bounding boxes and the enclosed keypoints, I proceed as follows. For each match, I extract the keypoint descriptors for both the previous and the current frame; if the region of interest (ROI) of the latter [8] contains the associated feature, I compute the Euclidean distance (L2-norm) between the train and the query keypoints and preliminary push it back to a vector. As we expect a rigid transformation of the preceding vehicle [9] given the focus on a straight ego lane, I then remove all enclosed matches whose distance exceeds 1.5 times the median (the mean is not a robust enough estimator). The remaining correspondences, both matches and keypoints, are finally assigned back to their respective `BoundingBox` attributes.

## Resources

1. https://knowledge.udacity.com/questions/570553
2. https://docs.opencv.org/4.2.0/d4/de0/classcv_1_1DMatch.html
3. https://knowledge.udacity.com/questions/1026363
4. https://en.wikipedia.org/wiki/Outlier
5. https://knowledge.udacity.com/questions/296395
6. https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p1/src/custom/clustering.h
7. https://docs.opencv.org/4.2.0/db/d18/classcv_1_1flann_1_1GenericIndex.html
8. https://knowledge.udacity.com/questions/110934
9. https://knowledge.udacity.com/questions/624666

[Home](../../README.md) | Previous: [Camera-Based 2D Feature Tracking](../p2/p2-camera-based-2d-feature-tracking.md) | Next: 
