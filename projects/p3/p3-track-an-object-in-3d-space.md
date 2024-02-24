[Home](../../README.md) | Previous: [Camera-Based 2D Feature Tracking](../p2/p2-camera-based-2d-feature-tracking.md) | Next: 

# Project 3: Track an Object in 3D Space

## Overview

The final project of the camera course involves fusing data from LiDAR and camera sensors to reliably estimate time-to-collision (TTC) with a vehicle in front, in the context of a Collision Detection System (CDS). After detecting and classifying the objects on the road using deep-learning framework YOLOv3 (You Only Look Once) [Figure 1], I identify the LiDAR points and keypoint matches that fall inside the region of interest (ROI) of the preceding vehicle's 2D bounding box across consecutive frame pairs, and use these to compute robust TTC estimates for both LiDAR and camera. All tasks are timed to monitor their efficiency.

__Figure 1: YOLOv3 2D Bounding Boxes__

![YOLOv3 Bounding Boxes](./img/mov4.gif)

## Project Structure

The directory structure tree for the project appears in Figure 2. In particular:

- `src` includes main file `FinalProject_Camera.cpp` (executable: `3D_object_tracking`) and `camFusion_Student.cpp`, which contains the logic for 3D object tracking and TTC computation;
- `dat` holds pre-trained YOLOv3 config files, weights, and COCO dataset class names;
- `images` has input camera frames and LiDAR point cloud binaries;
- `analysis` contains a spreadsheet with output statistics on LiDAR and camera-based TTC combinations (FP.5, FP.6).

__Figure 2: Directory Structure Tree__

```bash
.
├── analysis
│   └── p3_performance_evaluation.xls
├── build
│   ├── ...
│   └── 3D_object_tracking
├── CMakeLists.txt
├── dat
│   └── yolo
│       ├── coco.names
│       ├── yolov3.cfg
│       └── yolov3.weights
├── images
│   └── KITTI
│       └── 2011_09_26
│           ├── image_02
│           │   └── data
│           │       ├── 0000000000.png
│           │       ├── ...
│           │       └── 0000000077.png
│           └── velodyne_points
│               └── data
│                   ├── 0000000000.bin
│                   ├── ...
│                   └── 0000000077.bin
└── src
    ├── camFusion.hpp
    ├── camFusion_Student.cpp
    ├── dataStructures.h
    ├── FinalProject_Camera.cpp
    ├── lidarData.cpp
    ├── lidarData.hpp
    ├── matching2D.hpp
    ├── matching2D_Student.cpp
    ├── objectDetection2D.cpp
    └── objectDetection2D.hpp
```

### Options

Options can be set from within the `Options` struct in the main file.

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
            <td rowspan=1><b>Input data options</b></td>
            <td><code>bExtraAccuracy</code></td>
            <td><code>false</code></td>
            <td><code>true</code> for more accurate YOLOv3 blob size $(448 \times 448)$, <code>false</code> for default $(416 \times 416)$* [3]</td>
        </tr>
        <tr>
            <td rowspan=8><b>Visualisation and output options</b></td>
            <td><code>bVisYoloBoundingBoxes</code></td>
            <td><code>false</code></td>
            <td><code>true</code> to show YOLOv3 bounding boxes, COCO names, and confidence levels for each frame</td>
        </tr>
        <tr>
            <td><code>bVisLidarTopView</code></td>
            <td><code>false</code></td>
            <td><code>true</code> to display LiDAR top-view perspective, <code>false</code> to skip</td>
        </tr>
        <tr>
            <td><code>bStopAtLidarTopView</code></td>
            <td><code>false</code></td>
            <td>wrapper around the <code>continue</code> statement; <code>true</code> to cycle through all LiDAR top-views (if <code>bVisLidarTopView = true</code>), <code>false</code> to proceed with time-to-collision calculation</td>
        </tr>
        <tr>
            <td><code>bVisFinalOutput</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to view final output image with superimposed time-to-collision estimates</td>
        </tr>
        <tr>
            <td><code>bVisKeypointsOverlay</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to additionally superimpose keypoints on preceding vehicle bounding box</td>
        </tr>
        <tr>
            <td><code>bSaveYoloBBFrames</code></td>
            <td><code>false</code></td>
            <td><code>true</code> to write YOLOv3 bounding box frames inside the current working directory (if <code>bVisYoloBoundingBoxes = true</code>)</td>
        </tr>
        <tr>
            <td><code>bSaveLidarTopView</code></td>
            <td><code>false</code></td>
            <td><code>true</code> to write LiDAR top-views inside the current working directory (if <code>bVisLidarTopView = true</code>)</td>
        </tr>
        <tr>
            <td><code>bSaveOutputFrames</code></td>
            <td><code>false</code></td>
            <td><code>true</code> to write output frames inside the current working directory (if <code>bVisFinalOutput = true</code>)</td>
        </tr>
        <tr>
            <td rowspan=4><b>Outlier detection and diagnostics options</b></td>
            <td><code>FilteringMethod</code></td>
            <td><code>TUKEY</code></td>
            <td>Outlier filtering method. Either <code>TUKEY</code> for Tukey's fences [4] or <code>EUCLIDEAN_CLUSTERING</code> [6]</td>
        </tr>
        <tr>
            <td><code>bLimitKpts</code></td>
            <td><code>false</code></td>
            <td><code>true</code> to limit the number of keypoints. Helpful to debug and learn, but will introduce NaN values in time-to-collision calculations</td>
        </tr>
        <tr>
            <td><code>bRenderClusters</code></td>
            <td><code>false</code></td>
            <td><code>true</code> to render 3D LiDAR point clusters</td>
        </tr>
        <tr>
            <td><code>bShowRemoved</code></td>
            <td><code>true</code></td>
            <td><code>true</code> to also display colorless outliers (if <code>bRenderClusters = true</code>)</td>
        </tr>
    </tbody>
</table>

(*) Set to `true` to avoid spurious bounding boxes if `imgEndIndex` $>= 48$.

## Final Report

### FP.1: Match 3D Objects

The main reference for this task is [1]. To match bounding box pairs, I create a map object in which, for each element, the key will be a pair of bounding box indices (associated to the previous and current frames, respectively) and the value a counter of all occurrences of such key pair among the keypoint matches. For each match, after extracting the respective feature points in both frames [2], I first loop through all the bounding boxes in the query (previous) image to check which of them contains the related point; if a correspondence is found, I then run through all bounding boxes in the train (current) image to find the ones which also include the associated point. For those pairs where both conditions are met, the counter is incremented. The default overlap threshold value of 0.4 for the YOLOv3 non-maxima suppression algorithm does not guarantee a keypoint is contained in one and only one bounding box, so I avoid early termination once a correspondence is found despite the increase in computational time.

Once the map is populated, the query-train index pairs for which the counter is largest (one for each query bounding box index) are kept. Many-to-one cases in which multiple previous bounding boxes are associated to the same current one are not removed, as they do not seem to distort the analysis (for example, they are not related to cases of "spurious bounding boxes" [3] mentioned further below). The object-matching logic is found in [`matchBoundingBoxes`](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p3/src/camFusion_Student.cpp#L131) and called from the [main file](https://github.com/federicomariamassari/udacity-sfend/blob/main/projects/p3/src/FinalProject_Camera.cpp#L221).

### FP.2: Compute LiDAR-based TTC

LiDAR time-to-collision logic is handled by [`computeTTCLidar`](). Extreme or otherwise unreliable data are discarded using one of the available filtering methods as input to function [`removeOutliers`](). TTC is then given by the Constant Velocity Model (CVM) formula, with $\tilde{x}$ the median of all x-coordinates of the usable points (a proxy for distance $d$):

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
| `maxSize`  | 600           | Maximum cluster size. Clusters larger than this threshold will also be discarded. |

A visual comparison of the two algorithms is given in Figure 3 (white points do not enter the TTC calculation).

<table>
  <tr>
  <td align="center"><b>Figure 3.A</b>: Tukey's fences</td>
  <td align="center"><b>Figure 3.B</b>: Euclidean Clustering</td>
  <tr>
  </tr>
  <tr>
    <td align="center"><img align="center" src="img/mov2.gif" width="475"/></td>
    <td align="center"><img align="center" src="img/mov3.gif" width="475"/></td>
  </tr>
</table>

### FP.3: Associate Keypoint Correspondences with Bounding Boxes

To establish a connection between the YOLOv3 bounding boxes and the enclosed keypoints, I proceed as follows. For each match, I extract the keypoint descriptors for both the previous and the current frame; if the region of interest (ROI) of the latter [8] contains the associated feature, I compute the Euclidean distance (L2-norm) between the train and the query keypoints and preliminary push it back into a vector. As we expect a rigid transformation of the preceding vehicle [9] given the focus on a straight ego lane, I then remove all enclosed matches whose distance exceeds 1.5 times the median (the mean is not a robust enough estimator). The remaining correspondences, both matches and keypoints, are finally assigned back to their respective `BoundingBox` attributes. This logic is handled by [`clusterKptMatchesWithROI`]().

### FP.4: Compute Camera-based TTC

The main reference for camera-based time-to-collision computation is the solution to [10]. `minDistance` (the minimum threshold to avoid ambiguous matches when keypoints are too close or have too similar descriptors) is kept as 100, as both smaller and larger values were found to cause large swings in the final TTC estimates [11]. Tukey's fences are then applied to the filtered heights ratios before using the median ratio as input to time-to-collision.

### FP.5: Performance Evaluation 1

### FP.6: Performance Evaluation 2

To determine the best detector-descriptor pair for the camera-based time-to-collision, I rely on three criteria:

1. Proximity to the ground truth proxy (LiDAR time-to-collision, Tukey);
2. Speed of the detector-descriptor combination;
3. Relatively decreasing monotonicity of the time-to-collision estimate.

I consider all frames until the vehicle is nearly stationary (48), at which point the LiDAR TTC estimate becomes unreliable since the previous and current median values are so close to each other that their difference (at the denominator) is almost zero, leading to sudden spikes in the TTC output. `bExtraAccuracy = false` as default. The results are available in [`p3_performance_evaluation.xls`](./analysis/p3_performance_evaluation.xls).

__Figure 3: Camera-based Time-to-Collision vs LiDAR ground truth proxy__
<img src="./img/img1.svg" width="1000">

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
10. Lesson 3: Estimate TTC with a Camera, Engineering a Collision Detection System, Udacity Sensor Fusion Nanodegree
11. https://knowledge.udacity.com/questions/668076

[Home](../../README.md) | Previous: [Camera-Based 2D Feature Tracking](../p2/p2-camera-based-2d-feature-tracking.md) | Next: 
