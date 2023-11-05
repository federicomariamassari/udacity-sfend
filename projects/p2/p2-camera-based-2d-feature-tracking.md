[Home](../../README.md) | Previous: [LiDAR Obstacle Detection](../p1/p1-lidar-obstacle-detection.md) | Next: 

# Project 2: Camera-Based 2D Feature Tracking

## Preliminary Considerations

This project was originally developed on a UTM QEMU 7.0 Virtual Machine running Ubuntu 20.04-5 LTS on Apple Silicon architecture, and only later ported to the Udacity workspace. It uses OpenCV 4.2.0, built from source to enable the heavily-patented algorithms SIFT/SURF [1].

### MP.1 Data Buffer Optimization

A data ring buffer assures efficient memory management by limiting the number of images simultaneously present in the holding data structure, preventing the structure from growing excessively in size. Here, the buffer is a C++ vector with maximum size two; if the vector is full, we erase the earliest pushed-back image first (FIFO), then add the next element [2]. Complexity: linear on the element erased (basically constant, since it's always the first one and no search is involved), plus linear on moving the remaining element to position 0 [3].

### MP.2 Keypoint Detection

## Resources

1. [Preliminary Configurations, P4: Map My World](https://github.com/federicomariamassari/udacity-rsend/blob/main/projects/p4/p4-preliminary-config.md#3-rebuild-opencv-from-source-with-patented-modules), Udacity Robotics Software Engineer Nanodegree, Federico Massari
2. https://knowledge.udacity.com/questions/644337
3. https://cplusplus.com/reference/vector/vector/erase/

[Home](../../README.md) | Previous: [LiDAR Obstacle Detection](../p1/p1-lidar-obstacle-detection.md) | Next: 