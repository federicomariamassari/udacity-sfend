[Home](../../README.md) | Previous: [Track an Object in 3D Space](../p3/p3-track-an-object-in-3d-space.md) | Next:

# Project 4: Radar Target Generation and Detection

## Overview

Most tasks are timed to monitor their efficiency.

## Project Report

### FMCW Waveform Design

Given above parameter specifications, the Frequency-Modulated Continuous Wave (FMCW) is given by:

$$
\text{bandwidth} = \frac{\text{speed of light}}{2\times\text{range resolution}} = \frac{3\times 10^8}{2\times 1} =
 1.5\times 10^8
$$

$$
\text{chirp time} = 5.5 \times 2 \times \frac{\text{max range}}{\text{speed of light}} = 5.5 \times 2 \times
 \frac{200}{3\times 10^8} = 7.\overline{3} \times 10^{-6}
$$

$$
\text{slope} = \frac{\text{bandwidth}}{\text{chirp time}} = \frac{1.5\times 10^8}{7.\overline{3} \times 10^{-6}} =
 2.0455\times 10^{13}
$$

### Simulation Loop

### Range FFT

### 2D CA-CFAR

### Non-Thresholded Edges

[Home](../../README.md) | Previous: [Track an Object in 3D Space](../p3/p3-track-an-object-in-3d-space.md) | Next: