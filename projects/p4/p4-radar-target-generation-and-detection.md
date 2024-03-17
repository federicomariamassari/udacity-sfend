[Home](../../README.md) | Previous: [Track an Object in 3D Space](../p3/p3-track-an-object-in-3d-space.md) | Next:

# Project 4: Radar Target Generation and Detection

## Overview

Most tasks are timed to monitor their efficiency.

## Project Report

### Radar System Requirements

- Frequency: 77 GHz
- Range resolution: 1 m
- Maximum range: 200 m
- Maximum velocity: 70 m/s

### FMCW Waveform Design

From the above parameter specifications, the Frequency-Modulated Continuous Wave (FMCW) is given by:

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
 2.0\overline{45}\times 10^{13}
$$

### Simulation Loop

### Range FFT

<table>
  <tr>
  <td align="center"><b>Figure 1.A</b>: Single-Sided Beat Signal Amplitude Spectrum</td>
  <td align="center"><b>Figure 1.B</b>: Range-Doppler Map</td>
  <tr>
  </tr>
  <tr>
    <td align="center"><img align="center" src="img/img2a.svg" width="475"/></td>
    <td align="center"><img align="center" src="img/img2b.svg" width="475"/></td>
  </tr>
</table>

### 2D CA-CFAR

### Non-Thresholded Edges

__Figure 2: 2D Cell-Averaging Constant False Alarm Rate__
![2D CA-CFAR](./img/img3.svg)

[Home](../../README.md) | Previous: [Track an Object in 3D Space](../p3/p3-track-an-object-in-3d-space.md) | Next: