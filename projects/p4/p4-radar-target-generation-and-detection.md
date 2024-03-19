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
\text{bandwidth} = \frac{\text{speed of light}}{2\times\text{range resolution}} = \frac{3\times 10^8\ \text{m/s}}{2\times 1\ \text{m}} =
 1.5\times 10^8\ \text{Hz}
$$

$$
\text{chirp time} = 5.5 \times 2 \times \frac{\text{max range}}{\text{speed of light}} = 5.5 \times 2 \times
 \frac{200\ \text{m}}{3\times 10^8\ \text{m/s}} = 7.\overline{3} \times 10^{-6}\ \text{s}
$$

$$
\text{slope} = \frac{\text{bandwidth}}{\text{chirp time}} = \frac{1.5\times 10^8\ \text{Hz}}{7.\overline{3} \times 10^{-6}\ \text{s}} =
 2.0\overline{45}\times 10^{13}\ \text{Hz/s}
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

#### Training and Guard Cells

#### Offset

The optimal offset value was found to be 5-6. If the offset is too small (4 or below), a large number of false positives are detected [Figure 2.A]. If it is too large (e.g., 10), the target signal becomes increasingly faint [Figure 2.B].

<table>
  <tr>
  <td align="center"><b>Figure 2.A</b>: Large number of false positives (offset = 4)</td>
  <td align="center"><b>Figure 2.B</b>: Faint target signal (offset = 10)</td>
  <tr>
  </tr>
  <tr>
    <td align="center"><img align="center" src="img/img4a.svg" width="475"/></td>
    <td align="center"><img align="center" src="img/img4b.svg" width="475"/></td>
  </tr>
</table>

### Non-Thresholded Edges

__Figure 3: 2D Cell-Averaging Constant False Alarm Rate__
![2D CA-CFAR](./img/img3.svg)

## Resources

1. https://knowledge.udacity.com/questions/848592

[Home](../../README.md) | Previous: [Track an Object in 3D Space](../p3/p3-track-an-object-in-3d-space.md) | Next: