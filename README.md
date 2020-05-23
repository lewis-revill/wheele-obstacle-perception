### WHEEL.E Obstacle Perception

#### Background

Obstacle perception is a broad subsystem which forms part of the design for the WHEEL.E autonomous wheelchair. This subsystem takes inputs from sensors in the form of stereo images and produces a map of the obstacles within the field of view of the sensors at any given time.

This output is intended to be processed by the localisation subsystem in order to aid in determining the current location of the wheelchair, and by the route planning subsystem in order to determine the best route to reach a destination while avoiding the detected obstacles.

#### Specification

This project enables a 2D map to be created showing obstacles in the area surrounding the wheelchair. For this purpose the following inputs are required:

- Two RGB images

- The distance between each camera

The following outputs are expected:

- A 2D labelled occupancy map showing presence or absence of objects

#### Design

#### Implementation

#### Testing
