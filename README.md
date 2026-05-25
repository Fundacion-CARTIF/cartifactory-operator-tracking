<p align="center">

  <!-- ---------- TOP IMAGE ---------- -->
  <a href="https://www.cartif.es/en/human-robot-collaboration-pilot-plant-cartifactory/">
    <img src="images/cartifactory-transparent.png" alt="Top image" height="100"/>
  </a>
  <!-- ---------- CARTIF logo ---------- -->
  <a href="https://www.cartif.es/en/home/">
    <img src="images/CARTIF_transparente.png" height="100">
  </a>

</p>

<p align="center">
  <!-- ---------- ARISE logo ---------- -->
  <!-- Light mode -->
  <img src="images/ARISE_logo-light_mode.png#gh-light-mode-only" alt="Logo for light mode" height="60"/>
  <!-- Dark mode -->
  <img src="images/ARISE_logo-dark_mode.png#gh-dark-mode-only" alt="Logo for dark mode" height="60"/>
</p>


<p align="center">
  These 
  Modules are part of the ARISE Middleware<br>
  <a href="https://arise-middleware.eu/">ARISE Middleware site</a>
</p>

<hr>


[![vulcanexus](https://img.shields.io/badge/Vulcanexus%20Version-Jazzy-%230895CD?style=flat)](https://github.com/eProsima/vulcanexus)
[![ubuntu24](https://img.shields.io/badge/Ubuntu-24.04-%23E95420?style=flat&logo=ubuntu
)](https://releases.ubuntu.com/24.04/)


# General Overview
[ARISE](https://arise-middleware.eu/) aims towards making industrial HRI more accessible and cost-effective, in particular in healthcare, intra-logistics and manufacturing sectors. These modules hope to present an integration between [FIWARE Orion Context Broker](https://fiware-orion.readthedocs.io/en/master/) and [eProsima Vulcanexus](https://vulcanexus.org/) to enable context-aware robotic and industrial applications, alongside [ROS4HRI](https://ros4hri.github.io/) as an open-source ROS standard and a set of ROS packages to facilitate the development of Human-Robot Interaction (HRI) capabilities on robots.

>[!TIP]
>Check here the rest of our Testing and Experimenting Facility [CARTIFactory](https://fundacion-cartif.github.io/CARTIFactory/)!


## Features
- Automatic CPU / CUDA support
- Segmentation support (OBB from mask)
- Optional annotated image publishing
- Latched `class_map` publishing
- Industrial metrics (FPS, infer_ms, device, etc.)
- Configurable `class_id` mode

## Table of Contents
- [General Overview](#general-overview)
  - [Features](#features)
  - [Table of Contents](#table-of-contents)
- [Getting started](#getting-started)
  - [Dependencies](#dependencies)
- [What is included in the module](#what-is-included-in-the-module)
  - [Hand Mediapipe node](#hand-mediapipe-node)
  - [Dependencies](#dependencies-1)
  - [hand\_cam\_to\_base\_node](#hand_cam_to_base_node)
  - [Hand landmarks 3D node](#hand-landmarks-3d-node)
  - [Hand palm plane node](#hand-palm-plane-node)
  - [Features](#features-1)
  - [Published Topics](#published-topics)
  - [Subscribed Topics](#subscribed-topics)
  - [Plane Estimation](#plane-estimation)
  - [Mesh Generation](#mesh-generation)
  - [Dependencies](#dependencies-2)
- [Running the Module](#running-the-module)
  - [Launch Arguments](#launch-arguments)

# Getting started

## Dependencies

# What is included in the module


## Hand Mediapipe node
ROS2 node for real-time hand detection and 2D landmark extraction using MediaPipe Hands.

This node subscribes to RGB camera images, runs MediaPipe hand tracking inference, and publishes:
- Tracked hand identifiers
- Normalized 2D hand landmarks
- Normalized 2D hand bounding boxes (ROIs)
- Optional debug visualization images

The node dynamically creates publishers for each detected hand and supports left/right handedness classification with optional handedness swapping to compensate for mirrored camera streams.

Features:
- Real-time MediaPipe Hands inference
- Multi-hand tracking
- Dynamic ROS publisher creation
- 2D landmark extraction
- Normalized ROI generation
- Optional debug image visualization
- Handedness correction for non-mirrored cameras
- ROS2 integration using custom HRI messages


### Node interfaces
This node has the following interfaces:

| Interface | Topic | Description |
| ---------- |----- | ---------- |
| Publisher |`/humans/hands/tracked` | List of currently tracked hand identifiers. |
| Publisher | `/humans/hands/<id>/landmarks2d` | Normalized 2D landmarks for each detected hand. |
| Publisher | `/humans/hands/<id>/roi2d` | Normalized 2D bounding box for each detected hand. |
| Publisher | `/humans/hands/debug_image`| Debug visualization image with rendered landmarks. |
| Subscriber | *`*image_topic`*  | RGB image topic configured through the `image_topic` parameter. |

### Dependencies
This node has the following dependencies
- MediaPipe
- OpenCV
- cv_bridge
- NumPy
- hri_msgs
- hand_hri_mediapipe_msgs


## hand_cam_to_base_node
ROS2 node for transforming hand landmarks and palm plane estimations
from the camera reference frame into the robot base reference frame.

This node subscribes to:
- 3D hand landmark detections
- Palm plane estimations
- Robot TCP pose

Using the calibrated transformation between the robot TCP and the camera,
the node computes the transformation from camera coordinates to the robot
base frame and republishes:

- Hand landmarks in the base frame
- Palm plane estimations in the base frame
- Optional RViz visualization markers for palm centroid and normal vector

Main features:
- Dynamic management of tracked hands
- Landmark coordinate transformation
- Palm plane transformation and filtering
- Normal vector stabilization with:
    - Flip protection
    - Angular step limiting
    - Exponential smoothing
- Automatic cleanup of stale hands
- RViz visualization support

| Interface |Topic | Description |
|--|----- | ---------- |
| Publisher |`/humans/hands/<id>/landmarks3d_base` | |
| Publisher|`/humans/hands/<id>/palm_plane_base` |  |
- Optional visualization markers

## Hand landmarks 3D node
  ROS2 node that projects 2D hand landmarks into 3D space using
  synchronized depth images and camera intrinsic parameters.

  The node dynamically creates processing pipelines for tracked hands,
  converts normalized 2D landmark coordinates into 3D camera-frame
  coordinates, and publishes:

  - 3D hand landmarks
  - RViz visualization markers
  - Optional point clouds

  The node uses synchronized:
  - 2D hand landmark detections
  - Depth images
  - CameraInfo messages

  Features
  --------
  - Dynamic hand tracking pipeline creation/removal
  - 2D-to-3D landmark projection
  - Depth hole filling
  - Depth range validation
  - Normalized coordinate clamping
  - RViz skeleton and landmark visualization
  - Optional PointCloud2 publishing
  - Automatic cleanup of stale hands




## Hand palm plane node
ROS2 node for estimating stable palm planes from 3D hand landmarks.

This node subscribes to 3D hand landmark topics and computes a palm plane
for each tracked hand using Singular Value Decomposition (SVD)-based plane
fitting. The estimated plane is stabilized over time using configurable
filtering techniques and optionally visualized in RViz.

The node dynamically creates and removes per-hand processing pipelines
according to the currently tracked hands.

Features
--------
- Dynamic multi-hand tracking
- Plane fitting from configurable landmark indices
- SVD-based plane estimation
- Stable plane normal orientation
- Plane normal filtering and smoothing
- Angular step clamping
- Flip prevention for plane normals
- Plane quality validation using RMSE and inlier count
- Optional reuse of the last valid plane estimate
- RViz visualization markers:
    - Palm centroid
    - Palm normal vector
    - Palm mesh surface
- Optional convex hull mesh generation from hand landmarks

Published Topics
----------------
- /humans/hands/<id>/palm_plane_cam
    Estimated palm plane for each tracked hand.

- /humans/hands/<id>/palm_plane_centroid_marker
    RViz centroid visualization marker.

- /humans/hands/<id>/palm_plane_normal_marker
    RViz normal vector visualization marker.

- /humans/hands/<id>/palm_plane_mesh_marker
    RViz palm mesh visualization marker.

Subscribed Topics
-----------------
- /humans/hands/tracked
    List of currently tracked hands.

- /humans/hands/<id>/landmarks3d
    3D hand landmark detections.

Plane Estimation
----------------
The palm plane is estimated from a configurable subset of landmarks using
least-squares plane fitting based on SVD. The resulting normal vector can
be stabilized through:

- Initial camera-facing orientation
- Sign consistency enforcement
- Angular step limiting
- Exponential moving average filtering

Mesh Generation
----------------
When enabled, the node projects valid landmarks onto the estimated plane,
computes a 2D convex hull, triangulates the polygon, and publishes a
TRIANGLE_LIST RViz marker representing the palm surface.

Dependencies
------------
- ROS2
- NumPy
- OpenCV
- visualization_msgs
- geometry_msgs
- hand_hri_mediapipe_msgs
- hri_msgs


# Running the Module

## Launch Arguments

| Argument  | Default   | Type  | Description   | 
|--------   |--------   |-------|--------       |
|           |           |       |               |

