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
- 
## Table of Contents
- [General Overview](#general-overview)
  - [Features](#features)
- [Getting started](#getting-started)
  - [Dependencies](#dependencies)
- [What is included in the module](#what-is-included-in-the-module)
  - [Hand Mediapipe node](#hand-mediapipe-node)## 2D Hand detection (`hand_mediapipe_node`)
  - [Hand landmarks 3D node](#hand-landmarks-3d-node)
  - [Hand palm plane node](#hand-palm-plane-node) ## Hand palm plane node (`hand_palm_plane_node`)
  - [Camera to robot frame hand transformation ](#camera-to-robot-frame-hand-transformation-(`hand_cam_to_base_node`))


- [Custom Interfaces](#custom-interfaces)
- [Running the Module](#running-the-module)
  - [Launch Arguments](#launch-arguments)
## Camera to robot frame hand transformation (`hand_cam_to_base_node`)

# Getting started
This module represents part of an operator detection system. This module is specialized in hand-tracking and processing, outputing a calculated plane and normal.

<p align="left">
  <!-- ---------- REUSABLE MODULE DIAGRAM ---------- -->
  <!-- Light mode -->
  <img src="images/Diagram_light.png#gh-light-mode-only" alt="EU Funding for light mode" height="500"/>

  <!-- Dark mode -->
  <img src="images/Diagram_dark.png#gh-dark-mode-only" alt="EU Funding for dark mode" height="500"/>

</p>

## Dependencies
All Python dependencies are included inside the `requirements.txt` file. To install, execute on terminal:
```bash
pip install -r /requirements.txt
```

This package is dependent on other ROS2 interfaces:
``` bash
sudo apt install \
  ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-hri-msgs
```
# What is included in the module


## 2D Hand detection (`hand_mediapipe_node`)
ROS2 node for real-time hand detection and 2D landmark extraction using MediaPipe Hands. This node subscribes to RGB camera images, runs MediaPipe hand tracking inference, and publishes:
- Tracked hand identifiers
- Normalized 2D hand landmarks
- Normalized 2D hand bounding boxes (ROIs)
- Optional debug visualization images

The node dynamically creates publishers for each detected hand and supports left/right handedness classification with optional handedness swapping to compensate for mirrored camera streams.

This node has the following interfaces:

| Interface | Topic | Description |
| ---------- |----- | ---------- |
| Publisher |`/humans/hands/tracked` | List of currently tracked hand identifiers. |
| Publisher | `/humans/hands/<id>/landmarks2d` | Normalized 2D landmarks for each detected hand. |
| Publisher | `/humans/hands/<id>/roi2d` | Normalized 2D bounding box for each detected hand. |
| Publisher | `/humans/hands/debug_image`| Debug visualization image with rendered landmarks. |
| Subscriber | *`*image_topic`*  | RGB image topic configured through the `image_topic` parameter. Default value is `/camera/cameraD455/color/image_raw`. |

## Camera to robot frame hand transformation (`hand_cam_to_base_node`)
ROS2 node for transforming hand landmarks and palm plane estimations from the camera reference frame into the robot base reference frame. Using the calibrated transformation between the robot TCP and the camera, the node computes the transformation from camera coordinates to the robot base frame and republishes:

- Hand landmarks in the base frame
- Palm plane estimations in the base frame
- Optional RViz visualization markers for palm centroid and normal vector

This node has the following interfaces:
| Interface |Topic | Description |
|--|----- | ---------- |
| Publisher |`/humans/hands/<id>/landmarks3d_base` | |
| Publisher|`/humans/hands/<id>/palm_plane_base` |  |
- Optional visualization markers

## Hand landmarks 3D node (`hand_landmarks_3d_node`)
  The node dynamically creates processing pipelines for tracked hands, converts normalized 2D landmark coordinates into 3D camera-frame
  coordinates, and publishes:

  - 3D hand landmarks
  - RViz visualization markers
  - Optional point clouds

  The node uses synchronized:
  - 2D hand landmark detections
  - Depth images
  - CameraInfo messages

## Hand palm plane node (`hand_palm_plane_node`)
This node subscribes to 3D hand landmark topics and computes a palm plane for each tracked hand using Singular Value Decomposition (SVD)-based plane fitting. The estimated plane is stabilized over time using configurable filtering techniques and optionally visualized in RViz.

>[!Important]
>The node dynamically creates and removes per-hand processing pipelines according to the currently tracked hands.

This node has the following interfaces:
| Interface |Topic | Description |
|--|----- | ---------- |
| Publisher |`/humans/hands/<id>/palm_plane_cam` | Estimated palm plane for each tracked hand.|
| Publisher|`/humans/hands/<id>/palm_plane_centroid_marker` | RViz centroid visualization marker.  |
| Publisher|`/humans/hands/<id>/palm_plane_normal_marker` | RViz normal vector visualization marker. |
| Subscriber|`/humans/hands/tracked` | List of currently tracked hands.|
| Subscriber|`/humans/hands/<id>/landmarks3d` | 3D hand landmark detections.|

### Plane Estimation
The palm plane is estimated from a configurable subset of landmarks using least-squares plane fitting based on SVD. The resulting normal vector can be stabilized through:
- Initial camera-facing orientation
- Sign consistency enforcement
- Angular step limiting
- Exponential moving average filtering

### Mesh Generation
----------------
When enabled, the node projects valid landmarks onto the estimated plane,
computes a 2D convex hull, triangulates the polygon, and publishes a
TRIANGLE_LIST RViz marker representing the palm surface.




# Custom Interfaces

A `custom_interfaces` package is included, to handle the custom message for the pipeline information and the goal




## Hand Tracking Message
Message definition for representing a tracked hand using MediaPipe landmarks:
![Reusable module dashboard](images/hand_landmarks.png)

| Field | Type | Description |
| ----- | ---- | ----------- |
| `id` | `string` | Unique identifier for the detected hand or tracked person. |
| `handedness` | `uint8` | Indicates whether the detected hand is left or right. |
| `landmarks` | `hri_msgs/NormalizedPointOfInterest2D[21]` | Array containing the 21 MediaPipe hand landmarks in normalized image coordinates. |
| `confidence` | `float32` | Confidence score of the hand detection/tracking. |

>[!Note]
>Handedness Convention
>| Constant | Value | Description |
>|---|---|---|
>| `HANDEDNESS_UNKNOWN` | `0` | Unknown handedness. |
>| `HANDEDNESS_LEFT` | `1` | Left hand detected. |
>| `HANDEDNESS_RIGHT` | `2` | Right hand detected. |

## Hand Tracking 3D Message

Message definition for representing a tracked hand in 3D space using MediaPipe landmarks.

| Field | Type | Description |
|---|---|---|
| `id` | `string` | Unique identifier for the detected hand or tracked person. |
| `handedness` | `uint8` | Indicates whether the detected hand is left or right. |
| `confidence` | `float32` | Confidence score of the handedness classifier. If unavailable, use `NaN` or `0`. |
| `landmarks` | `geometry_msgs/Point[21]` |Array containing the 21 MediaPipe hand landmarks in normalized image coordinates.|
| `valid` | `bool[21]` | Indicates whether each landmark is valid/tracked. |

>[!Note]
> Landmark coordinates are expressed in 3D space using `geometry_msgs/Point`. The `valid` array allows checking whether each landmark has been successfully estimated. `valid[i] == false` means landmark `i` should not be considered reliable.




# Landmarks


## Pipeline Statistics

For the sending the diagnostics to the Context Broker we use a custom interface `custom_interfaces/PipelineStats` with the followind definition:

| Field| Type| Description|
| ------------ | ----------------- | --------------------------------- |
| `---`         | `---`          | --- |

## Lifecycle action
All nodes will start as `unconfigured` upon launching the module. To unify the initialization of the system, we have created an `orchestrator` that hosts two actions, the first one being an action to change state to all the nodes in the correct order.

### Goal
| Field | Type     | Description                                                        |
| ----- | -------- | ------------------------------------------------------------------ |
| `target_state`  | `string` | State we want to change the system to. |

### Result
| Field         | Type    | Description |
| ------------- | ------- | ----------- |
| `success`     | `bool`  | Indicates whether the action execution completed successfully.|
| `result_msg`  | `string`| Final message resulting from the execution of the action. |

### Feedback
| Field      | Type     | Description                                                                  |
| ---------- | -------- | ---------------------------------------------------------------------------- |
| `feedback` | `string` | Informational message describing the current status of the action execution. |

## Hand change action
The second action from the `orchestrator` is created for changing which hand is going to be tracked. Options are **left**, **right** or **both**.

### Goal
| Field | Type     | Description                                                        |
| ----- | -------- | ------------------------------------------------------------------ |
| `hand`  | `string` | Hand we want to track. Options: (left, right, both). |

### Result
| Field         | Type    | Description |
| ------------- | ------- | ----------- |
| `success`     | `bool`  | Indicates whether the action execution completed successfully.|
| `result_msg`  | `string`| Final message resulting from the execution of the action. |

### Feedback
| Field      | Type     | Description                                                                  |
| ---------- | -------- | ---------------------------------------------------------------------------- |
| `feedback` | `string` | Informational message describing the current status of the action execution. |


# Running the Module

The launch file starts the full hand-tracking pipeline:

```bash
ros2 launch hand_hri_mediapipe hand_pipeline_lifecycle.launch.py
```

## Launch Arguments

| Argument | Default | Type | Description |
|---|---|---|---|
| `camera_name` | `cameraD455` | string | Camera namespace used to build the color, depth, and camera info topics. |


---
This project has received funding from **Horizon Europe** research and innovation programme under grant agreement **no. 101135784**.
<p align="left">
  <!-- ---------- ARISE logo ---------- -->
  <!-- Light mode -->
  <img src="images/EN_FundedbytheEU_RGB_POS.png#gh-light-mode-only" alt="EU Funding for light mode" height="100"/>

  <!-- Dark mode -->
  <img src="images/EN_FundedbytheEU_RGB_NEG.png#gh-dark-mode-only" alt="EU Funding for dark mode" height="100"/>

</p>
