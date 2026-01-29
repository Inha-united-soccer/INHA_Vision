<div align="center">

# :soccer: INHA Vision
**Advanced Autonomous Agent for Humanoid Soccer**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-3490dc.svg?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-00599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![License](https://img.shields.io/badge/License-Apache_2.0-yellow.svg?style=for-the-badge)](LICENSE)

*robust Detection • real-time inference • 3D position estimation

---
</div>

## Mission & Vision
**"To enable humanoid robots to perceive the game reliably, even while moving."**


The **INHA Vision** is designed as a visual perception module to enable stable recognition even during movement. It reliably detects and tracks balls, robots, goalposts, and field markers(L, T, X) in dynamic environments with varying lighting conditions.

---

## Key Features

### **Robust Object Detection**
We employ a YOLOv8-based detector optimized for embedded platforms.
* Detection of **ball, goalposts, robots, and field markers (L, T, X)**
* Trained on over 40,000 annotated images, including public datasets and in-house data


### **Real-Time Inference**
To meet strict real-time constraints on embedded humanoid platforms, the vision model is optimized using NVIDIA TensorRT.
* Conversion of trained detection models to TensorRT engines for low-latency inference
* Stable real-time performance on Jetson Orin NX under on-board computational constraints

### **3D Pose Estimation in Robot Frame**
Detected objects are converted from image space into metric 3D positions:
* Intrinsic-based pixel-to-ray projection
* Transformation into the base frame using IMU-compensated kinematic data

---

## System Architecture

The system is built on a robust perception-action loop:
<p align="center">
  <img src="images/vision_pipeline.png" width="800"/>
</p>

---

<div align="center">
    <b>Built with by INHA United</b><br>
    <i>Pushing the boundaries of Autonomous Soccer</i>
</div>
