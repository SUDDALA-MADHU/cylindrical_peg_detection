# Cylindrical Peg Detection

A Python project for detecting cylindrical pegs in images using classical computer vision techniques (Hough Transform).

This repository implements a vision-based cylindrical object detector that can be used in robotics perception pipelines (e.g., peg-in-hole assembly or part localization). The solution is lightweight, interpretable, and suitable as a baseline vision detector for structured environments.

---

## Features :

- Detects cylindrical pegs in images using gradient and shape-based methods  
- Utilizes the Hough Circle/Line Transform for robust radial/cylindrical detection  
- Outputs annotated images showing detected objects  
- Easy to integrate with ROS perception or robot task planners

---

##  Motivation

Vision-based object detection is a foundational step in many robotic assembly tasks — e.g., peg-in-hole or insertion problems where accurate pose estimation of a cylindrical feature is required. Classical vision techniques like Hough transforms provide fast and explainable detection without the need for deep learning training. :contentReference[oaicite:1]{index=1}

---

## Getting Started
These steps will get you a copy of the project up and running on your local machine.

 ## Requirements:

- Ubuntu 20.04 / 22.04  
- ROS / ROS 2 (depending on your setup)
- Python 3
- OpenCV
- NumPy

##Install dependencies:
pip install opencv-python numpy

##Running the Project (ROS Command)
#1️⃣ Build the workspace:
-cd ~/ros_ws
-colcon build
-source install/setup.bash

#2️⃣ Launch the peg detection node:
-ros2 run cylindrical_peg_detection peg_detector_node

##The node subscribes to the camera image topic and performs real-time cylindrical peg detection.
