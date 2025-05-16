# TurtleBot Anomaly Detection & Surveillance System

A ROS-based security system that enables a TurtleBot to autonomously map an environment, patrol key areas, detect anomalies, and respond by collecting data such as images and positional information.

---

## Overview

Build a security system to detect anomalies (apart from buildings and boundaries), estimate their location, and send photos for review.

> **Anomalies Meaning:** Something that is unusual, unexpected, or doesn't fit a pattern; similar to "abnormality" but often used in scientific or technical contexts.

---

## 🔄 System Workflow

### Step 1: Environment Mapping

Initiate a mapping routine using TurtleBot's onboard sensors (e.g., SLAM with LiDAR or depth camera) to generate an accurate map of the operational area.

### Step 2: Define Surveillance Routine

Set up a predefined patrol or surveillance path using waypoints or navigation goals, enabling continuous monitoring of critical zones.

### Step 3: Anomalous Object Detection

Detect unexpected or unauthorized objects in the environment using:

- LiDAR-based shape deviation analysis
- Visual classification via neural networks (camera input)

### Step 4: Target Engagement and Data Collection

Upon anomaly detection:
- Navigate toward the target
- Capture detailed images
- Record GPS or relative coordinates
- Emit sound alerts or audio messages

### Step 5: Resume Patrol

After engagement and data collection, return to the previous patrol route and continue surveillance operations.

---

## Development Phases

### Phase 1: Basic Detection and Localization

- Detect object via LiDAR
- Estimate position using environment maps
- Refine location using camera data

### Phase 2: Object Classification

- Integrate a pre-trained image classification model
- Categorize detected anomalies (e.g., unknown object, misplaced item, person, etc.)

---

## 📦 Dependencies

- ROS (Robot Operating System)
- TurtleBot3 packages
- SLAM Toolbox or GMapping
- OpenCV (for image processing)
- TensorFlow / PyTorch (for neural network inference)
- RViz (for visualization)
- Python 3.x

---

## 📂 Project Structure

