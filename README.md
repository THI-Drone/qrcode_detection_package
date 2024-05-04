# QR Code Scanner Node

This program is a ROS 2 node designed for detecting and decoding QR codes from images captured either from a camera or from a test image. It utilizes the OpenCV library for QR code detection and decoding.

## Table of Contents
- [Overview](#overview)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [ROS 2 Topics](#ros-2-topics)
- [Node States](#node-states)
- [Functions](#functions)
- [License](#license)

## Overview
The QR Code Scanner Node is responsible for capturing images, detecting QR codes within those images, and publishing information about the detected QR codes. It supports multiple methods for capturing images, including using a camera feed or loading a test image. The detected QR code information, along with its position relative to the image, is published on a ROS 2 topic for further processing.

## Dependencies
- ROS 2: The Robot Operating System 2 framework is required to run this node.
- OpenCV: The OpenCV library is used for QR code detection and decoding.
- common_package_py
- interfaces

## Usage

To run the node, use the following command:

```bash
ros2 run qrcode_detection_package qr_code_scanner_node
```

## ROS 2 Topics
- **Subscribed Topics**:
  - `/control`: Control messages are subscribed to activate or deactivate the QR code scanner node.
  
- **Published Topics**:
  - `/qr_codes`: Detected QR code information is published on this topic.
  - `/job_finished`: Publishes the end of the job with optional error code or the QR-Code content as payload.

## Node States
The QR Code Scanner Node operates in two main states:
1. **Ready**: The node is initialized and waiting for a control message to activate it. In this state, no image capturing or QR code scanning occurs.
2. **Searching**: The node continuously captures images and scans them for QR codes. If a valid QR code is found, its information is published, and the node switches back to the "Ready" state.

## Functions

### Main Functions

#### Constructor
Initializes the QRCodeScannerNode object and performs necessary setup tasks, such as setting the initial state to "ready", configuring subscriptions and publishers, and initializing the QR code detector.

#### __callback_control
Activates or deactivates the QR code scanner node based on the received control message.

#### main
Initializes the node and runs the state machine over the lifetime of the node. It continuously checks the state of the node and performs appropriate actions based on that state.

#### scan_for_qr_code
Captures an image, detects QR code, and publishes information about it. If a valid QR code is detected, the method saves the image containing the QR code and sends a job finished message indicating successful completion of the task. If no QR code is found or if there are errors in capturing the image, appropriate log messages are generated.

### Helper Functions

#### set_state
Sets the state of the node to the provided state.

#### __capture_image
Captures an image from the camera or a test image, depending on the configuration specified by 'config_detection_method'. If 'config_detection_method' is set to 0, a test image is loaded. If set to 1, an image is captured from the camera with OpenCV. If set to 2, an image is captured using the libcamera command line tool.

#### __corners_to_middlepoint
Calculates the geometric midpoint the detected qr-code formed by 4 points.

#### __relative_midpoint
Calculates the position of the QR code relative to the middle of the image by determining the percentage of deviation from the image center along the x and y axes. Output is in percentage from -100% to 100%

#### __detect_qr_codes
Detects QR codes in the provided image using the OpenCV library. If a QR code is successfully decoded, its information along with the geometric midpoint of its bounding rectangle are logged. Additionally, the relative midpoint coordinates, relative to the center of the image, are calculated and logged. If no QR code is detected, the method returns an error message along with default midpoint coordinates.


## License
This program is licensed under the [MIT License](LICENSE).