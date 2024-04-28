# QRCodeScannerNode

QRCodeScannerNode is a simple ROS2 node that captures images and locates QR codes within them using OpenCV. When a QR code is discovered, the node publishes information about the code and the position of its midpoint.

This program utilizes the `common_package_py` package and the `interfaces` package for message types.

Both real-time streams from a camera and test images can be used for QR detection.

## Dependencies

- ROS 2
- OpenCV
- `std_msgs`
- `common_package_py`
- `interfaces`

## Classes

### QRCodeScannerNode

This class inherits from `CommonNode`.

#### Main methods:

- `__init__(self, id: str)`: Initializes the QR code detector and the publisher.
- `process_images(self)`: Continuously runs and processes images to detect and publish QR codes.

#### Helper methods:

- `__capture_image(self)`: Captures an image either from the camera or a test image.
- `__corners_to_middlepoint(self, points)`: Calculates the geometric midpoint of a rectangle formed by four points.
- `__detect_qr_codes(self, image)`: Detects QR codes in the image and calculates the midpoint of the bounding rectangle.

## Usage

To run the node, use the following command:

```bash
ros2 run qrcode_detection_package qr_code_scanner_node
```
