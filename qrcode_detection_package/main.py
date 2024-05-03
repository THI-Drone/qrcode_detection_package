import sys
import rclpy
import time
import subprocess
from rclpy.node import Node
import cv2
from std_msgs.msg import String

from common_package_py.common_node import CommonNode
from interfaces.msg import QRCodeInfo


class QRCodeScannerNode(CommonNode):
    def __init__(self, id: str):
        super().__init__(id)
        self.qr_code_detector = cv2.QRCodeDetector()
        self.qr_publisher = self.create_publisher(QRCodeInfo, 'qr_codes', 10)
        # until tested properly different detection methods are implemented and can be configured here
        # 0 = load stored test image (for testing purposes)
        # 1 = use OpenCV to get image from camera (this is the wanted solution)
        # 2 = use libcamera shell script to take photo and load it (fallback solution)
        self.config_detection_method = 0

    def __capture_image(self):
        """
        @brief Capture an image either from the camera or a test image, depending on the configuration.

        @return OpenCV MatLike representing the captured image.
        """
        import cv2
        
        # Check which detection style should be used
        if (self.config_detection_method == 0):
            # Path to the test image
            test_image_path = 'src/qrcode_detection_package/test_image/test.jpg'
            
            # Load the test image
            captured_image = cv2.imread(test_image_path)
        elif (self.config_detection_method == 1):
            # Initialize a VideoCapture object for the camera
            cap = cv2.VideoCapture(0)
            
            # Check if the VideoCapture object was opened successfully
            if not cap.isOpened():
                self.get_logger().info("Error: Unable to open camera")
            
            # Capture an image from the camera
            ret, img = cap.read()
            
            # Check if the image was captured successfully
            if not ret:
                self.get_logger().info("Error: Unable to capture image from camera")
            
            captured_image = img
            
            # Release the VideoCapture object
            cap.release()
            
        elif (self.config_detection_method == 2):
            try:
                # set image path with timestamp as name
                timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
                image_path = f"/image/{timestamp}.jpg"
                # execute libcamera command to capture imagae
                command = ["libcamera-jpeg", "-o", image_path]
                subprocess.run(command)
            except:
                self.get_logger().info("libcamera could not take picture")
            captured_image = cv2.imread(image_path)
        
        return captured_image
    
    def __corners_to_middlepoint(self, points):
        """
        @brief Calculate the geometric midpoint of a rectangle formed by 4 points.

        @param points: A list of tuples representing the coordinates of the 4 points.

        @return A tuple representing the geometric midpoint of the rectangle.
        """
        # Extract the coordinates of the diagonal points
        top_left = points[0][0]
        bottom_right = points[0][2]
        
        # Calculate the geometric midpoint
        midpoint_x = (top_left[0] + bottom_right[0]) / 2
        midpoint_y = (top_left[1] + bottom_right[1]) / 2
        
        # Return the midpoint as a tuple (x, y)
        return (midpoint_x, midpoint_y)
    
    def __relative_midpoint(self, midpoint_x, midpoint_y, img_width, img_height):
        """
        @brief calculate the midpoints of the qrcode relative to the middle of the image

        @param absolute midpoint coordinates and image width and height

        @return A tuple representing the relative position of the qr code to the middle of the picture
        """
        # calculation of y requires inversed logic due to inversed OpenCV image y-coordinates
        rel_midpoint_x = midpoint_x - (img_width/2)
        rel_midpoint_y = (img_height/2) - midpoint_y
        
        return (rel_midpoint_x, rel_midpoint_y)


    def __detect_qr_codes(self, image):
        """
        @brief Detect QR codes in the image and calculate the midpoint of the bounding rectangle.

        @param image: The image in which to detect QR codes.

        @return A tuple containing:
                - The decoded information from the QR code.
                - The x-coordinate of the geometric midpoint of the QR code.
                - The y-coordinate of the geometric midpoint of the QR code.
        """
        # Detect QR codes in the image using OpenCV
        decoded_info, points, _ = self.qr_code_detector.detectAndDecode(image)
        
        # Calculate the geometric midpoint of the bounding rectangle
        midpoint_x, midpoint_y = self.__corners_to_middlepoint(points)
        
        # Log the midpoint coordinates
        self.get_logger().info(f"QR code middle point: ({midpoint_x}|{midpoint_y})")
        
        # Get midpoints relative to the center of the picture
        img_height, img_width = image.shape[:2]
        rel_midpoint_x, rel_midpoint_y = self.__relative_midpoint(midpoint_x, midpoint_y, img_width, img_height)
        
        # Log the relative midpoint coordinates
        self.get_logger().info(f"Relative QR code middle point: ({rel_midpoint_x}|{rel_midpoint_y})")
        
        return decoded_info, rel_midpoint_x, rel_midpoint_y

    def process_images(self):
        """
        @brief This method continuously captures images, detects QR codes in the images, and publishes
        information about the detected QR codes.

        """
        while True:
            # capture image
            captured_image = self.__capture_image()
            if captured_image is not None:
                # use OpenCV to detect qr codes in the image
                qr_code_content, qrcode_center_x, qrcode_center_y = self.__detect_qr_codes(captured_image)
                if qr_code_content:
                    # if a QR-Code was successfully detected, publish contents on the topic
                    qr_code_position = [qrcode_center_x, qrcode_center_y]
                    
                    msg = QRCodeInfo()
                    msg.qr_code_content = qr_code_content
                    msg.qr_code_position = qr_code_position

                    self.qr_publisher.publish(msg)
                    self.get_logger().info("Published QR code info")
                    
                    self.get_logger().info(
                        f"Detected QR code: {qr_code_content}")
                else:
                    self.get_logger().info(f"No QR Code found")


def main(args=None):
    """
    @brief Entry point of the QR code scanner node.

    This function initializes the ROS 2 node, creates an instance of the QRCodeScannerNode class,
    and starts the image processing loop. It handles the cleanup operations before shutting down
    the node.

    @param args: Command-line arguments. Default is None.
    """
    rclpy.init(args=args)
    node_id = 'qr_code_scanner_node'
    qr_code_scanner_node = QRCodeScannerNode(node_id)

    try:
        qr_code_scanner_node.process_images()
    finally:
        qr_code_scanner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
