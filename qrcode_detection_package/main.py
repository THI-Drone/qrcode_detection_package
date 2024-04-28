import sys
import rclpy
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
        self.use_test_image = True

    def __capture_image(self):
        """
        Capture an image either from the camera or a test image, depending on the configuration.

        Returns:
        - A numpy array representing the captured image.
        """
        import cv2
        
        
        # Check if the test image should be used
        if self.use_test_image:
            # Path to the test image
            test_image_path = 'src/qrcode_detection_package/test_image/test.jpg'
            
            # Load the test image
            captured_image = cv2.imread(test_image_path)
        else:
            # Initialize a VideoCapture object for the camera
            cap = cv2.VideoCapture(0)
            
            # Check if the VideoCapture object was opened successfully
            if not cap.isOpened():
                print("Error: Unable to open camera")
                return None
            
            # Capture an image from the camera
            ret, frame = cap.read()
            
            # Check if the image was captured successfully
            if not ret:
                print("Error: Unable to capture image from camera")
                return None
            
            # Release the VideoCapture object
            cap.release()
            
            captured_image = frame
        
        return captured_image
    
    def __corners_to_middlepoint(self, points):
        """
        Calculate the geometric midpoint of a rectangle formed by 4 points.

        Args:
        - points: A list of tuples representing the coordinates of the 4 points.

        Returns:
        - A tuple representing the geometric midpoint of the rectangle.
        """
        # Extract the coordinates of the diagonal points
        top_left = points[0][0]
        bottom_right = points[0][2]
        
        # Calculate the geometric midpoint
        midpoint_x = (top_left[0] + bottom_right[0]) / 2
        midpoint_y = (top_left[1] + bottom_right[1]) / 2
        
        # Return the midpoint as a tuple (x, y)
        return (midpoint_x, midpoint_y)


    def __detect_qr_codes(self, image):
        # Detect QR codes in the image using opencv
        decoded_info, points, _ = self.qr_code_detector.detectAndDecode(image)
        midpoint_x, midpoint_y = self.__corners_to_middlepoint(points)
        self.get_logger().info(f"QR code middle point: ({midpoint_x}|{midpoint_y})")
        return decoded_info, midpoint_x, midpoint_y

    def process_images(self):
        while True:
            captured_image = self.__capture_image()
            if captured_image is not None:
                qr_code_content, midpoint_x, midpoint_y = self.__detect_qr_codes(captured_image)
                if qr_code_content:
                    qr_code_position = [midpoint_x, midpoint_y]
                    
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
