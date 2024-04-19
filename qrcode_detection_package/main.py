import sys
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String

from common_package_py.common_node import CommonNode


class QRCodeScannerNode(CommonNode):
    def __init__(self, id: str):
        super().__init__(id)
        self.qr_code_detector = cv2.QRCodeDetector()
        self.qr_publisher = self.create_publisher(
            String, 'qr_codes', 10)

    def _capture_image(self):
        # For testing purposes the image is not taken from the camera but loaded a test file
        # Path to the image file
        image_path = 'src/qrcode_detection_package/test_image/test.jpg'
        # Load the image using OpenCV
        captured_image = cv2.imread(image_path)
        return captured_image


    def _detect_qr_codes(self, image):
        # Detect QR codes in the image using opencv
        decoded_info, points, _ = self.qr_code_detector.detectAndDecode(image)
        
        return decoded_info

    def process_images(self):
        while True:
            captured_image = self._capture_image()
            if captured_image is not None:
                qr_code_content = self._detect_qr_codes(captured_image)
                if qr_code_content:
                    #self.qr_publisher.publish(code)
                    self.get_logger().info(f"Detected QR code: {qr_code_content}")
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
