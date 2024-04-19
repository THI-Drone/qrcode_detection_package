import sys
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String

from common_package_py.common_node import CommonNode


class QRCodeScannerNode(CommonNode):
    def __init__(self, id: str):
        super().__init__(id)
        self.qr_publisher = self.create_publisher(
            String, 'qr_codes', 10)

    def capture_image(self):
        # For testing purposes the image is not taken from the camera but loaded a test file
        # Path to the image file
        image_path = '/home/ws/src/src/qrcode_detection_package/test_image/test.jpg'
        # Load the image using OpenCV
        captured_image = cv2.imread(image_path)

        return captured_image


    def detect_qr_codes(self, image):
        # implementation of QR code detection using OpenCV
        qr_code_detector = cv2.QRCodeDetector()
        # Detect QR codes in the image
        decoded_info, points, _ = qr_code_detector.detectAndDecode(image)
        qr_codes = []
        if decoded_info:
            qr_codes.append(decoded_info)
        else:
            self.get_logger().info(f"No QR Code found")
        return qr_codes

    def process_images(self):
        while True:
            captured_image = self.capture_image()
            if captured_image is not None:
                qr_codes = self.detect_qr_codes(captured_image)
                if qr_codes:
                    for code in qr_codes:
                        #self.qr_publisher.publish(code)
                        self.get_logger().info(f"Detected QR code: {code}")

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
