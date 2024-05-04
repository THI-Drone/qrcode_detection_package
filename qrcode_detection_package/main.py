import sys
import rclpy
import time
import subprocess
from rclpy.node import Node
import cv2
from std_msgs.msg import String

from common_package_py.common_node import CommonNode
from interfaces.msg import QRCodeInfo
from interfaces.msg import Control


class QRCodeScannerNode(CommonNode):
    def __init__(self, id: str):
        super().__init__(id)
        self.__set_state("ready")
        self.control_subscription = self.create_subscription(
            Control,
            "control",
            self.__callback_control,
            10
        )
        self.qr_code_detector = cv2.QRCodeDetector()
        self.qr_publisher = self.create_publisher(QRCodeInfo, 'qr_codes', 10)
        
        # until tested properly different detection methods are implemented and can be configured here
        # 0 = load stored test image (for testing purposes)
        # 1 = use OpenCV to get image from camera (this is the wanted solution)
        # 2 = use libcamera shell script to take photo and load it (fallback solution)
        
        self.config_detection_method = 0

    def __callback_control(self, control_msg):
        if (control_msg.target_id == self.node_id):
            self.get_logger().info("Recieved control message")
            if (control_msg.active):
                self._activate_()
            else:
                self._deactivate_()
        
    def __set_state(self, state):
        self.nodeState = state
        
        
    def __capture_image(self):
        """
        @brief Capture an image either from the camera or a test image, depending on the configuration.

        @return OpenCV MatLike representing the captured image.
        """
        import cv2
        
        # Check which detection style should be used
        if (self.config_detection_method == 0):
            # Path to the test image
            test_image_path = 'src/qrcode_detection_package/test_image/test3.png'
            
            # Load the test image
            captured_image = cv2.imread(test_image_path)
        elif (self.config_detection_method == 1):
            # Initialize a VideoCapture object for the camera
            cap = cv2.VideoCapture(0)
            
            # To-Do: Evaluate influence of different camera settings to quality and performance on the real hardware
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4056)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 3040)
            
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
        @brief calculate the position of the qrcode relative to the middle of the image in percent from 
               -100% to 100% relative to image height and width

        @param absolute midpoint coordinates and image width and height

        @return A tuple representing the relative position of the qr code to the middle of the picture
        """
        # calculation of y requires inversed logic due to inversed OpenCV image y-coordinates        
        rel_midpoint_x_percent = ((midpoint_x - img_width / 2) / (img_width / 2)) * 100
        rel_midpoint_y_percent = -((midpoint_y - img_height / 2) / (img_height / 2)) * 100
        
        return (rel_midpoint_x_percent, rel_midpoint_y_percent)


    def __detect_qr_codes(self, image):
        """
        @brief Detect QR codes in the image and calculate the midpoint of the bounding rectangle.

        @param image: The image in which to detect QR codes.

        @return A tuple containing:
                - The decoded information from the QR code.
                - The x-coordinate of the geometric midpoint of the QR code.
                - The y-coordinate of the geometric midpoint of the QR code.
        """
        # detect and decode QR codes in the image using OpenCV library
        decoded_info, points, _ = self.qr_code_detector.detectAndDecode(image)
        
        # check if QR code was successfully decoded
        if (len(decoded_info) > 0):
            # Log QR-Code content
            self.get_logger().info(f"Detected QR code: {decoded_info}")
            
            # Calculate the geometric midpoint of the bounding rectangle
            midpoint_x, midpoint_y = self.__corners_to_middlepoint(points)
            
            # Log the midpoint coordinates
            self.get_logger().info(f"QR code middle point: ({midpoint_x}|{midpoint_y})")
            
            # Get midpoints relative to the center of the picture
            img_height, img_width = image.shape[:2]
            rel_midpoint_x, rel_midpoint_y = self.__relative_midpoint(midpoint_x, midpoint_y, img_width, img_height)
            
            # Log the relative midpoint coordinates
            self.get_logger().info(f"Relative QR code middle point: ({rel_midpoint_x}|{rel_midpoint_y})")
        else:
            # if no QR-Code was detected set return values to error standard
            decoded_info = "Error"
            rel_midpoint_x = 0
            rel_midpoint_y = 0
        
        # return content and relative position of the QR-Code
        return decoded_info, rel_midpoint_x, rel_midpoint_y

    def process_images(self):
        """
        @brief This method continuously captures images, detects QR codes in the images, and publishes
        information about the detected QR codes.

        """
        # capture image
        captured_image = self.__capture_image()
        # check if an image was captured
        if captured_image is not None:
            # use OpenCV to detect qr codes in the image
            qr_code_content, qrcode_center_x, qrcode_center_y = self.__detect_qr_codes(captured_image)
            
            # check if a valid QR-Code has been found
            if qr_code_content and qr_code_content != "Error":
                # if a QR-Code was successfully detected, publish contents on the topic
                
                #create QRCodeInfo message to publish on qr_codes topic
                msg = QRCodeInfo()
                msg.sender_id = self.node_id
                msg.qr_code_content = qr_code_content
                msg.qrcode_position_x = qrcode_center_x
                msg.qrcode_position_y = qrcode_center_y
                
                self.qr_publisher.publish(msg)
                self.get_logger().info("Published QR code info")
                
            else:
                self.get_logger().info("No QR Code found")
        else:
            self.get_logger().info("Could not take image")


def main(args=None):
    """
    @brief The main function initialises the node and runs the state machine over the lifetime of the node

    This function initializes the ROS 2 node, creates an instance of the QRCodeScannerNode class,
    and starts the image processing loop. 
    
    Then the state machine decides what the node does depending on the internal state.
    The states are:
    - "ready": The node is running and waits for a control message which activates it. There happens no image
               capturing or qr code scanning in this state
    - "searching": in this state the node continously takes images and uses the OpenCV library to scan them for
                   QR-Codes. If a valid code is found, the nodes publishes its contents and position and switches
                   back to the state "ready"
    
    It handles the cleanup operations before shutting down the node.

    @param args: Command-line arguments. Default is None.
    """
    rclpy.init(args=args)
    node_id = 'qr_code_scanner_node'
    qr_code_scanner_node = QRCodeScannerNode(node_id)
    
    # start excecuting state machine until node gets destroyed
    while True:
        # check if node is in state "ready"
        # in this state the node waits for the control message to activate the node
        if (qr_code_scanner_node.nodeState == "ready"):
            # if the node got activated it sets its internal state to searching
            if (qr_code_scanner_node.active()):
                qr_code_scanner_node.__set_state("searching")
        # check if node is in state "searching"
        # in this state the node will continue to capture images and scan them for qrcodes until node gets deactivated
        elif (qr_code_scanner_node.nodeState == "searching"):
            # if the node is active start qr-code search
            if (qr_code_scanner_node.active()):
                try:
                    qr_code_scanner_node.process_images()
                except:
                    qr_code_scanner_node.get_logger().info("Error ocurred when scanning QR Code")     
            # if the node gets deactivated in searching the state changes to "ready"
            else:
                qr_code_scanner_node.__set_state("ready")
        else:
            # if the node is not in a valid state send error code to mission control and destroy node
            qr_code_scanner_node.get_logger().info("QR-Code detection node is in unknown state")
            qr_code_scanner_node._job_finished_error_msg_("Node shut down because it is in unknwon state")
            break
        
    # destroy the node
    qr_code_scanner_node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()
