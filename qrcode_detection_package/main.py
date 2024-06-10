import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
import rclpy
import time
import subprocess
import threading
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import logging
import cv2
from cv2.typing import MatLike
from std_msgs.msg import String
from enum import Enum
from typing import Union, Tuple, List
from common_package_py.common_node import CommonNode
from common_package_py.topic_names import TopicNames
from interfaces.msg import QRCodeInfo
from interfaces.msg import Control
from qreader import QReader
import signal


class NodeState(Enum):
    READY = 0
    SEARCHING = 1


class CaptureImageMethod(Enum):
    # until tested properly different image taking methods are implemented and can be configured
    # 0 = load stored test image (for testing purposes)
    # 1 = use OpenCV to get image from camera (this is the wanted solution)
    # 2 = use libcamera shell script to take photo and load it (fallback solution)
    # 3 = use PiCam2 lib to capture image as numpy array (best working solution)
    LOADIMAGE = 0
    OPENCV = 1
    LIBCAMERA = 2
    PICAM = 3


class NoQRCodeDetectedError(Exception):
    pass


class QRCodeScannerNode(CommonNode):
    def __init__(self, id: str) -> None:
        """
        Constructor of the QRCodeScannerNode class.

        This method initializes the object properties for the QRCodeSearch object.
        It sets the initial state to "ready" and configures the subscription and publisher objects for control 
        and QR code information. Additionally, it initializes the QR code detector from OpenCV.
        The method also includes a configuration variable for selecting the QR code detection method.
        This method is called automatically when creating a QRCodeSearch object.

        Args: 
            id: The node_id.

        Returns: 
            None
        """
        super().__init__(id)
        self.set_state(NodeState.READY)
        self.control_subscription = self.create_subscription(
            Control,
            TopicNames.Control,
            self.__callback_control,
            10
        )
        
        self.is_busy = False
        
        #self.qr_code_detector = cv2.QRCodeDetector()
        self.qreader = QReader(model_size = 'n', min_confidence = 0.5)
        
        self.qr_publisher = self.create_publisher(
            QRCodeInfo, TopicNames.QRCodeInfo, 10)

        self.declare_parameter('sim', True)
        # read sim parameter
        sim_param = self.get_parameter('sim').get_parameter_value().bool_value
        self.picam2 = None
        # configure image capturing method
        if (sim_param):
            self.config_detection_method = CaptureImageMethod.LOADIMAGE
        else:
            self.config_detection_method = CaptureImageMethod.PICAM

        if (self.config_detection_method == CaptureImageMethod.PICAM):
            # init picam
            from picamera2 import Picamera2
            self.picam2 = Picamera2()
            config = self.picam2.create_still_configuration(main={"size": (2048, 1536), 'format': 'RGB888',})
            #config = picam2.create_still_configuration(main={"size": (1000, 750), 'format': 'RGB888',})
            self.picam2.configure(config)
            self.picam2.start()

        self.declare_parameter('IMG_PATH', 'images/default')
        # read imgage path parameter
        self.image_path = self.get_parameter('IMG_PATH').get_parameter_value().string_value
        
        if not os.path.exists(self.image_path):
            os.makedirs(self.image_path)

        # count detected markers
        self.numDetMark = 0

        self.reentrantCallback = ReentrantCallbackGroup()
        
        main_timer = self.create_timer(
            1.5, self.main, callback_group=self.reentrantCallback)

    def __callback_control(self, control_msg: Control) -> None:
        """
        Callback function for receiving control messages.

        This callback function is triggered when a control message is received.
        It checks if the target ID of the control message matches the QR code scanner node.
        If it does, the function logs a message indicating the receipt of the control message.
        Depending on the 'active' flag in the control message, the function either activates
        or deactivates the QR code scanner node by calling the respective private methods.

        Args: 
            control_msg: The received control message.

        Returns: 
            None
        """
        if (control_msg.target_id == self.get_name()):
            self.get_logger().info("Received control message")
            if (control_msg.active):
                self._activate_()
            else:
                self._deactivate_()

    def set_state(self, state: NodeState) -> None:
        """
        Sets the state of the node to the new state provided.

        Args: 
            state: The new state to set the node to.

        Returns: 
            None
        """
        self.node_state = state
        self.get_logger().info(f"Set node to state {state}")

    def __capture_image(self) -> Union[None, MatLike]:
        """
        Capture an image either from the camera or a test image, depending on the configuration.

        This method captures an image either from the camera or from a test image,
        depending on the configuration specified by 'config_detection_method'.
        - If 'config_detection_method' is set to 0, a test image is loaded from a predefined path.
        - If 'config_detection_method' is set to 1, an image is captured from the camera.
        - If 'config_detection_method' is set to 2, an image is captured using the libcamera command line tool.
        The captured image is returned as an OpenCV MatLike object.

        Returns: 
            OpenCV MatLike representing the captured image.
        """
        captured_image = None
        # Check which detection style should be used
        match self.config_detection_method:
            case CaptureImageMethod.LOADIMAGE:
                try:
                    script_dir = os.path.dirname(os.path.realpath(__file__))
                    # use path of different images for sim
                    image_num = self.numDetMark % 4
                    #rel_path = "../test_image/qrtest_content_" + \
                    #    str(image_num) + ".png"
                    rel_path = "/home/ws/src/qrcode_detection_package/test_image/difficult/80_not_detected.jpg"
                    image_path = os.path.join(
                        script_dir, rel_path)
                    self.get_logger().info(f"Try loading from {image_path}")
                    # Load the test image
                    #captured_image = cv2.imread(image_path)
                    captured_image = cv2.imread(rel_path)
                    self.numDetMark += 1
                except:
                    self.get_logger().info("Image could not be loaded")
                    return None

            case CaptureImageMethod.OPENCV:
                # Initialize a VideoCapture object for the camera
                cap = cv2.VideoCapture(0)

                # To-Do: Evaluate influence of different camera settings to quality and performance on the real hardware
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 4056)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 3040)

                # Check if the VideoCapture object was opened successfully
                if not cap.isOpened():
                    self.get_logger().info("Unable to open camera OpenCV")
                    return None
                else:
                    # Capture an image from the camera
                    ret, img = cap.read()

                    # Check if the image was captured successfully
                    if not ret:
                        self.get_logger().info("OpenCV could not take picture")
                        return None
                    else:
                        captured_image = img

                # Release the VideoCapture object
                cap.release()

            case CaptureImageMethod.LIBCAMERA:
                try:
                    # set image path with timestamp as name
                    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
                    image_path = f"/image/{timestamp}.jpg"
                    # execute libcamera command to capture imagae
                    command = ["libcamera-jpeg", "-o", image_path]
                    subprocess.run(command)
                    captured_image = cv2.imread(image_path)
                except:
                    self.get_logger().info("libcamera could not take picture")
                    return None

            case CaptureImageMethod.PICAM:
                if self.picam2 is not None:
                    try:
                        captured_image = self.picam2.capture_array("main")
                        self.numDetMark += 1
                        #if self.numDetMark % 10 == 0:
                        img_path = f'{self.image_path}/{self.numDetMark}_captured_image.jpg'
                        cv2.imwrite(img_path, captured_image)
                        self.get_logger().info(
                            f"Saved captured image as: {img_path}")
                    except:
                        self.get_logger().info("PiCam could not take picture")
                        return None
                else:
                    self.get_logger().info("PiCam is not initialized")
                    return None

        return captured_image

    def __corners_to_middlepoint(self, points: List[Tuple[int, int]]) -> Tuple[float, float]:
        """
        Calculate the geometric midpoint of a rectangle formed by 4 points.

        Args: 
            points: A list of tuples representing the coordinates of the 4 points.

        Returns: 
            A tuple representing the geometric midpoint of the rectangle.
        """
        # Extract the coordinates of the diagonal points
        top_left = points[0][0]
        bottom_right = points[0][2]

        # Calculate the geometric midpoint
        midpoint_x = (top_left[0] + bottom_right[0]) / 2
        midpoint_y = (top_left[1] + bottom_right[1]) / 2

        # Return the midpoint as a tuple (x, y)
        return (midpoint_x, midpoint_y)

    def __relative_midpoint(self, midpoint_x: float, midpoint_y: float, img_width: int, img_height: int) -> Tuple[float, float]:
        """
        Calculate the position of the QR code relative to the middle of the image in percent from 
        -100% to 100% relative to image height and width.

        This function calculates the position of the QR code relative to the middle of the image.
        The position is represented in percent from -100% to 100% relative to the image height and width.

        Args: 
            midpoint_x: The x-coordinate of the absolute midpoint of the QR code.
            midpoint_y: The y-coordinate of the absolute midpoint of the QR code.
            image_width: The width of the image.
            image_height: The height of the image.

        Returns: 
            A tuple representing the relative position of the QR code to the middle of the picture.
        """
        # calculation of y requires inversed logic due to inversed OpenCV image y-coordinates
        rel_midpoint_x_percent = (
            (midpoint_x - img_width / 2) / (img_width / 2)) * 100
        rel_midpoint_y_percent = - \
            ((midpoint_y - img_height / 2) / (img_height / 2)) * 100

        return (rel_midpoint_x_percent, rel_midpoint_y_percent)

    def __call_qreader(self, image:MatLike, decoded_info_list):
        try:
            #decoded_info, points, _ = self.qr_code_detector.detectAndDecode(
            #    image)
            
            decoded_info = self.qreader.detect_and_decode(image=image)
            decoded_info_list.append(decoded_info[0])
        except:
            pass
            #raise NoQRCodeDetectedError("Exception while detecting QR Code")

    def __detect_qr_codes(self, image: MatLike) -> Tuple[str, float, float]:
        """
        Detects QR codes in the provided image and calculates the midpoint of the bounding rectangle.

        This method utilizes the OpenCV library to detect and decode QR codes in the provided image.
        If a QR code is successfully decoded, its information along with the geometric midpoint of its bounding rectangle
        are logged. Additionally, the relative midpoint coordinates, relative to the center of the image, are calculated
        and logged. If no QR code is detected, the method returns an error message along with default midpoint coordinates.

        Args: 
            image: The image in which to detect QR codes in the OpenCV MatLike format.

        Returns: 
            A tuple containing:
                - The decoded information from the QR code, or "Error" if no QR code is detected.
                - The x-coordinate of the geometric midpoint of the detected QR code.
                - The y-coordinate of the geometric midpoint of the detected QR code.
        """
        rel_midpoint_x = 0
        rel_midpoint_y = 0

        decoded_info_list = []
        # detect and decode QR codes in the image using OpenCV library

        thread = threading.Thread(target=self.__call_qreader, args=(image, decoded_info_list))

        # Starte den Thread
        thread.start()

        # Optional: Warte, bis der Thread beendet ist
        thread.join()


        #self.__call_qreader(image, decoded_info_list)

        #try:
            #decoded_info, points, _ = self.qr_code_detector.detectAndDecode(
            #    image)
        #    decoded_info = self.qreader.detect_and_decode(image=image)
        #except:
        #    raise NoQRCodeDetectedError("Exception while detecting QR Code")

        # check if QR code was successfully decoded
        if (len(decoded_info_list) > 0):
            # Log QR-Code content
            if (decoded_info_list[0] != None):
                self.get_logger().info(f"Detected QR code: {decoded_info_list[0]}")

            # Calculate the geometric midpoint of the bounding rectangle
            #midpoint_x, midpoint_y = self.__corners_to_middlepoint(points)

            # Log the midpoint coordinates
            #self.get_logger().info(
            #    f"QR code middle point: ({midpoint_x}|{midpoint_y})")

            # Get midpoints relative to the center of the picture
            #img_height, img_width = image.shape[:2]
            #rel_midpoint_x, rel_midpoint_y = self.__relative_midpoint(
            #    midpoint_x, midpoint_y, img_width, img_height)
            # Log the relative midpoint coordinates
            #self.get_logger().info(
            #    f"Relative QR code middle point: ({rel_midpoint_x}|{rel_midpoint_y})")
            else:
                raise NoQRCodeDetectedError("Decoded QR Code content was None")
        else:
            # if no QR-Code was detected raise Excpetion
            raise NoQRCodeDetectedError("No QR code was detected")

        # return content and relative position of the QR-Code
        return decoded_info_list[0]

    def scan_for_qr_code(self) -> None:
        """
        Captures an image, detects QR code, and publishes information about it.

        This method captures a single image either from the camera or a test image,
        detects QR codes in the captured image, and publishes information about them on the 'qr_codes' topic.
        If a valid QR code is detected, the method saves the image containing the QR code and sends a job
        finished message indicating successful completion of the task. If no QR code is found or if there are
        errors in capturing the image, appropriate log messages are generated.

        Returns: 
            None
        """
        # capture image
        captured_image = self.__capture_image()
        # check if an image was captured
        if captured_image is not None:
            try:
                # use OpenCV to detect qr codes in the image
                qr_code_content = self.__detect_qr_codes(captured_image)
                # check if qr_code_really has content
                if qr_code_content:
                    # if a QR-Code was successfully detected save the image and publish contents on the topic

                    # save the image that contains successfully decoded QR-Code
                    img_path = f'{self.image_path}/{self.numDetMark}_successfull_detected.jpg'
                    cv2.imwrite(img_path, captured_image)
                    self.get_logger().info(
                        f"Saved image of detected QR-Code as: {img_path}")

                    # create QRCodeInfo message to publish on qr_codes topic
                    msg = QRCodeInfo()
                    msg.time_stamp = self.get_clock().now().to_msg()
                    msg.sender_id = self.get_name()
                    msg.qr_code_content = qr_code_content
                    msg.qrcode_position_x = 0.0
                    msg.qrcode_position_y = 0.0

                    self.qr_publisher.publish(msg)
                    self.get_logger().info("Published QR code info")
                    self.get_logger().info(f"Content = {qr_code_content}")

                    # create dict for sending job finished message
                    payload = {"marker": str(qr_code_content)}
                    self._job_finished_custom_(
                        CommonNode.EXIT_SUCCESS, payload)
                    self.get_logger().info("Published job finished message")

                else:
                    self.get_logger().info("No QR Code found")
            except NoQRCodeDetectedError as error:
                # QR code detector raised exception
                self.get_logger().info(
                    f"No QR Code could be found in the image: {error}")
        else:
            self.get_logger().info("Could not capture image")

    def main(self) -> None:
        """
        The main function initializes the node and runs the state machine over the lifetime of the node.

        This function initializes the ROS 2 node, creates an instance of the QRCodeScannerNode class,
        and starts the image processing loop. Then the state machine decides what the node does depending on the internal state.
        The states are:
        - "ready": The node is running and waits for a control message which activates it. There happens no image
                capturing or qr code scanning in this state
        - "searching": in this state the node continuously takes images and uses the OpenCV library to scan them for
                    QR-Codes. If a valid code is found, the nodes publishes its contents and position and switches
                    back to the state "ready"
        It handles the cleanup operations before shutting down the node.

        Args: 
            args: Command-line arguments. Default is None.

        Returns: 
            None
        """
        if not self.is_busy:
            self.is_busy = True
            # start excecuting state machine until node gets destroyed
            match self.node_state:
                # check if node is in state "ready"
                # in this state the node waits for the control message to activate the node
                case NodeState.READY:
                    # if the node got activated it sets its internal state to searching
                    if (self.active):
                        self.set_state(NodeState.SEARCHING)
                # check if node is in state "searching"
                # in this state the node will continue to capture images and scan them for qrcodes until node gets deactivated
                case NodeState.SEARCHING:
                    # if the node is active start qr-code search
                    if (self.active):
                        try:
                            self.scan_for_qr_code()
                        except Exception as error:
                            # handle the exception
                            self.get_logger().info(
                                f"Error ocurred when scanning QR Code: {error}")
                    # if the node gets deactivated in searching the state changes to "ready"
                    else:
                        self.set_state(NodeState.READY)
                case _:
                    # if the node is not in a valid state send error code to mission control and destroy node
                    self.get_logger().info(
                        "QR-Code detection node is in unknown state")
                    self._job_finished_error_msg_(
                        "Node shut down because it is in unknown state")
            self.is_busy = False
        else:
            self.get_logger().info(
                        "QR-Code node is busy, skip iteration")


def main(args=None) -> None:
    rclpy.init(args=args)
    node_id = 'qr_code_scanner_node'
    
    try:
        qr_code_scanner_node = QRCodeScannerNode(node_id)
    except Exception as e:
        logging.getLogger('EMERGENCY').error(
            f"Error occured when creating QRCodeScannerNode: {e}")
        os._exit(1)

    try:
        #executor = SingleThreadedExecutor()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(qr_code_scanner_node)
        executor.spin()
    except Exception as e:
        qr_code_scanner_node.get_logger().error(
            f"Error occured when creating executor: {e}")
        raise e
    finally:
        del executor
        qr_code_scanner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
