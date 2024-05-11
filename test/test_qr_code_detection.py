import pytest
import time
import json
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.msg import Control, QRCodeInfo, JobFinished
from qrcode_detection_package.main import QRCodeScannerNode
from common_package_py.topic_names import TopicNames


def test_qrcode_info_publish():
    executor = SingleThreadedExecutor()

    qr_scanner_node = QRCodeScannerNode("qr_scanner_node")
    assert not qr_scanner_node.active

    test_node = Node("test")

    def job_finished_callback(msg):
        nonlocal qr_scanner_node
        nonlocal test_node
        nonlocal executor

        test_node.get_logger().debug("Got job_finished message")
        assert msg.sender_id == "qr_scanner_node"
        assert msg.error_code == 5

        payload_check = {}
        payload_check["marker"] = "42"
        assert json.loads(msg.payload) == payload_check

        assert not qr_scanner_node.active
        executor.shutdown(0)

    job_finished_sub = test_node.create_subscription(
        JobFinished, TopicNames.JobFinished, job_finished_callback, 10)

    def timer_callback():
        nonlocal qr_scanner_node
        test_node.get_logger().debug("Sending job_finished successfull message")

        payload = {}
        payload["marker"] = "42"
        qr_scanner_node._job_finished_custom_(5, payload)

    job_finished_timer = qr_scanner_node.create_timer(
        0.1, timer_callback)

    executor.add_node(qr_scanner_node)
    executor.add_node(test_node)

    executor.spin()
    del executor


def test_activate_with_control_message():
    executor = SingleThreadedExecutor()

    qr_scanner_node = QRCodeScannerNode("qr_scanner_node")
    assert not qr_scanner_node.active

    test_node = Node("test")

    control_publisher = test_node.create_publisher(
        Control, TopicNames.Control, 10)

    def timer_callback():
        nonlocal test_node
        test_node.get_logger().debug("Sending control message to activate QRCodeScannerNode")
        print("Test print")
        msg = Control()
        msg.target_id = "qr_scanner_node"
        msg.active = True
        msg.payload = ""

        control_publisher.publish(msg)

    def end_timer_callback():
        assert qr_scanner_node.active
        executor.shutdown(0)

    publish_control_timer = test_node.create_timer(
        0.1, timer_callback)

    end_timer = test_node.create_timer(
        0.2, end_timer_callback)

    executor.add_node(qr_scanner_node)
    executor.add_node(test_node)

    executor.spin()

    del executor
