import sys
import rclpy
from rclpy.node import Node

from common_package_py.common_node import CommonNode


class MinimalPublisher(CommonNode):
    """A class that represents a minimal node.
    """
    
    def __init__(self, id: str):
        """Creates a new MinimalPublisher node.

        Args:
            id (str): Unique node id
        """
        
        super().__init__(id)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher(sys.argv[1])

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
