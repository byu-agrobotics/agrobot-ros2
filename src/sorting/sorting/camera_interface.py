import rclpy
from rclpy.node import Node

class CameraInterface(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    Node that interfaces with the camera system of the robot.

    TODO: Add here
    '''

    def __init__(self):
        super().__init__('camera_interface')

        # TODO: Add here


def main(args=None):
    rclpy.init(args=args)

    camera_interface_node = CameraInterface()
    rclpy.spin(camera_interface_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
