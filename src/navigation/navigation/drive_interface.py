import rclpy
from rclpy.node import Node
from agrobot_interfaces.msg import DriveCommand

class DriveInterface(Node):
    '''
    :author: ADD HERE
    :date: ADD HERE

    Node that interfaces with the drive system of the robot.

    Subscribers:
         - drive/command (agrobot_interfaces/msg/DriveCommand)
    '''

    def __init__(self):
        super().__init__('drive_interface')

        self.drive_command_sub = self.create_subscription(DriveCommand, 'drive/command', self.drive_command_callback, 10)

    def drive_command_callback(self, msg):
        '''
        Callback function for the drive command subscriber.

        :param msg: The drive command message.
        :type msg: agrobot_interfaces.msg.DriveCommand
        '''
        self.get_logger().info('Received drive command: %s' % msg)

        # TODO: Interface with the drive system here

def main(args=None):
    rclpy.init(args=args)

    drive_interface_node = DriveInterface()
    rclpy.spin(drive_interface_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drive_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
