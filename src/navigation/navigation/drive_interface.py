import time
import rclpy
from rclpy.node import Node
from agrobot_interfaces.msg import DriveCommand
from roboclaw_3 import Roboclaw

ROBOCLAW_ADDR = 0x80
ROBOCLAW_BAUD = 115200
ROBOCLAW_NAME = "/dev/roboclaw"
TIMEOUT_THRESHOLD = 3 # seconds

class DriveInterface(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    Node that interfaces with the drive system of the robot.

    Subscribers:
         - drive/command (agrobot_interfaces/msg/DriveCommand)
    '''

    def __init__(self):
        super().__init__('drive_interface')

        self.drive_command_sub = self.create_subscription(DriveCommand, 'drive/command', self.drive_command_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.roboclaw = Roboclaw(ROBOCLAW_NAME, ROBOCLAW_BAUD) # TODO: Add udev rule for roboclaw?
        self.roboclaw.Open()
        self.roboclaw.ForwardM1(ROBOCLAW_ADDR, 0) # left side
        self.roboclaw.ForwardM2(ROBOCLAW_ADDR, 0) # right side

        self.last_time = 0

    def drive_command_callback(self, msg):
        '''
        Callback function for the drive command subscriber.

        :param msg: The drive command message.
        :type msg: agrobot_interfaces.msg.DriveCommand
        '''

        if msg.left_speed >= 0:
            self.roboclaw.ForwardM1(ROBOCLAW_ADDR, int(msg.left_speed)) # drive forward
        else:
            self.roboclaw.BackwardM1(ROBOCLAW_ADDR, -1 * int(self.left_speed)) # drive backward

        if self.right_speed >= 0:
            self.roboclaw.ForwardM2(ROBOCLAW_ADDR, int(self.right_speed)) # drive forward
        else:
            self.roboclaw.BackwardM2(ROBOCLAW_ADDR, -1 * int(self.right_speed)) # drive backward

        self.last_time = time.time()

    def timer_callback(self):
        '''
        Timer callback function to check for expired commands.
        '''
        
        if time.time() - self.last_time > TIMEOUT_THRESHOLD: # in seconds
            self.get_logger().warn('Drive command timeout, killing motors')
            self.roboclaw.ForwardM1(ROBOCLAW_ADDR, 0)
            self.roboclaw.ForwardM2(ROBOCLAW_ADDR, 0)

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
