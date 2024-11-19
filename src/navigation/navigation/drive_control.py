import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from agrobot_interfaces.msg import DriveCommand, ToFData
from agrobot_interfaces.action import Center

STABILITY_THRESHOLD = 10

class DriveControl(Node):
    '''
    :author: ADD HERE
    :date: ADD HERE

    Node that runs the drive command controllers.

    Publishers:
        - drive/command (agrobot_interfaces/msg/DriveCommand)

    Subscribers:
        - tof/data (agrobot_interfaces/msg/ToFData)

    Action Servers:
        - control/center (agrobot_interfaces/action/Center)
        - TODO: Add more here? Straight line, turn, etc?
    '''

    def __init__(self):
        super().__init__('drive_control')

        self.drive_pub = self.create_publisher(DriveCommand, 'drive/command', 10)
        self.tof_sub = self.create_subscription(ToFData, 'tof/data', self.tof_callback, 10)
        self.center_action_server = ActionServer(self, Center, 'control/center', self.center_callback)

        # PID control parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

    def tof_callback(self, msg):

        self.get_logger().info('Received ToF data: %s' % msg)
        self.tof_data = msg.data # TODO: Update this

    def center_callback(self, goal_handle):
        '''
        Callback function for the center action server.

        :param goal_handle: The goal handle for the action server.
        :type goal_handle: agrobot_interfaces.action.Center.GoalHandle
        '''
        self.get_logger().info('Received request to center the robot')

        stability_count = 0
        centered = False
        while not centered:
            if self.new_data:

                # Run PID control to center the robot
                pass

                # Check if the robot is centered
                if self.tof_data == "centered":
                    stability_count += 1
                    if stability_count >= STABILITY_THRESHOLD:
                        centered = True
            
                self.new_data = False

        goal_handle.succeed()

        result = Center.Result()
        return result

def main(args=None):
    rclpy.init(args=args)

    drive_control_node = DriveControl()
    rclpy.spin(drive_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drive_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
