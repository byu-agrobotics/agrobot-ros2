import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from agrobot_interfaces.msg import ToFData, DriveCommand
from agrobot_interfaces.action import DriveControl

STABILITY_THRESHOLD = 10
CENTERING_THRESHOLD = 1 # 1mm threshold for centering the robot

class DriveController(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    Node that runs the drive command controllers.

    Publishers:
        - drive/command (agrobot_interfaces/msg/DriveCommand)

    Subscribers:
        - tof/data (agrobot_interfaces/msg/ToFData)

    Action Servers:
        - control/center (agrobot_interfaces/action/DriveControl)
        - TODO: Add more here? Straight line, turn, etc?
    '''

    def __init__(self):
        super().__init__('drive_controller')

        self.drive_pub = self.create_publisher(DriveCommand, 'drive/command', 10)
        self.tof_sub = self.create_subscription(ToFData, 'tof/data', self.tof_callback, 10)
        self.center_action_server = ActionServer(self, DriveControl, 'control/center', self.center_callback)

        # PID control parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

    def tof_callback(self, msg):

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
                # pass

                # Run PD control to center the robot
                # Calculate position errors
                forward_error = (self.tof_data.front - self.tof_data.back)
                lateral_error = (self.tof_data.left - self.tof_data.right)

                # Calculate PD outputs
                kp = 0.3  # Proportional gain

                forward_correction = kp * forward_error
                lateral_correction = kp * lateral_error

                # Generate motor commands (differential drive)
                drive_cmd = DriveCommand()
                drive_cmd.left = forward_correction - lateral_correction
                drive_cmd.right = forward_correction + lateral_correction

                # Apply speed limits
                max_speed = 0.5
                drive_cmd.left = max(min(drive_cmd.left, max_speed), -max_speed)
                drive_cmd.right = max(min(drive_cmd.right, max_speed), -max_speed)

                self.drive_pub.publish(drive_cmd)

                # Check if the robot is centered
                if abs(forward_error) < CENTERING_THRESHOLD and abs(lateral_error) < CENTERING_THRESHOLD:
                    stability_count += 1
                    if stability_count >= STABILITY_THRESHOLD:
                        centered = True
            
                self.new_data = False

        goal_handle.succeed()

        result = DriveControl.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)

    drive_controller_node = DriveController()
    rclpy.spin(drive_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drive_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
