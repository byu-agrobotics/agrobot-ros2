# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionServer
# from agrobot_interfaces.msg import ToFData, DriveCommand
# from agrobot_interfaces.action import DriveControl

# STABILITY_THRESHOLD = 10
# CENTERING_THRESHOLD = 1 # 1mm threshold for centering the robot

# class DriveController(Node):
#     '''
#     :author: Nelson Durrant
#     :date: November 2024

#     Node that runs the drive command controllers.

#     Publishers:
#         - drive/command (agrobot_interfaces/msg/DriveCommand)

#     Subscribers:
#         - tof/data (agrobot_interfaces/msg/ToFData)

#     Action Servers:
#         - control/center (agrobot_interfaces/action/DriveControl)
#         - TODO: Add more here? Straight line, turn, etc?
#     '''

#     def __init__(self):
#         super().__init__('drive_controller')

#         self.drive_pub = self.create_publisher(DriveCommand, 'drive/command', 10)
#         self.tof_sub = self.create_subscription(ToFData, 'tof/data', self.tof_callback, 10)
#         self.center_action_server = ActionServer(self, DriveControl, 'control/center', self.center_callback)

#         # PID control parameters
#         self.declare_parameter('kp', 1.0)
#         self.declare_parameter('ki', 0.0)
#         self.declare_parameter('kd', 0.0)
#         self.declare_parameter('max_output', 100.0)
#         self.declare_parameter('min_output', -100.0)

#         # initialize variables



#     def tof_callback(self, msg):
#         self.tof_data = msg  
#         self.new_data = True


#     def center_goal_callback(self, goal_request):
#         """
#         Called when a goal is received. Decide whether to accept or reject.
#         """
#         self.get_logger().info('Received center robot goal request')
        
#         # Check if we're already processing a goal
#         if self.centering_active:
#             return GoalResponse.REJECT
        
#         return GoalResponse.ACCEPT

#     def center_cancel_callback(self, goal_handle):
#             """
#             Called when a goal cancellation is requested
#             """
#             self.get_logger().info('Received request to cancel centering')
            
#             # Stop the centering process
#             if self.center_timer:
#                 self.center_timer.cancel()
#                 self.center_timer = None
            
#             self.centering_active = False
#             self.stop_robot()
            
#             return CancelResponse.ACCEPT


    

#     def center_callback(self, goal_handle):
#         '''
#         Callback function for the center action server.

#         :param goal_handle: The goal handle for the action server.
#         :type goal_handle: agrobot_interfaces.action.Center.GoalHandle
#         '''

#         self.get_logger().info('Received request to center the robot')

#         stability_count = 0
#         centered = False
#         while not centered:
#             if self.new_data:

#                 # Run PD control to center the robot
#                 # Calculate position errors
#                 forward_error = (self.tof_data.front - self.tof_data.back)
#                 lateral_error = (self.tof_data.left - self.tof_data.right)

#                 # Calculate PD outputs
#                 kp = 0.3  # Proportional gain

#                 forward_correction = kp * forward_error
#                 lateral_correction = kp * lateral_error

#                 # Generate motor commands (differential drive)
#                 drive_cmd = DriveCommand()
#                 drive_cmd.left = forward_correction - lateral_correction
#                 drive_cmd.right = forward_correction + lateral_correction

#                 # Apply speed limits
#                 max_speed = 0.5
#                 drive_cmd.left = max(min(drive_cmd.left, max_speed), -max_speed)
#                 drive_cmd.right = max(min(drive_cmd.right, max_speed), -max_speed)

#                 self.drive_pub.publish(drive_cmd)

#                 # Check if the robot is centered
#                 if abs(forward_error) < CENTERING_THRESHOLD and abs(lateral_error) < CENTERING_THRESHOLD:
#                     stability_count += 1
#                     if stability_count >= STABILITY_THRESHOLD:
#                         centered = True
            
#                 self.new_data = False

#         goal_handle.succeed()

#         result = DriveControl.Result()
#         result.success = True
#         return result

# def main(args=None):
#     rclpy.init(args=args)

#     drive_controller_node = DriveController()
#     rclpy.spin(drive_controller_node)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     drive_controller_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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

        # Use ReentrantCallbackGroup to allow concurrent callbacks
        cb_group = ReentrantCallbackGroup()

        # Initialize publishers and subscribers
        self.drive_pub = self.create_publisher(DriveCommand, 'drive/command', 10)
        self.tof_sub = self.create_subscription(
            ToFData, 
            'tof/data', 
            self.tof_callback, 
            10,
            callback_group=cb_group
        )

        # Initialize action server with proper callbacks
        self.center_action_server = ActionServer(
            self,
            DriveControl,
            'control/center',
            execute_callback=self.center_execute_callback,
            goal_callback=self.center_goal_callback,
            cancel_callback=self.center_cancel_callback,
            callback_group=cb_group
        )

        # PID control parameters from ROS parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('max_output', 100.0)
        self.declare_parameter('min_output', -100.0)
        
        # Initialize data storage
        self.tof_data = None
        self.new_data = False
        
        # For PID control
        self.prev_forward_error = 0.0
        self.prev_lateral_error = 0.0
        self.forward_integral = 0.0
        self.lateral_integral = 0.0
        
        # Control execution
        self.centering_active = False
        self.center_timer = None
        self.stability_count = 0
        self.current_goal_handle = None

    def tof_callback(self, msg):
        # Store the ToF data correctly
        self.tof_data = msg  # Assuming msg has front, back, left, right attributes
        self.new_data = True

    def center_goal_callback(self, goal_request):
        """
        Called when a goal is received. Decide whether to accept or reject.
        """
        self.get_logger().info('Received center robot goal request')
        
        # Check if we're already processing a goal
        if self.centering_active:
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def center_cancel_callback(self, goal_handle):
        """
        Called when a goal cancellation is requested
        """
        self.get_logger().info('Received request to cancel centering')
        
        # Stop the centering process
        if self.center_timer:
            self.center_timer.cancel()
            self.center_timer = None
        
        self.centering_active = False
        self.stop_robot()
        
        return CancelResponse.ACCEPT

    def center_execute_callback(self, goal_handle):
        """
        Called when a goal is accepted. This is where the actual execution happens.
        """
        self.get_logger().info('Executing center robot goal')
        
        # Store the goal handle for sending feedback
        self.current_goal_handle = goal_handle
        self.centering_active = True
        self.stability_count = 0
        
        # Reset PID control values
        self.forward_integral = 0.0
        self.lateral_integral = 0.0
        self.prev_forward_error = 0.0
        self.prev_lateral_error = 0.0
        
        # Create a timer that will repeatedly call our control function
        # Use a reasonable frequency (e.g., 10Hz)
        self.center_timer = self.create_timer(0.1, self.center_control_iteration)
        
        # Wait for the centering to complete or be cancelled
        while self.centering_active and rclpy.ok():
            # Non-blocking sleep to allow other callbacks to run
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                self.center_timer.cancel()
                self.center_timer = None
                self.centering_active = False
                self.stop_robot()
                goal_handle.canceled()
                return DriveControl.Result(success=False)
        
        # Check success condition
        result = DriveControl.Result()
        if not self.centering_active:  # If we exited due to cancellation
            result.success = False
        else:
            result.success = True
            self.center_timer.cancel()
            self.center_timer = None
            self.centering_active = False
        
        return result

    def center_control_iteration(self):
        """
        One iteration of the centering control loop
        """
        if not self.tof_data or not self.new_data:
            return
            
        # Get PID parameters from ROS parameters
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_output = self.get_parameter('max_output').value
        min_output = self.get_parameter('min_output').value
        
        # Calculate position errors
        forward_error = (self.tof_data.front - self.tof_data.back)
        lateral_error = (self.tof_data.left - self.tof_data.right)
        
        # Calculate PID outputs for forward control
        self.forward_integral += forward_error * 0.1  # dt = 0.1 (timer period)
        forward_derivative = (forward_error - self.prev_forward_error) / 0.1
        
        # Calculate PID outputs for lateral control
        self.lateral_integral += lateral_error * 0.1
        lateral_derivative = (lateral_error - self.prev_lateral_error) / 0.1
        
        # Apply PID formula
        forward_correction = (kp * forward_error + 
                             ki * self.forward_integral + 
                             kd * forward_derivative)
        
        lateral_correction = (kp * lateral_error + 
                             ki * self.lateral_integral + 
                             kd * lateral_derivative)
        
        # Generate motor commands (differential drive)
        drive_cmd = DriveCommand()
        drive_cmd.left = forward_correction - lateral_correction
        drive_cmd.right = forward_correction + lateral_correction
        
        # Apply speed limits
        max_speed = 0.5  # Could be a parameter as well
        drive_cmd.left = max(min(drive_cmd.left, max_speed), -max_speed)
        drive_cmd.right = max(min(drive_cmd.right, max_speed), -max_speed)
        
        # Publish command
        self.drive_pub.publish(drive_cmd)
        
        # Save current errors for next iteration
        self.prev_forward_error = forward_error
        self.prev_lateral_error = lateral_error
        
        # Reset the new_data flag
        self.new_data = False
        
        # Check if the robot is centered
        if abs(forward_error) < CENTERING_THRESHOLD and abs(lateral_error) < CENTERING_THRESHOLD:
            self.stability_count += 1
            if self.stability_count >= STABILITY_THRESHOLD:
                self.centering_active = False
        else:
            self.stability_count = 0
        
        # Send feedback about current progress
        if self.current_goal_handle is not None and self.centering_active:
            feedback_msg = DriveControl.Feedback()
            feedback_msg.forward_error = forward_error
            feedback_msg.lateral_error = lateral_error
            feedback_msg.stability_count = self.stability_count
            self.current_goal_handle.publish_feedback(feedback_msg)

    def stop_robot(self):
        """
        Stop the robot by sending zero velocity commands
        """
        drive_cmd = DriveCommand()
        drive_cmd.left = 0.0
        drive_cmd.right = 0.0
        self.drive_pub.publish(drive_cmd)

def main(args=None):
    rclpy.init(args=args)
    
    # Use MultiThreadedExecutor to enable concurrent callbacks
    executor = MultiThreadedExecutor()
    drive_controller_node = DriveController()
    executor.add_node(drive_controller_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        drive_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()