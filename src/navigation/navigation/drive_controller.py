import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from agrobot_interfaces.msg import ToFData, DriveCommand
from agrobot_interfaces.action import DriveControl, DriveStraight, Turn
import math

STABILITY_THRESHOLD = 10
CENTERING_THRESHOLD = 10  # 10mm = 1cm threshold for centering
TURN_ERROR_THRESHOLD = 20 # 20mm = 2cm threshold for turning
TURN_TIMEOUT_S = 15.0 # Timeout for the turn action in seconds

class DriveController(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024 (Updated December 2024)

    Node that runs the drive command controllers.

    Publishers:
        - drive/command (agrobot_interfaces/msg/DriveCommand)

    Subscribers:
        - tof/data (agrobot_interfaces/msg/ToFData)

    Action Servers:
        - control/center (agrobot_interfaces/action/DriveControl)
        - control/drive_straight (agrobot_interfaces/action/DriveStraight)
        - control/turn (agrobot_interfaces/action/Turn)
    '''

    def __init__(self):
        super().__init__('drive_controller')

        cb_group = ReentrantCallbackGroup()

        self.drive_pub = self.create_publisher(DriveCommand, 'drive/command', 10)
        self.tof_sub = self.create_subscription(
            ToFData, 'tof/data', self.tof_callback, 10, callback_group=cb_group
        )

        self.center_action_server = ActionServer(
            self, DriveControl, 'control/center',
            execute_callback=self.center_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )
        self.drive_straight_action_server = ActionServer(
            self, DriveStraight, 'control/drive_straight',
            execute_callback=self.drive_straight_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )
        self.turn_action_server = ActionServer(
            self, Turn, 'control/turn',
            execute_callback=self.turn_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )

        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('turn_speed', 40.0)


        self.tof_data = None
        self.new_data = False
        self.active_goal = False

    def tof_callback(self, msg):
        self.tof_data = msg
        self.new_data = True

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request for {goal_request.__class__.__name__}')
        if self.active_goal:
            self.get_logger().warn('Another goal is active, rejecting new goal.')
            return GoalResponse.REJECT
        self.active_goal = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        self.active_goal = False
        self.stop_robot()
        return CancelResponse.ACCEPT

    def stop_robot(self):
        drive_cmd = DriveCommand()
        drive_cmd.left_speed = 0.0
        drive_cmd.right_speed = 0.0
        self.drive_pub.publish(drive_cmd)

    # --- Center Action ---
    def center_execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Center Robot')

        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        
        integral_forward = 0
        integral_lateral = 0
        last_error_forward = 0
        last_error_lateral = 0
        stability_count = 0

        while rclpy.ok() and self.active_goal:
            if self.new_data:
                forward_error = self.tof_data.front - self.tof_data.back
                lateral_error = self.tof_data.left - self.tof_data.right

                integral_forward += forward_error
                derivative_forward = forward_error - last_error_forward
                
                integral_lateral += lateral_error
                derivative_lateral = lateral_error - last_error_lateral

                forward_correction = kp * forward_error + ki * integral_forward + kd * derivative_forward
                lateral_correction = kp * lateral_error + ki * integral_lateral + kd * derivative_lateral
                
                drive_cmd = DriveCommand()
                drive_cmd.left_speed = forward_correction - lateral_correction
                drive_cmd.right_speed = forward_correction + lateral_correction
                self.drive_pub.publish(drive_cmd)

                last_error_forward = forward_error
                last_error_lateral = lateral_error
                self.new_data = False

                if abs(forward_error) < CENTERING_THRESHOLD and abs(lateral_error) < CENTERING_THRESHOLD:
                    stability_count += 1
                    if stability_count >= STABILITY_THRESHOLD:
                        goal_handle.succeed()
                        break
                else:
                    stability_count = 0
            
            rclpy.spin_once(self, timeout_sec=0.1)

        self.stop_robot()
        self.active_goal = False
        return DriveControl.Result(success=True)

    # --- Drive Straight Action ---
    def drive_straight_execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Drive Straight')
        desired_front_distance = goal_handle.request.front_distance

        if not self.tof_data:
            self.get_logger().error("No ToF data available to start driving straight.")
            goal_handle.abort()
            self.active_goal = False
            return DriveStraight.Result(success=False)
            
        desired_side_distance = self.tof_data.left 
        
        kp = self.get_parameter('kp').value
        
        while rclpy.ok() and self.active_goal:
            if self.new_data:
                front_error = self.tof_data.front - desired_front_distance
                side_error = self.tof_data.left - desired_side_distance

                forward_speed = kp * front_error
                turn_correction = kp * side_error
                
                drive_cmd = DriveCommand()
                drive_cmd.left_speed = forward_speed - turn_correction
                drive_cmd.right_speed = forward_speed + turn_correction
                self.drive_pub.publish(drive_cmd)
                
                self.new_data = False

                if abs(front_error) < CENTERING_THRESHOLD:
                    goal_handle.succeed()
                    break

            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.stop_robot()
        self.active_goal = False
        return DriveStraight.Result(success=True)
        
    # --- Turn Action ---
    def turn_execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Turn with Wall Distances')
        angle = goal_handle.request.angle

        if not self.tof_data:
            self.get_logger().error("No ToF data available to start turn.")
            goal_handle.abort()
            self.active_goal = False
            return Turn.Result(success=False)

        # Store initial distances
        initial_front = self.tof_data.front
        initial_back = self.tof_data.back
        initial_left = self.tof_data.left
        initial_right = self.tof_data.right
        
        self.get_logger().info(f"Initial distances - Front: {initial_front}, Back: {initial_back}, Left: {initial_left}, Right: {initial_right}")

        # Determine target distances based on 90-degree turn
        target_front, target_back, target_left, target_right = 0, 0, 0, 0
        # Right turn
        if angle > 0: 
            target_front, target_right, target_back, target_left = initial_left, initial_front, initial_right, initial_back
        # Left turn
        else: 
            target_front, target_right, target_back, target_left = initial_right, initial_back, initial_front, initial_left
        
        turn_speed = self.get_parameter('turn_speed').value
        drive_cmd = DriveCommand()
        drive_cmd.left_speed = turn_speed if angle > 0 else -turn_speed
        drive_cmd.right_speed = -turn_speed if angle > 0 else turn_speed
        self.drive_pub.publish(drive_cmd)
        
        start_time = self.get_clock().now()
        stability_count = 0
        
        while rclpy.ok() and self.active_goal:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > TURN_TIMEOUT_S:
                self.get_logger().error("Turn action timed out!")
                goal_handle.abort()
                break

            if self.new_data:
                # Calculate the total error
                error_front = abs(self.tof_data.front - target_front)
                error_back = abs(self.tof_data.back - target_back)
                error_left = abs(self.tof_data.left - target_left)
                error_right = abs(self.tof_data.right - target_right)
                total_error = error_front + error_back + error_left + error_right

                # Publish feedback
                feedback_msg = Turn.Feedback()
                feedback_msg.current_error = total_error
                goal_handle.publish_feedback(feedback_msg)

                if total_error < TURN_ERROR_THRESHOLD:
                    stability_count += 1
                    if stability_count >= STABILITY_THRESHOLD:
                        self.get_logger().info("Turn successful!")
                        goal_handle.succeed()
                        break
                else:
                    stability_count = 0
                
                self.new_data = False

            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.active_goal = False
        # If the loop was exited for any other reason (timeout, cancel), the result is implicitly abort/canceled.
        return Turn.Result(success=True if goal_handle.status == 4 else False)


def main(args=None):
    rclpy.init(args=args)
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