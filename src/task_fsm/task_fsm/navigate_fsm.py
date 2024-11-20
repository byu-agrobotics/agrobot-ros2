import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from agrobot_interfaces.msg import LEDCommand
from agrobot_interfaces.srv import StartFSM
from agrobot_interfaces.action import DriveControl

from enum import Enum

class NavigateFSM(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    Finite State Machine for the navigation task.

    Publishers:
        - led/command (agrobot_interfaces/msg/LEDCommand) # TODO: Make this

    Services:
        - navigate/start (agrobot_interfaces/srv/StartFSM)

    Action Clients:
        - control/center (agrobot_interfaces/action/DriveControl)
    '''

    # Define the states of the FSM
    class State(Enum):
        RED = 1
        GREEN = 2
        BLUE = 3

    def __init__(self):
        super().__init__('navigate_fsm')

        self.running = False
        self.state = self.State.RED

        # Create the action client
        self._action_client = ActionClient(self, DriveControl, 'control/center')

        self.led_pub = self.create_publisher(LEDCommand, 'led/command', 10)
        self.start_service = self.create_service(StartFSM, 'navigate/start', self.start_callback)

    def start_callback(self, request, response):

        self.get_logger().info('Received request to start the navigation FSM')
        self.running = True
        response.success = True
        return response
    
    def send_goal(self):

        goal_msg = DriveControl.Goal()
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def navigate_fsm(node):

    while node.running:
        match node.state:
            case node.State.RED:
                # Do something
                pass
            case node.State.GREEN:
                # Do something
                pass
            case node.State.BLUE:
                # Do something
                pass
            case _: # Default case
                node.get_logger().error('Invalid state')
                break

    # Action call example
    future = node.send_goal()
    rclpy.spin_until_future_complete(node, future)
    # TODO: Add result handling here ?

    # Publisher call example
    led_msg = LEDCommand()
    node.led_pub.publish(led_msg)

    node.running = False # Set when finished

def main(args=None):
    rclpy.init(args=args)

    navigate_fsm_node = NavigateFSM()
    while not navigate_fsm_node.running:
        rclpy.spin_once(navigate_fsm_node)

    while navigate_fsm_node.running:
        navigate_fsm(navigate_fsm_node)

    navigate_fsm_node.get_logger().info('The navigation FSM finished')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigate_fsm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
