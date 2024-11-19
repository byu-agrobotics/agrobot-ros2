import rclpy
from rclpy.node import Node
from agrobot_interfaces.msg import Command
from agrobot_interfaces.srv import StartFSM

from enum import Enum

class NavigateFSM(Node):
    '''
    :author: ADD HERE
    :date: ADD HERE

    Finite State Machine for the navigation task.

    Publishers:
        - control/command (agrobot_interfaces/msg/Command) # TODO: Change this

    Services:
        - navigate/start (agrobot_interfaces/srv/StartFSM)
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

        # Create the actuator publishers
        self.control_pub = self.create_publisher(Command, 'control/command', 10)

        # Create the start service
        self.start_service = self.create_service(StartFSM, 'navigate/start', self.start_callback)

    def start_callback(self, request, response):

        self.get_logger().info('Received request to start the navigation FSM')
        self.running = True
        response.success = True
        return response

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

    # Publisher call example
    control_msg = Command()
    control_msg.command = 'control'
    node.control_pub.publish(control_msg)

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
