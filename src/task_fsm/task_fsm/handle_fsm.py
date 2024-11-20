import rclpy
from rclpy.node import Node
from agrobot_interfaces.msg import Command
from agrobot_interfaces.srv import StartFSM

from enum import Enum

class HandleFSM(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    Finite State Machine for the handle task.

    Publishers:
        - actuator/command (agrobot_interfaces/msg/Command) # TODO: Change this  

    Services:
        - handle/start (agrobot_interfaces/srv/StartFSM)
    '''

    # Define the states of the FSM
    class State(Enum):
        RED = 1
        GREEN = 2
        BLUE = 3

    def __init__(self):
        super().__init__('handle_fsm')

        self.running = False
        self.state = self.State.RED

        self.actuator_pub = self.create_publisher(Command, 'actuator/command', 10)
        self.start_service = self.create_service(StartFSM, 'handle/start', self.start_callback)

    def start_callback(self, request, response):

        self.get_logger().info('Received request to start the handle FSM')
        self.running = True
        response.success = True
        return response

def handle_fsm(node):

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
    actuator_msg = Command()
    node.actuator_pub.publish(actuator_msg)

    node.running = False # Set when finished

def main(args=None):
    rclpy.init(args=args)

    handle_fsm_node = HandleFSM()
    while not handle_fsm_node.running:
        rclpy.spin_once(handle_fsm_node)

    while handle_fsm_node.running:
        handle_fsm(handle_fsm_node)

    handle_fsm_node.get_logger().info('The sorting FSM finished')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    handle_fsm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
