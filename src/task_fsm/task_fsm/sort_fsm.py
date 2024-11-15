import rclpy
from rclpy.node import Node
from agrobot_interfaces.srv import IdentifyEgg, StartFSM

from enum import Enum

class SortFSM(Node):
    '''
    :author: ADD HERE
    :date: ADD HERE

    Finite State Machine for the sorting task.

    Clients:
        - egg/identify (agrobot_interfaces/srv/IdentifyEgg)        

    Services:
        - sort/start (agrobot_interfaces/srv/StartFSM)
    '''

    # Define the states of the FSM
    class State(Enum):
        RED = 1
        GREEN = 2
        BLUE = 3

    def __init__(self):
        super().__init__('sort_fsm')

        self.running = False
        self.state = self.State.RED

        # Create the egg identification client
        self.egg_id_client = self.create_client(IdentifyEgg, 'egg/identify')
        if not self.egg_id_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Egg identification client not available')
            return
        self.egg_id_request = IdentifyEgg.Request

        # Create the start service
        self.start_service = self.create_service(StartFSM, 'sort/start', self.start_callback)

    def start_callback(self, request, response):

        self.get_logger().info('Received request to start the sorting FSM')
        self.running = True
        response.success = True
        return response
    
    def send_request(self):
        # Add something to the request?
        return self.egg_id_client.call_async(self.egg_id_request)

def sort_fsm(node):

    while node.running:
        match node.state:
            case node.State.RED:
                # Do something
                break
            case node.State.GREEN:
                # Do something
                break
            case node.State.BLUE:
                # Do something
                break
            case _:
                # Throw an error
                break

    # Service call example
    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if not response:
        node.get_logger().error('Service call failed')
    else:
        node.get_logger().info('Egg identified: %s' % response.egg_type)
        # Do something with the response

    node.running = False # Set when finished

def main(args=None):
    rclpy.init(args=args)

    sort_fsm_node = SortFSM()
    while not sort_fsm_node.running:
        rclpy.spin_once(sort_fsm_node)

    while sort_fsm_node.running:
        sort_fsm(sort_fsm_node)

    sort_fsm_node.get_logger().info('The sorting FSM finished')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sort_fsm_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
