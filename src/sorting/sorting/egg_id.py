import rclpy
from rclpy.node import Node
from agrobot_interfaces.msg import EggType
from agrobot_interfaces.srv import IdentifyEgg

class EggID(Node):
    '''
    :author: Nelson Durrant
    :date: November 2024

    Service that identifies eggs.

    Clients:
        - TODO: Add camera interface services here? Or just a subscriber?

    Services:
        - egg/identify (agrobot_interfaces/srv/IdentifyEgg)
    '''

    def __init__(self):
        super().__init__('egg_id')

        self.egg_id_service = self.create_service(IdentifyEgg, 'egg/identify', self.egg_id_callback)

    def egg_id_callback(self, request, response):

        self.get_logger().info('Received request to identify an egg')

        # TODO: Call the camera interface services here? Or just a subscriber?

        # TODO: Add egg identification code here

        # Return the egg type for the FSM
        egg_type = 1
        response.egg_type = egg_type # 1: small, 2: large, 3: bad
        return response

def main(args=None):
    rclpy.init(args=args)

    egg_id_node = EggID()
    rclpy.spin(egg_id_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    egg_id_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
