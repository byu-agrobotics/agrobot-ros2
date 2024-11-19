import rclpy
from rclpy.node import Node
from agrobot_interfaces.msg import EggType
from agrobot_interfaces.srv import IdentifyEgg

class EggID(Node):
    '''
    :author: ADD HERE
    :date: ADD HERE

    Service that identifies eggs.

    Publishers:
        - egg/type (agrobot_interfaces/msg/EggType)

    Services:
        - egg/identify (agrobot_interfaces/srv/IdentifyEgg)  
    '''

    def __init__(self):
        super().__init__('egg_id')

        # Create the egg type publisher
        self.egg_type_pub = self.create_publisher(EggType, 'egg/type', 10)

        # Create the egg identification service
        self.egg_id_service = self.create_service(IdentifyEgg, 'egg/identify', self.egg_id_callback)

    def egg_id_callback(self, request, response):

        self.get_logger().info('Received request to identify an egg')

        # TODO: Interface with the camera here

        # TODO: Add egg identification code here

        # Publish the egg type
        egg_type = EggType()
        egg_type.egg_type = 1 # 1: small, 2: large, 3: bad
        self.egg_type_pub.publish(egg_type)

        response.egg_type = egg_type.egg_type
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
