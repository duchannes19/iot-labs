import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import Spin

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spin_client')

    client = node.create_client(Spin, 'spin')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    while rclpy.ok():
        request = Spin.Request()
        request.direction = input('Enter direction (left/right/quit): ')

        if request.direction.lower() == 'quit':
            rclpy.shutdown()
            node.destroy_node()
            return 0

        future = client.call_async(request)

        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()
            node.get_logger().info('Response: %s' % response.message)
        else:
            node.get_logger().info('Service call failed')

if __name__ == '__main__':
    main()
