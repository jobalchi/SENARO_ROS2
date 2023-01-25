#! /usr/bin/env python3

from senaro_interface.srv import CreateMap  # CHANGE
import rclpy
from rclpy.node import Node


class CreateMapServer(Node):
    def __init__(self):
        super().__init__('create_map_service_server')
        self.srv = self.create_service(
            CreateMap, 'create_map', self.create_map_callback
        )
        self.get_logger().info('==== Addition Server Started, Waiting for Request ====')

    def create_map_callback(self, request, response):
        response.is_successed = True
        self.get_logger().info('hello world!!')
        return response


def main(args=None):
    rclpy.init(args=args)
    create_map_node = CreateMapServer()
    rclpy.spin(create_map_node)
    create_map_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()