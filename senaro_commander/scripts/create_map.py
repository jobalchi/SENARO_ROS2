#! /usr/bin/env python3

from senaro_interface.srv import CreateMap  # CHANGE
from insta360_interface.srv import TakePicture
from std_msgs.msg import Float32MultiArray

import rclpy
from rclpy.node import Node
from threading import Event

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

from action_msgs.msg import GoalStatus

from robot_navigator import BasicNavigator

import serial
import time

py_serial = serial.Serial(
    
    # Window
    port='/dev/ttyACM0',
    
    # 보드 레이트 (통신 속도)
    baudrate=9600,
)

class CreateMapServer(Node):
    def __init__(self):
        super().__init__('create_map_service_server')
        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            CreateMap, 'create_map', self.create_map_callback, callback_group=self.callback_group
        )
        self.insta360_request = TakePicture.Request()
        self.client = self.create_client(
            TakePicture, 'insta360_node/take_picture', callback_group=self.callback_group
        )

        self.process_pub = self.create_publisher(Float32MultiArray, 'current_process', 10)

        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_logger().info('==== Addition Server Started, Waiting for Request ====')

    def create_map_callback(self, request, response):
        msg = Float32MultiArray()
        msg.data = [request.waypoints[0] * 1.0, request.waypoints[1] * 1.0, 0.0]
        self.process_pub.publish(msg)
        
        py_serial.write(1)

        len_waypoints = len(request.waypoints) // 2
        for count in range(len_waypoints):
            x = request.waypoints[count * 2]
            y = request.waypoints[count * 2 + 1]
            x_next = 100000000.1
            y_next = 100000000.1
            if count < (len_waypoints - 1):
                x_next = request.waypoints[(count + 1) * 2] * 1.0
                y_next = request.waypoints[(count + 1) * 2 + 1] * 1.0

            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.pose.position.x = x
            initial_pose.pose.position.y = y
            initial_pose.pose.orientation.z = 1.0
            initial_pose.pose.orientation.w = 0.0
            nav_success = self.goToPose(initial_pose)

            if nav_success:
                self.insta360_request.target_path = "/home/senaro/pictures/picture_{0}".format(count)
                self.insta360_request.source_path = ""
                self.insta360_request.execute_take_photo = True
                insta360_response = self.client.call(self.insta360_request)
                print(insta360_response.is_successed)
            else:
                self.warn("nav fail to goal pose x: {0} y: {1}".format(x, y))

            msg = Float32MultiArray()

            msg.data = [x_next, y_next, count * 1.0 + 1.0]
            self.process_pub.publish(msg)

        py_serial.write(0)

        response.is_successed = True
        return response

    def goToPose(self, pose):
        # Sends a `NavToPose` action request
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal(goal_msg)
        if send_goal_future.status == GoalStatus.STATUS_ABORTED:
            return False
        return True

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def main(args=None):
    rclpy.init(args=args)
    create_map_node = CreateMapServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(create_map_node, executor)
    create_map_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()