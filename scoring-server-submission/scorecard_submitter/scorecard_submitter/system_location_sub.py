#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import NavSatFix
from submission import post_system_location


"""
    Yifan
    February 2026
    Node that sends the latest received robot position to the server.
    publishes at rate set in config, and will retry up to 3 times if the post request fails.

    DTC PRONTO 2026:

Yifan's notes:
-
Might block itself for some of the callbacks. Needs to be tested
"""

UAV = ['dione']
UGV = ['deimos', 'phobos', 'titania', 'oberon']
class SystemLocationSub(Node):
    def __init__(self):
        super().__init__('system_location_submitter')

        self.declare_parameter('rate', 2)
        self.rate = self.get_parameter('rate').value

        self.declare_parameter('robots', ['deimos', 'phobos', 'titania', 'oberon', 'dione'])
        self.robots = self.get_parameter('robots').value
        
        self.latest_location = {robot: None for robot in self.robots}
        self.position_update = self.create_timer(1.0 / self.rate, self.position_update_callback)
        
        self.subscriptions = [] 

        for robot in self.robots:
            if robot in UAV:
                self.subscriptions.append(self.create_subscription(
                    NavSatFix, f"/{robot}/mavros/fix", lambda msg, robot=robot: self.UAV_callback(msg, robot), 1)
                    )
                self.get_logger().info(f"Subscribed to /{robot}/mavros/fix for UAV")
            elif robot in UGV:
                self.subscriptions.append(self.create_subscription(
                    NavSatFix, f"/{robot}/ublox/fix", lambda msg, robot=robot: self.UGV_callback(msg, robot), 1)
                    )
                self.get_logger().info(f"Subscribed to /{robot}/ublox/fix for UGV")

    def UAV_callback(self, msg, robot_name):
        self.latest_location[robot_name] = msg
        self.get_logger().info(f"Received UAV location for {robot_name}: {msg.latitude}, {msg.longitude}, {msg.altitude}")

    def UGV_callback(self, msg, robot_name):
        self.latest_location[robot_name] = msg
        self.get_logger().info(f"Received UGV location for {robot_name}: {msg.latitude}, {msg.longitude}, {msg.altitude}")
    
    def position_update_callback(self):
        for robot, location in self.latest_location.items():
            if location is not None:
                response = self.response_checked(robot)
                if response is None or response.status_code != 200:
                    self.get_logger().error(f"Failed to post location for {robot} after retries")

    def response_checked(self, robot):
        response = None
        for i in range(4):
            response = post_system_location(
                lat=self.latest_location[robot].latitude,
                lon=self.latest_location[robot].longitude,
                alt=self.latest_location[robot].altitude,
                system=robot,
                type='1' if robot in UAV else '0'
            )
            if response.status_code == 200:
                self.get_logger().info(f"Successfully posted location for {robot}")
                break
            else:
                self.get_logger().error(f"Failed to post location for {robot}: {response.status_code} - {response.text}")

        return response