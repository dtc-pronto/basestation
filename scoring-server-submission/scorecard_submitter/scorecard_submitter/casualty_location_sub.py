#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CasualtyFixArray, CasualtyFix
from helpers import update_drone_casualty_db, update_jackal_casualty_db
from submission import location_report


UAV = ['dione']
UGV = ['deimos', 'phobos', 'titania', 'oberon']
class CasualtyLocationSub(Node):
    def __init__(self):
        super().__init__('casualty_location_submitter')

        self.declare_parameter('robots', [])
        self.robots = self.get_parameter('robots').value

        self.subscription = []
        for robot in self.robots:
            if robot in UAV:
                self.subscription.append(self.create_subscription(
                    CasualtyFixArray, f"/{robot}/casualtyfix_report", lambda msg, robot=robot: self.location_callback(msg, robot), 1)
                    )
                self.get_logger().info(f"Subscribed to /{robot}/casualtyfix_report for casualty location")
            elif robot in UGV:
                self.subscription.append(self.create_subscription(
                    CasualtyFixArray, f"/{robot}/casualty_report", lambda msg, robot=robot: self.location_callback(msg, robot), 1)
                    )
                self.get_logger().info(f"Subscribed to /{robot}/casualty_report for casualty location")