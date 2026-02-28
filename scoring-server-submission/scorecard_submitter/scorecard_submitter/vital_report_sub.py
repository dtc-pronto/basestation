#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CasualtyFixArray, CasualtyFix
from submission import vitals_report


UAV = ['dione']
UGV = ['deimos', 'phobos', 'titania', 'oberon']
class VitalsReportNode(Node):
    def __init__(self):
        super().__init__('Vitals_Report_Submitter')