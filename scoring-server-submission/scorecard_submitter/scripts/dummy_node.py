#!/usr/bin/env python3
import os
import sys
import re

script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)

import threading 
import subprocess
import time
import ast
import json
import utm
import rospy
from helpers import gps_distance


"""
    Edward Zhang
    June 2025
    Node that runs VLM inference on the Jackal robot.
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2025
"""

import rospy

from std_msgs.msg import Bool, String, UInt8, Float32

import cv2
import json

class DummyNode:
    def __init__(self):
        self.pub = rospy.Publisher("/deimos/report_status", String, queue_size=10)

        path = "/home/dtc/ws/src/scorecard_submitter/scripts/data/reports_09_18_2025.json"
        with open(path, "r") as f:
            self.reports = json.load(f)

        time.sleep(10)
        for elt in self.reports:
            time.sleep(5)
            self.pub.publish(json.dumps(elt))
            rospy.loginfo(f"Published: {elt}")
    
    
        

if __name__ == "__main__":
    rospy.init_node("scorecard_submitter")
    DummyNode()    
    rospy.spin()

    
