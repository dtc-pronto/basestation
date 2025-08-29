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
import rospy
from sensor_msgs.msg import Image, CompressedImage
from wfov_camera_msgs.msg import WFOVImage



"""
    Edward Zhang
    June 2025
    Node that runs VLM inference on the Jackal robot.
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2025
"""

import rospy
from submission import submit_scorecard

from std_msgs.msg import Bool, String, UInt8, Float32
from dtc_msgs.msg import ScoreCard
from cv_bridge import CvBridge


class SubmissionNode:

    def __init__(self):
        print("[Scorecard][STATUS] Submission Node initialized")

        #Subscribe to scorecard_topic
        self.report_sub = rospy.Subscriber("/deimos/report_status", String, self.DeimosScoreCallback)
    
    def DeimosScoreCallback(self, msg):
        print("[Scorecard][STATUS] Received report from /deimos/report_status")


        print("[Scorecard][STATUS] Publishing report status")

        #extract the string from the message
        report_str = msg.data
        print(report_str)

        submit_scorecard(report_str)
        
        

if __name__ == "__main__":
    rospy.init_node("scorecard_submitter")
    SubmissionNode()
    rospy.spin()