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
from submission import submit_scorecard, start_run, submit_image

from std_msgs.msg import Bool, String, UInt8, Float32
from dtc_msgs.msg import ScoreCard
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

#TODO: Add a hashing dictionary to keep track of victims and corresponding reports

class SubmissionNode:

    def __init__(self):
        print("[Scorecard][STATUS] Submission Node initialized")
        start_run()
        #Subscribe to scorecard_topic
        self.deimos_report_sub = rospy.Subscriber("/deimos/report_status", String, self.DeimosScoreCallback)
        self.deimos_image_sub = rospy.Subscriber("/deimos/camera/image", Image, self.DeimosImageCallback)
    
    def DeimosImageCallback(self, msg):
        print("[Scorecard][STATUS] Received image from /deimos/camera/image")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        timestamp = int(rospy.Time.now().to_sec())
        image_path = f"/tmp/deimos_image_{timestamp}.jpg"
        # Save the image to a file
        
        cv2.imwrite(image_path, cv_image)
        print(f"[Scorecard][STATUS] Image saved to {image_path}")
        image_id = submit_scorecard(image_path=image_path)
        print(f"[Scorecard][STATUS] Image submitted with ID: {image_id}")

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