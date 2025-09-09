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
from sensor_msgs.msg import Image, CompressedImage
from dtc_msgs.msg import CasualtyFix
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
from helpers import gps_distance, closest_casualty

from std_msgs.msg import Bool, String, UInt8, Float32
from dtc_msgs.msg import ScoreCard, CasualtyFixArray, CasualtyFix
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
import cv2

#TODO: Add a hashing dictionary to keep track of victims and corresponding reports
class Casualty:
    def __init__(self, id, position, image_id=None, report=None):
        self.id = id
        self.position = position
        self.image_id = image_id
        self.report = report

    def to_dict(self):
        return {
            "id": self.id,
            "position": self.position,
            "image_id": self.image_id,
            "report": self.report
        }

class SubmissionNode:
    def __init__(self):
        print("[Scorecard][STATUS] Submission Node initialized")
        #start_run()
        #Subscribe to scorecard_topic
        self.deimos_report_sub = rospy.Subscriber("/deimos/report_status", String, self.DeimosScoreCallback)
        self.deimos_image_sub = rospy.Subscriber("/deimos/camera/image", Image, self.DeimosImageCallback)

        self.dione_position_sub = rospy.Subscriber("/casualty_info", CasualtyFixArray, self.CasualtyPosCallback)

        self.casualty_dict_list = []
    
    def CasualtyPosCallback(self, msg):
        self.casualty_dict_list = []
        if len(msg.casualties) != len(self.casualty_dict_list):
            print("[Scorecard][STATUS] Received new casualty position from /casualty_info")
            self.casualty_dict_list = []
            for msg in msg.casualties:
                
                location = msg.location
                casualty = Casualty(id=msg.casualty_id, position=(location.longitude, location.longitude))
                self.casualty_dict_list.append(casualty.to_dict())
                print(f"[Scorecard][STATUS] Casualty ID: {casualty.id}, Position:{casualty.position}")
                print(f"{msg.time_ago} seconds ago")
            
            print(self.casualty_dict_list)

    def DeimosImageCallback(self, msg):
        print("[Scorecard][STATUS] Received image from /deimos/camera/image")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        timestamp = int(rospy.Time.now().to_sec())
        image_path = f"/tmp/deimos_image_{timestamp}.jpg"
        # Save the image to a file
        
        cv2.imwrite(image_path, cv_image)
        print(f"[Scorecard][STATUS] Image saved to {image_path}")
        image_id = submit_image(image_path=image_path)
        print(f"[Scorecard][STATUS] Image submitted with ID: {image_id}")

    def DeimosScoreCallback(self, msg):
        print("[Scorecard][STATUS] Received report from /deimos/report_status")
        print("[Scorecard][STATUS] Publishing report status")

        #extract the string from the message
        report_str = msg.data
        print(report_str)

        _, _, closest_casualty_idx, payload = closest_casualty(report_str, self.casualty_dict_list)
        
        if closest_casualty_idx == -1:
            print("[Scorecard][WARNING] No casualties available to assign report to.")
            return
        
        print(f"[Scorecard][STATUS] Closest casualty index: {closest_casualty_idx}")
        payload["casualty_id"] = self.casualty_dict_list[closest_casualty_idx]["id"]
        payload["longitude"], payload["latitude"] = self.casualty_dict_list[closest_casualty_idx]["position"]
        self.casualty_dict_list[closest_casualty_idx]["report"] = payload
        submit_scorecard(payload)        
        

if __name__ == "__main__":
    rospy.init_node("scorecard_submitter")
    SubmissionNode()
    rospy.spin()