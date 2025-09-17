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
from helpers import gps_distance


"""
    Edward Zhang
    June 2025
    Node that runs VLM inference on the Jackal robot.
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2025
"""

import rospy
from submission import start_run, submit_image, report_new_casualty, update_casualty
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
        self.time = rospy.Time.now().to_sec()

    def to_dict(self):
        return {
            "id": self.id,
            "position": self.position,
            "image_id": self.image_id,
            "report": self.report,
            "time": self.time
        }

class SubmissionNode:
    def __init__(self):
        print("[Scorecard][STATUS] Submission Node initialized")
        start_run()
        #Subscribe to scorecard_topic
        self.deimos_report_sub = rospy.Subscriber("/deimos/report_status", String, self.DeimosScoreCallback)
        self.deimos_image_sub = rospy.Subscriber("/deimos/camera/image", Image, self.DeimosImageCallback)
        self.deimos_report_sub = rospy.Subscriber("/phobos/report_status", String, self.PhobosScoreCallback)
        self.deimos_image_sub = rospy.Subscriber("/phobos/camera/image", Image, self.PhobosImageCallback)

        self.dione_position_sub = rospy.Subscriber("/dione/casualty_info", CasualtyFixArray, self.CasualtyPosCallback)

        self.casualty_dict_list = []
        self.most_recent_deimos_image_path = None
        #Add the paths of other robots
    
    def CasualtyPosCallback(self, msg):
        print(len(msg.casualties))
        print(len(self.casualty_dict_list))
        if len(msg.casualties) > len(self.casualty_dict_list):
            print("[Scorecard][STATUS] Received new casualty position from /casualty_info")
            for elt in msg.casualties:
                location = elt.location
                print(location)
                if len(self.casualty_dict_list) == 0:
                    print("[Scorecard][STATUS] First casualty detected, adding to list")
                    new_casualty = Casualty(id=len(self.casualty_dict_list), position=(location.latitude, location.longitude))
                    self.casualty_dict_list.append(new_casualty.to_dict())
                    print(f"[Scorecard][STATUS] Casualty ID: {new_casualty.id}, Position:{new_casualty.position}")
                    report_new_casualty(new_casualty.id, location.latitude, location.longitude, new_casualty.time)
                else:
                    for casualty in self.casualty_dict_list:
                        lat1, lon1 = casualty["position"]
                        lat2, lon2 = location.latitude, location.longitude
                        if gps_distance(lat1, lon1, lat2, lon2) > 0.2:
                            print("[Scorecard][STATUS] New casualty detected, addding to list")
                            new_casualty = Casualty(id=len(self.casualty_dict_list), position=(lat2, lon2))
                            self.casualty_dict_list.append(new_casualty.to_dict())
                            print(f"[Scorecard][STATUS] Casualty ID: {new_casualty.id}, Position:{new_casualty.position}")
                            print(f"{elt.time_ago} seconds ago")

                            report_new_casualty(new_casualty.id, lat2, lon2, new_casualty.time)
                            break
            
            db_path = os.path.join(script_dir, "data/casualty_list.json")
                #self.casualty_dict_list to a json file
            with open(db_path, "w") as f:
                json.dump(self.casualty_dict_list, f, indent=2)
            print(self.casualty_dict_list)

    def DeimosImageCallback(self, msg):
        print("[Scorecard][STATUS] Received image from /deimos/camera/image")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        timestamp = int(rospy.Time.now().to_sec())
        image_path = f"/data/deimos_image_{timestamp}.jpg"
        self.most_recent_deimos_image_path = image_path
        # Save the image to a file
        
        cv2.imwrite(image_path, cv_image)
        print(f"[Scorecard][STATUS] Image saved to {image_path}")
        

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
        payload["location"]["longitude"], payload["location"]["latitude"] = self.casualty_dict_list[closest_casualty_idx]["position"]
        payload["location"]["time_ago"] = self.casualty_dict_list[closest_casualty_idx]["time"]
        self.casualty_dict_list[closest_casualty_idx]["report"] = payload
        update_casualty(payload)  

        timestamp = int(self.most_recent_deimos_image_path.split("_")[-1].split(".")[0])
        image_id = submit_image(image_path=self.most_recent_deimos_image_path, time=timestamp, id=self.casualty_dict_list[closest_casualty_idx]["id"])
        print(f"[Scorecard][STATUS] Image submitted with ID: {image_id}")      
        

if __name__ == "__main__":
    rospy.init_node("scorecard_submitter")
    SubmissionNode()
    rospy.spin()
