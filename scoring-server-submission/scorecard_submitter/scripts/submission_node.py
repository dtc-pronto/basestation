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
from wfov_camera_msgs.msg import WFOVImage
from helpers import gps_distance


"""
    Edward Zhang
    Jason Hughes
    June 2025
    Node that runs VLM inference on the Jackal robot.
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2025
"""

import rospy
import numpy as np
from submission import start_run, submit_image, report_new_casualty, update_casualty, init_position, init_supplement, total_posts, parse_report_string_as_json
from helpers import gps_distance, closest_casualty, update_drone_casualty_db, update_jackal_casualty_db, parse_report_string

from std_msgs.msg import Bool, String, UInt8, Float32
from dtc_msgs.msg import ScoreCard, CasualtyFixArray, CasualtyFix, ScoreCardString
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from cv_bridge import CvBridge
import cv2
import json
import copy
from datetime import datetime
from typing import Dict

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
        start_run()
        
        threshold_config_path = rospy.get_param("~threshold_path")
        with open(threshold_config_path, "r") as f:
            self.config = json.load(f)

        
        data_path = rospy.get_param("~data_path")
        folder_name = f"casualty_db_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.db_path = os.path.join(data_path, folder_name)
        os.makedirs(self.db_path, exist_ok=False)
        rospy.loginfo("[SENDER] Using database at %s" %self.db_path)
        self.drone_db = os.path.join(data_path, folder_name, "uav_casualty_list.json")
        self.jackal_db = os.path.join(data_path, folder_name, "ugv_casualty_list.json")
        self.matching_table = os.path.join(data_path, folder_name, "matching_table.json")

        #write empty json files to start
        with open(self.drone_db, "w") as f:
            json.dump([], f)
        with open(self.jackal_db, "w") as f:
            json.dump([], f)
        with open(self.matching_table, "w") as f:
            json.dump([], f)

        robots = rospy.get_param("/robots") 
        self.mutex = threading.Lock()
        #Subscribe to scorecard_topic
        # TODO deprecate these topics
        #self.deimos_report_sub = rospy.Subscriber("/deimos/report_status", String, self.deimosScoreCallback) #changed atm
        #self.deimos_image_sub = rospy.Subscriber("/deimos/processed_image", Image, self.deimosImageCallback)
        #self.deimos_report_sub = rospy.Subscriber("/phobos/report_status", String, self.phobosScoreCallback)
        #self.deimos_image_sub = rospy.Subscriber("/phobos/process_image", Image, self.phobosImageCallback)

        if "deimos" in robots:
            rospy.loginfo("[SENDER] Adding Deimos to submission")
            self.deimos_sub = rospy.Subscriber("/deimos/report_status", ScoreCardString, self.deimosCallback)
        if "phobos" in robots:
            rospy.loginfo("[SENDER] Adding Phobos to submission")
            self.phobos_sub = rospy.Subscriber("/phobos/report_status", ScoreCardString, self.phobosCallback)
        if "titania" in robots:
            rospy.loginfo("[SENDER] Adding Titania to submission")
            self.titania_sub = rospy.Subscriber("/titania/report_status", ScoreCardString, self.titaniaCallback)
        if "oberon" in robots:
            rospy.loginfo("[SENDER] Adding Oberon to submission")
            self.oberon_sub = rospy.Subscriber("/oberon/report_status", ScoreCardString, self.oberonCallback)
        if "dione" in robots:
            rospy.loginfo("[SENDER] Adding Dione to submission")
            self.dione_position_sub = rospy.Subscriber("dione/casualty_info", CasualtyFixArray, self.casualtyPosCallback)

        self.drone_casualty_dict_list = []
        self.jackal_casualty_dict_list = []
    
    def casualtyPosCallback(self, msg : CasualtyFixArray) -> None:
        #print(len(msg.casualties))
        #print(len(self.casualty_dict_list))
        self.drone_casualty_dict_list = []

        for elt in msg.casualties:
            new_casualty = self.initFalconEntry()
            new_casualty["id"] = elt.casualty_id
            new_casualty["lat"] = elt.location.latitude
            new_casualty["lon"] = elt.location.longitude
            self.drone_casualty_dict_list.append(new_casualty)

        self.mutex.acquire()
        with open(self.drone_db, "w") as f:
            json.dump(self.drone_casualty_dict_list, f, indent=2)
    
        update_drone_casualty_db(self.db_path, self.config["ugv_uav_distance_threshold"])

        with open(self.matching_table, "r") as f:
            matching_table = json.load(f)
        
        for elt in matching_table:
            if elt["action"] == "init" and not(elt["pos_sent"]):
                init_position(id=elt["casualty_id"], lat=elt["uav"]["lat"], lon=elt["uav"]["lon"], time=elt["timestamp"])
                elt["action"] = ""
                elt["pos_sent"] = True
        
        with open(self.matching_table, "w") as f:
            json.dump(matching_table, f, indent=2)
        self.mutex.release()

        total_posts()

    def compressedImageMsgToArray(self, msg : CompressedImage) -> np.ndarray:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image

    def deimosCallback(self, msg : ScoreCardString) -> None:
        report = parse_report_string_as_json(msg.scorecard.data)
        image = self.compressedImageMsgToArray(msg.image)

        self.mutex.acquire()
        self.writeJackalReport("deimos", report, image)
        self.mutex.release()

    def phobosCallback(self, msg : ScoreCardString) -> None: 
        report = parse_report_string_as_json(msg.scorecard.data)
        image = self.compressedImageMsgToArray(msg.image)
        
        self.mutex.acquire()
        self.writeJackalReport("phobos", report, image)
        self.mutex.release()

    def titaniaCallback(self, msg : ScoreCardString) -> None: 
        report = parse_report_string_as_json(msg.scorecard.data)
        image = self.compressedImageMsgToArray(msg.image)

        self.mutex.acquire()
        self.writeJackalReport("titania", report, image)
        self.mutex.release()

    def oberonCallback(self, msg : ScoreCardString) -> None: 
        report = parse_report_string_as_json(msg.scorecard.data)
        image = self.compressedImageMsgToArray(msg.image)

        self.mutex.acquire()
        self.writeJackalReport("oberon", report, image)
        self.mutex.release()
    
    def writeJackalReport(self, robot : str, report : str, image : np.ndarray) -> None:

        new_jackal_entry = self.initJackalEntry()
        new_jackal_entry["id"] = len(self.jackal_casualty_dict_list)
        new_jackal_entry["lat"] = report["location"]["latitude"]
        new_jackal_entry["lon"] = report["location"]["longitude"]
        new_jackal_entry["report"] = report
        new_jackal_entry["whoami"] = robot
        new_jackal_entry["image_path"] = os.path.join(self.db_path, f"{robot}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")

        cv2.imwrite(new_jackal_entry["image_path"], image)

        self.jackal_casualty_dict_list.append(new_jackal_entry)

        with open(self.jackal_db, "w") as f:
            json.dump(self.jackal_casualty_dict_list, f, indent=2)
        
        update_jackal_casualty_db(self.db_path, self.config["ugv_uav_distance_threshold"], self.config["ugv_ugv_distance_threshold"])

        with open(self.matching_table, "r") as f:
            matching_table = json.load(f)

        for elt in matching_table:
            if elt["action"] == "init_update":
                init_supplement(elt["casualty_id"], elt["report"])
                elt["image_path"] = new_jackal_entry["image_path"]
                #try:
                if elt["image_path"] is not None:
                    submit_image(elt["image_path"],elt["report"]["severe_hemorrhage"]["time_ago"], elt["casualty_id"])
                #except:
                #    print("[Scorecard][ERROR] Failed to submit image")
                elt["action"] = ""
                elt["pos_sent"] = True
                elt["init_supp_sent"] = True
            elif elt["action"] == "update" and elt["init_supp_sent"]:
                print("Should be updating")
                if elt["update_sent"]:
                    print("[Scorecard][STATUS] Two Jackals have already triaged this. Update already sent for casualty_id ", elt["casualty_id"])
                else:
                    update_casualty(elt["casualty_id"], elt["report"])
                    elt["action"] = ""                
                    elt["update_sent"] = True
        with open(self.matching_table, "w") as f:
            json.dump(matching_table, f, indent=2)

        total_posts()
        

    def initJackalEntry(self) -> Dict:
        return {"id": None,
                "lat": None,
                "lon": None,
                "report": {},
                "whoami": None,
                "image_path": None}

    def initFalconEntry(self) -> Dict:
        return {"id": None,
                "lat": None,
                "lon": None}



    """
    def deimosImageCallback(self, msg):
        print("[Scorecard][STATUS] Received image from /deimos/camera/image")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        timestamp = int(rospy.Time.now().to_sec())
        image_path = f"/home/dtc/ws/data/casualty_images/deimos_image_{timestamp}.jpg"
        self.most_recent_deimos_image_path = image_path
        
        # Save the image to a file
        cv2.imwrite(image_path, cv_image)
        print(f"[Scorecard][STATUS] Image saved to {image_path}")

    def deimosScoreCallback(self, msg):
        print("[Scorecard][STATUS] Received report from /deimos/report_status")
        print("[Scorecard][STATUS] Publishing report status")

        #extract the string from the message
        report_str = msg.data
        print(report_str)
        report = parse_report_string(report_str)
        new_jackal_entry = copy.deepcopy(jackal_db_entry)
        new_jackal_entry["id"] = len(self.jackal_casualty_dict_list)
        new_jackal_entry["lat"] = report["location"]["latitude"]
        new_jackal_entry["lon"] = report["location"]["longitude"]
        new_jackal_entry["report"] = report
        new_jackal_entry["image_path"] = self.most_recent_deimos_image_path

        self.jackal_casualty_dict_list.append(new_jackal_entry)

        with open(self.jackal_db, "w") as f:
            json.dump(self.jackal_casualty_dict_list, f, indent=2)
        
        update_jackal_casualty_db(self.config["ugv_uav_distance_threshold"], self.config["ugv_ugv_distance_threshold"])

        with open(self.matching_table, "r") as f:
            matching_table = json.load(f)

        for elt in matching_table:
            if elt["action"] == "init_update":
                init_supplement(elt["casualty_id"], elt["report"])
                elt["image_path"] = self.most_recent_deimos_image_path
                try:
                    if elt["image_path"] is not None:
                        submit_image(elt["casualty_id"], elt["image_path"])
                except:
                    print("[Scorecard][ERROR] Failed to submit image")
                elt["action"] = ""
                elt["pos_sent"] = True
                elt["init_supp_sent"] = True
            elif elt["action"] == "update" and elt["init_supp_sent"]:
                print("Should be updating")
                if elt["update_sent"]:
                    print("[Scorecard][STATUS] Two Jackals have already triaged this. Update already sent for casualty_id ", elt["casualty_id"])
                else:
                    update_casualty(elt["casualty_id"], elt["report"])
                    elt["action"] = ""                
                    elt["update_sent"] = True
        with open(self.matching_table, "w") as f:
            json.dump(matching_table, f, indent=2)

        total_posts()

    def phobosImageCallback(self, msg):
        print("[Scorecard][STATUS] Received image from /phobos/camera/image")
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        timestamp = int(rospy.Time.now().to_sec())
        image_path = f"/home/dtc/ws/data/casualty_images/phobos_image_{timestamp}.jpg"
        self.most_recent_deimos_image_path = image_path
        
        # Save the image to a file
        cv2.imwrite(image_path, cv_image)
        print(f"[Scorecard][STATUS] Image saved to {image_path}")

    def phobosScoreCallback(self, msg):
        print("[Scorecard][STATUS] Received report from /phobos/report_status")
        print("[Scorecard][STATUS] Publishing report status")

        #extract the string from the message
        report_str = msg.data
        print(report_str)
        report = parse_report_string(report_str)
        new_jackal_entry = copy.deepcopy(jackal_db_entry)
        new_jackal_entry["id"] = len(self.jackal_casualty_dict_list)
        new_jackal_entry["lat"] = report["location"]["latitude"]
        new_jackal_entry["lon"] = report["location"]["longitude"]
        new_jackal_entry["report"] = report
        new_jackal_entry["image_path"] = self.most_recent_phobos_image_path

        self.jackal_casualty_dict_list.append(new_jackal_entry)

        with open(self.jackal_db, "w") as f:
            json.dump(self.jackal_casualty_dict_list, f, indent=2)
        
        update_jackal_casualty_db(self.config["ugv_uav_distance_threshold"], self.config["ugv_ugv_distance_threshold"])

        with open(self.matching_table, "r") as f:
            matching_table = json.load(f)

        for elt in matching_table:
            if elt["action"] == "init_update":
                init_supplement(elt["casualty_id"], elt["report"])
                elt["image_path"] = self.most_recent_phobos_image_path
                try:
                    if elt["image_path"] is not None:
                        submit_image(elt["casualty_id"], elt["image_path"])
                except:
                    print("[Scorecard][ERROR] Failed to submit image")
                elt["action"] = ""
                elt["pos_sent"] = True
                elt["init_supp_sent"] = True
            elif elt["action"] == "update" and elt["init_supp_sent"]:
                print("Should be updating")
                if elt["update_sent"]:
                    print("[Scorecard][STATUS] Two Jackals have already triaged this. Update already sent for casualty_id ", elt["casualty_id"])
                else:
                    update_casualty(elt["casualty_id"], elt["report"])
                    elt["action"] = ""                
                    elt["update_sent"] = True
        with open(self.matching_table, "w") as f:
            json.dump(matching_table, f, indent=2)

        total_posts()
    """
if __name__ == "__main__":
    rospy.init_node("scorecard_submitter")
    SubmissionNode()    
    rospy.spin()

    
