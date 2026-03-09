#!/usr/bin/env python3
import os
import sys
import re

script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(script_dir)

import threading 
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image, CompressedImage
from helpers import gps_distance


"""
    Edward Zhang
    Jason Hughes
    June 2025
    Node that runs VLM inference on the Jackal robot.
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2025
"""
import cv2
import numpy as np
from submission import start_run, submit_image, update_casualty, init_position, init_supplement, total_posts, parse_report_string_as_json
from helpers import gps_distance, closest_casualty, update_drone_casualty_db, update_jackal_casualty_db, parse_report_string

from std_msgs.msg import Bool, String, UInt8, Float32
from dtc_msgs.msg import ScoreCard, CasualtyFixArray, CasualtyFix, ScoreCardString
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from basestation_msgs.msg import ServerReport, InitialReport, FullReport
from cv_bridge import CvBridge
import json
import copy
from datetime import datetime
from typing import Dict

#TODO: Add a hashing dictionary to keep track of victims and corresponding reports
class Casualty:
    def __init__(self, node, id, position, image_id=None, report=None):
        self.id = id
        self.position = position
        self.image_id = image_id
        self.report = report
        self.time = node.get_clock().now().nanoseconds / 1e9

    def to_dict(self):
        return {
            "id": self.id,
            "position": self.position,
            "image_id": self.image_id,
            "report": self.report,
            "time": self.time
        }

class SubmissionNode(Node):
    def __init__(self):
        super().__init__('scorecard_submitter')
        
        # Declare parameters
        self.declare_parameter('start_run', False)
        self.declare_parameter('threshold_path', '')
        self.declare_parameter('data_path', '')
        self.declare_parameter('robots', [""])
        
        if self.get_parameter('start_run').value:
            self.get_logger().info("[SENDER] Starting run")
            start_run()
        
        threshold_config_path = self.get_parameter('threshold_path').value
        with open(threshold_config_path, "r") as f:
            self.config = json.load(f)

        
        data_path = self.get_parameter('data_path').value
        folder_name = f"casualty_db_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.db_path = os.path.join(data_path, folder_name)
        os.makedirs(self.db_path, exist_ok=False)
        self.get_logger().info(f"[SENDER] Using database at {self.db_path}")
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

        robots = self.get_parameter('robots').value
        self.mutex = threading.Lock()

        # QoS profile for publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        if "deimos" in robots:
            self.get_logger().info("[SENDER] Adding Deimos to submission")
            self.deimos_sub = self.create_subscription(
                ScoreCardString, 
                "/deimos/report_status", 
                self.deimosCallback,
                qos_profile)
        if "phobos" in robots:
            self.get_logger().info("[SENDER] Adding Phobos to submission")
            self.phobos_sub = self.create_subscription(
                ScoreCardString,
                "/phobos/report_status", 
                self.phobosCallback,
                qos_profile)
        if "titania" in robots:
            self.get_logger().info("[SENDER] Adding Titania to submission")
            self.titania_sub = self.create_subscription(
                ScoreCardString,
                "/titania/report_status", 
                self.titaniaCallback,
                qos_profile)
        if "oberon" in robots:
            self.get_logger().info("[SENDER] Adding Oberon to submission")
            self.oberon_sub = self.create_subscription(
                ScoreCardString,
                "/oberon/report_status", 
                self.oberonCallback,
                qos_profile)
        if "dione" in robots:
            self.get_logger().info("[SENDER] Adding Dione to submission")
            self.dione_position_sub = self.create_subscription(
                CasualtyFixArray,
                "/dione/casualty_info", 
                self.casualtyPosCallback,
                qos_profile)

        self.server_pub_ = self.create_publisher(ServerReport, "/basestation/scoring_server_report", qos_profile)
        self.initial_pub_ = self.create_publisher(InitialReport, "/basestation/initial_report", qos_profile)
        self.full_report_pub_ = self.create_publisher(FullReport, "/basestation/full_report", qos_profile)

        self.drone_casualty_dict_list = []
        self.jackal_casualty_dict_list = []
    
    def casualtyPosCallback(self, msg : CasualtyFixArray) -> None:
        #print(len(msg.casualties))
        #print(len(self.casualty_dict_list))
        r = None
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

        time_now = self.get_clock().now().to_msg().sec
        update_drone_casualty_db(self.db_path, time_now=time_now, ugv_uav_threshold=self.config["ugv_uav_distance_threshold"])

        with open(self.matching_table, "r") as f:
            matching_table = json.load(f)
        
        for elt in matching_table:
            if elt["action"] == "init" and not(elt["pos_sent"]):
                time_now = self.get_clock().now().to_msg().sec
                r = init_position(id=elt["casualty_id"], lat=elt["uav"]["lat"], lon=elt["uav"]["lon"], time=elt["timestamp"], time_now=time_now)
                elt["action"] = ""
                elt["pos_sent"] = True
                self.publishInitialReport(elt["casualty_id"], elt["uav"]["lat"], elt["uav"]["lon"], elt["timestamp"])

        with open(self.matching_table, "w") as f:
            json.dump(matching_table, f, indent=2)
        self.mutex.release()
        total_posts()
        if r is not None:
            self.publishServerReport(r, "dione")

    def publishServerReport(self, r, robot : str) -> None:
        msg = ServerReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.code = r.status_code
        msg.report = json.dumps(r.json())
        msg.robot = robot

        self.server_pub_.publish(msg)

    def publishInitialReport(self, cid : int, lat : float, lon : float, time : float) -> None:
        msg = InitialReport()
        msg.casualty_id = int(cid)
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.time_ago = float(time)

        self.initial_pub_.publish(msg)

    def compressedImageMsgToArray(self, msg : CompressedImage) -> np.ndarray:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image

    def deimosCallback(self, msg : ScoreCardString) -> None:
        time_now = self.get_clock().now().to_msg().sec
        report = parse_report_string_as_json(msg.scorecard.data, time_now=time_now)
        image = self.compressedImageMsgToArray(msg.image)

        self.mutex.acquire()
        r = self.writeJackalReport("deimos", report, image)
        self.mutex.release()
        if r is not None:
            self.publishServerReport(r, "deimos")

    def phobosCallback(self, msg : ScoreCardString) -> None: 
        time_now = self.get_clock().now().to_msg().sec
        report = parse_report_string_as_json(msg.scorecard.data, time_now=time_now)
        image = self.compressedImageMsgToArray(msg.image)
        
        self.mutex.acquire()
        r = self.writeJackalReport("phobos", report, image)
        self.mutex.release()
        if r is not None:
            self.publishServerReport(r, "phobos")

    def titaniaCallback(self, msg : ScoreCardString) -> None: 
        time_now = self.get_clock().now().to_msg().sec
        report = parse_report_string_as_json(msg.scorecard.data, time_now=time_now)
        image = self.compressedImageMsgToArray(msg.image)

        self.mutex.acquire()
        r = self.writeJackalReport("titania", report, image)
        self.mutex.release()
        if r is not None:
            self.publishServerReport(r, "titania")

    def oberonCallback(self, msg : ScoreCardString) -> None: 
        time_now = self.get_clock().now().to_msg().sec
        report = parse_report_string_as_json(msg.scorecard.data, time_now=time_now)
        image = self.compressedImageMsgToArray(msg.image)

        self.mutex.acquire()
        r = self.writeJackalReport("oberon", report, image)
        self.mutex.release()
        if r is not None:
            self.publishServerReport(r, "oberon")
    
    def writeJackalReport(self, robot : str, report : str, image : np.ndarray):

        r = None
        new_jackal_entry = self.initJackalEntry()
        new_jackal_entry["id"] = len(self.jackal_casualty_dict_list)
        new_jackal_entry["lat"] = report["location"]["latitude"]
        new_jackal_entry["lon"] = report["location"]["longitude"]
        new_jackal_entry["report"] = report
        new_jackal_entry["whoami"] = robot
        new_jackal_entry["image_path"] = os.path.join(self.db_path, f"{robot}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")

        cv2.imwrite(new_jackal_entry["image_path"], image)

        self.jackal_casualty_dict_list.append(new_jackal_entry)
        print(self.jackal_casualty_dict_list)
        with open(self.jackal_db, "w") as f:
            json.dump(self.jackal_casualty_dict_list, f, indent=2)
        
        time_now = self.get_clock().now().to_msg().sec
        update_jackal_casualty_db(self.db_path, time_now=time_now, ugv_uav_threshold=self.config["ugv_uav_distance_threshold"], ugv_ugv_threshold=self.config["ugv_ugv_distance_threshold"])

        with open(self.matching_table, "r") as f:
            matching_table = json.load(f)

        for elt in matching_table:
            if elt["action"] == "init_update":
                time_now = self.get_clock().now().to_msg().sec
                init_supplement(elt["casualty_id"], elt["report"], time_now)
                elt["image_path"] = new_jackal_entry["image_path"]
                self.publishFullReport(elt["casualty_id"], elt["report"], elt["image_path"]) 
                if elt["image_path"] is not None:
                    time_now = self.get_clock().now().to_msg().sec
                    r = submit_image(elt["image_path"],elt["report"]["severe_hemorrhage"]["time_ago"], time_now, elt["casualty_id"])
                elt["action"] = ""
                elt["pos_sent"] = True
                elt["init_supp_sent"] = True
            elif elt["action"] == "update" and elt["init_supp_sent"]:
                print("Should be updating")
                if elt["update_sent"]:
                    print("[Scorecard][STATUS] Two Jackals have already triaged this. Update already sent for casualty_id ", elt["casualty_id"])
                else:
                    time_now = self.get_clock().now().to_msg().sec
                    r = update_casualty(elt["casualty_id"], elt["report"], time_now)
                    elt["action"] = ""                
                    elt["update_sent"] = True
        with open(self.matching_table, "w") as f:
            json.dump(matching_table, f, indent=2)  
        total_posts()
        return r
        

    def publishFullReport(self, cid : int, report : Dict, image_path : str) -> None:
        msg = FullReport()
        msg.casualty_id = cid
        msg.report = json.dumps(report)
        img = cv2.imread(image_path)
        cimg = self.createCompressedImageMsg(img)
        msg.image = cimg

        self.full_report_pub_.publish(msg)

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

    def createCompressedImageMsg(self, image : np.ndarray ) -> CompressedImage:
        compressed_img = CompressedImage()
        compressed_img.header.stamp = self.get_clock().now().to_msg()
        compressed_img.header.frame_id = "camera_frame"
        
        # Set format
        compressed_img.format = 'jpg'
        
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, 90]
        _, compressed_data = cv2.imencode('.jpg', image, encode_params)
        
        compressed_img.data = compressed_data.tobytes()
        
        return compressed_img

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
    rclpy.init()
    node = SubmissionNode()    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    
