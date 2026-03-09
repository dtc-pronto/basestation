#!/usr/bin/env python3
"""
    Jason Hughes
    September 2025

    spoof the ScoreCardString msg
    from robots in the swarm
"""

import os
import cv2
import utm
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from dtc_msgs.msg import ScoreCardString, CasualtyFixArray, CasualtyFix
from sensor_msgs.msg import CompressedImage

from typing import Dict, Tuple, List

def initReport(node: Node) -> Dict:
    now = node.get_clock().now().nanoseconds / 1e9
    hr = np.random.randint(60, 100)
    rr = np.random.randint(12, 30)
    alertness_ocular = np.random.randint(0, 1)
    alertness_verbal = np.random.randint(0, 2)
    alertness_motor = np.random.randint(0, 2)
    severe_hemorrhage = np.random.randint(0, 1)
    respiratory_distress = np.random.randint(0, 1)
    trauma_head = np.random.randint(0, 2)
    trauma_torso = np.random.randint(0, 2)
    trauma_lower_ext = np.random.randint(0, 2)
    trauma_upper_ext = np.random.randint(0, 2)
    return {
      "hr": {
        "value": hr,
        "time_ago": now
      },
      "rr": {
        "value": rr,
        "time_ago": now
      },
      "alertness_ocular": {
        "value": alertness_ocular,
        "time_ago": now
      },
      "alertness_verbal": {
        "value": alertness_verbal,
        "time_ago": now
      },
      "alertness_motor": {
        "value": alertness_motor,
        "time_ago": now
      },
      "severe_hemorrhage": {
        "value": severe_hemorrhage,
        "time_ago": now
      },
      "respiratory_distress": {
        "value": respiratory_distress,
        "time_ago": now
      },
      "trauma_head": trauma_head,
      "trauma_torso": trauma_torso,
      "trauma_lower_ext": trauma_lower_ext,
      "trauma_upper_ext": trauma_upper_ext,
      "temp": {
        "value": 98,
        "time_ago": now
      },
      "casualty_id": 2,
      "team": "PennPronto",
      "system": "JackalNVILA",
      "location": {
        "latitude": 0.0,
        "longitude": 0.0,
        "time_ago": now
      }}

def initGps(count : int) -> List[float]:
    
    coords = [[39.942356, -75.199110],
              [39.942395, -75.199187],
              [39.942449, -75.199307],
              [39.942539, -75.199448]]
    return coords[count]

class ScoreCardSpoof(Node):

    def __init__(self) -> None:
        super().__init__('spoofer_node')

        self.fix_arr_ = list()
        
        # Declare and get the robots parameter
        self.declare_parameter('robots', [''])
        robots = self.get_parameter('robots').value

        if "deimos" in robots:
            self.deimos_timer_ = self.create_timer(7.0, self.deimosCallback)
            self.deimos_pub_ = self.create_publisher(ScoreCardString, "/deimos/report_status", 1)
            self.get_logger().info("Deimos timer created")
            self.deimos_count_ = 0
        if "phobos" in robots:
            self.phobos_timer_ = self.create_timer(2.0, self.phobosCallback)
            self.phobos_pub_ = self.create_publisher(ScoreCardString, "/phobos/report_status", 1)
            self.get_logger().info("Phobos timer created")
            self.phobos_count_ = 0
        if "titania" in robots:
            self.titania_timer_ = self.create_timer(1.0, self.titaniaCallback)
            self.titania_pub_ = self.create_publisher(ScoreCardString, "/titania/report_status", 1)
            self.get_logger().info("Titania timer created")
            self.titania_count_ = 0
        if "oberon" in robots:
            self.oberon_timer_ = self.create_timer(8.0, self.oberonCallback)
            self.oberon_pub_ = self.create_publisher(ScoreCardString, "/oberon/report_status", 1)
            self.get_logger().info("Oberon timer created")
            self.oberon_count_ = 0

        if "dione" in robots:
            self.dione_timer_ = self.create_timer(3.0, self.dioneCallback)
            self.dione_pub_ = self.create_publisher(CasualtyFixArray, "/dione/casualty_info", 1)
            self.get_logger().info("Dione timer created")
            self.dione_count_ = 0
            

    def getGpsCoord(self, count : int) -> Tuple[float, float]:
        ll = initGps(count)
        e, n, zone, zid = utm.from_latlon(ll[0], ll[1])
        e += np.random.beta(5, 5)
        n += np.random.beta(5, 5)

        coord = utm.to_latlon(e, n, zone, zid)
        return coord

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

    def createStringMsg(self, report : Dict) -> String:
        msg = String()
        msg.data = json.dumps(report)
        return msg

    def createRandomMsg(self, count : int, robot : str) -> ScoreCardString:
        report = initReport(self)
        coords = self.getGpsCoord(count)
        
        report["location"]["latitude"] = coords[0]
        report["location"]["longitude"] = coords[1]
        #report["system"] = robot

        image = np.random.randint(0, 256, (480, 480, 3), dtype=np.uint8)
        img_msg = self.createCompressedImageMsg(image)
        str_msg = self.createStringMsg(report)

        msg = ScoreCardString()
        msg.scorecard = str_msg
        msg.image = img_msg

        return msg

    def createCasualtyFix(self, count : int) -> CasualtyFix:
        coords = self.getGpsCoord(count)
        msg = CasualtyFix()
        msg.casualty_id = count
        msg.location.latitude = coords[0]
        msg.location.longitude = coords[1]
        msg.time_ago = self.get_clock().now().to_msg()

        return msg

    def deimosCallback(self) -> None:
        if self.deimos_count_ == 4:
            self.deimos_timer_.cancel()
            return
        msg = self.createRandomMsg(self.deimos_count_, "deimos")
        self.get_logger().info("Spoofing Deimos")
        self.deimos_pub_.publish(msg)
        self.deimos_count_+=1

    def phobosCallback(self) -> None:
        if self.phobos_count_ == 4:
            self.phobos_timer_.cancel()
            return
        msg = self.createRandomMsg(self.phobos_count_, "phobos")
        self.get_logger().info("Spoofing Phobos")
        self.phobos_pub_.publish(msg)
        self.phobos_count_+=1

    def titaniaCallback(self) -> None:
        if self.titania_count_ == 4:
            self.titania_timer_.cancel()
            return
        msg = self.createRandomMsg(self.titania_count_, "titania")
        self.get_logger().info("Spoofing Titania")
        self.titania_pub_.publish(msg)
        self.titania_count_+=1

    def oberonCallback(self) -> None:
        if self.oberon_count_ == 4:
            self.oberon_timer_.cancel()
            return
        msg = self.createRandomMsg(self.oberon_count_, "oberon")
        self.get_logger().info("Spoofing Oberon")
        self.oberon_pub_.publish(msg)
        self.oberon_count_+=1

    def dioneCallback(self) -> None:
        if self.dione_count_ == 4:
            self.dione_timer_.cancel()
            return

        msg = self.createCasualtyFix(self.dione_count_)
        self.get_logger().info("Spoofing Dione")

        self.fix_arr_.append(msg)

        msg_arr = CasualtyFixArray()
        msg_arr.header.stamp = self.get_clock().now().to_msg()
        msg_arr.casualties = self.fix_arr_

        self.dione_pub_.publish(msg_arr)
        self.dione_count_ += 1


if __name__ == "__main__":
    rclpy.init()
    
    node = ScoreCardSpoof()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
