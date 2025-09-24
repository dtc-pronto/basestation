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
import rospy

from std_msgs.msg import String
from dtc_msgs.msg import ScoreCardString, CasualtyFixArray, CasualtyFix
from sensor_msgs.msg import CompressedImage

from typing import Dict, Tuple, List

def initReport() -> Dict:
    return {
      "hr": {
        "value": 0,
        "time_ago": rospy.Time.now().to_sec()
      },
      "rr": {
        "value": 0,
        "time_ago": rospy.Time.now().to_sec()
      },
      "alertness_ocular": {
        "value": 0,
        "time_ago":  rospy.Time.now().to_sec()
      },
      "alertness_verbal": {
        "value": 0,
        "time_ago":  rospy.Time.now().to_sec()
      },
      "alertness_motor": {
        "value": 0,
        "time_ago":  rospy.Time.now().to_sec()
      },
      "severe_hemorrhage": {
        "value": 0,
        "time_ago":  rospy.Time.now().to_sec()
      },
      "respiratory_distress": {
        "value": 0,
        "time_ago":  rospy.Time.now().to_sec()
      },
      "trauma_head": 0,
      "trauma_torso": 0,
      "trauma_lower_ext": 0,
      "trauma_upper_ext": 0,
      "temp": {
        "value": 98,
        "time_ago":  rospy.Time.now().to_sec()
      },
      "casualty_id": 2,
      "team": "PennPronto",
      "system": "JackalNVILA",
      "location": {
        "latitude": 0.0,
        "longitude": 0.0,
        "time_ago":  rospy.Time.now().to_sec()
      }}

def initGps(count : int) -> List[float]:
    
    coords = [[39.942356, -75.199110],
              [39.942395, -75.199187],
              [39.942449, -75.199307],
              [39.942539, -75.199448]]
    return coords[count]

class ScoreCardSpoof:

    def __init__(self) -> None:

        self.fix_arr_ = list()
        robots = rospy.get_param("/robots")

        if "deimos" in robots:
            rospy.Timer(rospy.Duration(7.0), self.deimosCallback)
            self.deimos_pub_ = rospy.Publisher("/deimos/report_status", ScoreCardString, queue_size=1)
            self.deimos_count_ = 0
        if "phobos" in robots:
            rospy.Timer(rospy.Duration(2.0), self.phobosCallback)
            self.phobos_pub_ = rospy.Publisher("/phobos/report_status", ScoreCardString, queue_size=1)
            self.phobos_count_ = 2
        if "titania" in robots:
            rospy.Timer(rospy.Duration(1.0), self.titaniaCallback)
            self.titania_pub_ = rospy.Publisher("/titania/report_status", ScoreCardString, queue_size=1)
            self.titania_count_ = 3
        if "oberon" in robots:
            rospy.Timer(rospy.Duration(8.0), self.oberonCallback)
            self.oberon_pub_ = rospy.Publisher("/oberon/report_status", ScoreCardString, queue_size=1)
            self.oberon_count_ = 1

        if "dione" in robots:
            rospy.Timer(rospy.Duration(3.0), self.dioneCallback)
            self.dione_pub_ = rospy.Publisher("/dione/casualty_info", CasualtyFixArray, queue_size=1)
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
        compressed_img.header.stamp = rospy.Time.now()
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
        report = initReport()
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

    def createCasualtyFix(self, count : int) -> CasualtyFixArray:
        coords = self.getGpsCoord(count)
        msg = CasualtyFix()
        msg.casualty_id = count
        msg.location.latitude = coords[0]
        msg.location.longitude = coords[1]
        msg.time_ago.data = rospy.Time.now()


        return msg

    def deimosCallback(self, event) -> None:
        rospy.loginfo("Spoofing Deimos")
        if self.deimos_count_ == 4: return
        msg = self.createRandomMsg(self.deimos_count_, "deimos")
        self.deimos_pub_.publish(msg)
        self.deimos_count_+=1

    def phobosCallback(self, event) -> None:
        if self.phobos_count_ == 4: return
        rospy.loginfo("Spoofing Phobos")
        msg = self.createRandomMsg(self.phobos_count_, "phobos")
        self.phobos_pub_.publish(msg)
        self.phobos_count_+=1

    def titaniaCallback(self, event) -> None: 
        rospy.loginfo("Spoofing Titania")
        if self.titania_count_ == 4: return
        msg = self.createRandomMsg(self.titania_count_, "titania")
        self.titania_pub_.publish(msg)
        self.titania_count_+=1

    def oberonCallback(self, event) -> None:
        rospy.loginfo("Spoofing Oberon")
        if self.oberon_count_ == 4: return
        msg = self.createRandomMsg(self.oberon_count_, "oberon")
        self.oberon_pub_.publish(msg)
        self.oberon_count_+=1

    def dioneCallback(self, event) -> None:
        rospy.loginfo("Spoofing Dione")
        if self.dione_count_ == 4: return

        msg = self.createCasualtyFix(self.dione_count_)

        self.fix_arr_.append(msg)

        msg_arr = CasualtyFixArray()
        msg_arr.header.stamp = rospy.Time.now()
        msg_arr.casualties = self.fix_arr_

        self.dione_pub_.publish(msg_arr)
        self.dione_count_ += 1


if __name__ == "__main__":
    rospy.init_node("spoofer_node")

    ScoreCardSpoof()

    rospy.spin()
