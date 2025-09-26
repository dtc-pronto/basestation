"""
    Jason Hughes
    July 2025

    spoof the health status of the jackal
"""

import rospy
import random
from dtc_msgs.msg import JackalStatus
from dtc_msgs.msg import FalconStatus
from basestation_msgs.msg import ServerReport
from rospy.timer import TimerEvent

class StatusSpoofer:

    def __init__(self) -> None:
        
        self.phobos_pub_ = rospy.Publisher("/phobos/status", JackalStatus, queue_size=1)
        self.deimos_pub_ = rospy.Publisher("/deimos/status", JackalStatus, queue_size=1)
        self.oberon_pub_ = rospy.Publisher("/oberon/status", JackalStatus, queue_size=1)
        self.titania_pub_ = rospy.Publisher("/titania/status", JackalStatus, queue_size=1)
        self.dione_pub_ = rospy.Publisher("/dione/status", FalconStatus, queue_size=1)
        self.server_pub_ = rospy.Publisher("/basestation/scoring_server_report", ServerReport, queue_size=1)    

        rospy.Timer(rospy.Duration(5.0), self.phobos_callback)
        rospy.Timer(rospy.Duration(5.0), self.deimos_callback)
        rospy.Timer(rospy.Duration(5.0), self.oberon_callback)
        rospy.Timer(rospy.Duration(5.0), self.titania_callback)
        rospy.Timer(rospy.Duration(5.0), self.dione_callback)
    
        rospy.Timer(rospy.Duration(3.0), self.server_callback)

        # use true 80 percent, false 20 percent
        self.dist_ = [True, True, True, True, True, True, True, True, False, False]

    def generate_random_msg(self) -> JackalStatus:
        msg = JackalStatus()
        msg.rgb = self.dist_[random.randint(0,9)]
        msg.thermal = self.dist_[random.randint(0,9)]
        msg.gps = self.dist_[random.randint(0,9)]
        msg.rtk = self.dist_[random.randint(0,9)]
        msg.jeti = self.dist_[random.randint(0,9)]
        msg.outser = self.dist_[random.randint(0,9)]
        msg.mic = self.dist_[random.randint(0,9)]
        return msg
    
    def generate_random_falcon_msg(self) -> FalconStatus:
        msg = FalconStatus()
        msg.rgb = self.dist_[random.randint(0,9)]
        msg.thermal = self.dist_[random.randint(0,9)]
        msg.gps = self.dist_[random.randint(0,9)]
        msg.rtk = self.dist_[random.randint(0,9)]

        return msg

    def phobos_callback(self, event : TimerEvent) -> None:
        msg = self.generate_random_msg()
        self.phobos_pub_.publish(msg)

    def deimos_callback(self, event : TimerEvent) -> None: 
        msg = self.generate_random_msg()
        self.deimos_pub_.publish(msg)

    def oberon_callback(self, event : TimerEvent) -> None: 
        msg = self.generate_random_msg()
        self.oberon_pub_.publish(msg)

    def titania_callback(self, event : TimerEvent) -> None: 
        msg = self.generate_random_msg()
        self.titania_pub_.publish(msg)

    def dione_callback(self, event : TimerEvent) -> None:
        msg = self.generate_random_falcon_msg()
        self.dione_pub_.publish(msg)

    def server_callback(self, event : TimerEvent) -> None:
        msg = ServerReport()
        msg.code = 200
        msg.report.data = "good soup report"
        msg.robot.data = "deimos"

        self.server_pub_.publish(msg)

if __name__ == "__main__":
    rospy.init_node("status_spoofer")
    
    node = StatusSpoofer()

    rospy.spin()
