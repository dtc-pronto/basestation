"""
    Jason Hughes
    July 2025

    spoof the health status of the jackal
"""

import rospy
from dtc_msgs.msg import JackalStatus
from dtc_msgs.msg import FalconStatus
from rospy.timer import TimerEvent

class StatusSpoofer:

    def __init__(self) -> None:
        
        self.phobos_pub_ = rospy.Publisher("/phobos/status", JackalStatus, queue_size=1)

        rospy.Timer(rospy.Duration(5.0), self.phobos_callback)

    def phobos_callback(self, event : TimerEvent) -> None:
        msg = JackalStatus()

        self.phobos_pub_.publish(msg)

if __name__ == "__main__":
    rospy.init_node("status_spoofer")
    
    node = StatusSpoofer()

    rospy.spin()
