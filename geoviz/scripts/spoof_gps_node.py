"""
    Jason Hughes
    July 2025

    Node to spoof GPS around pennovation    
    for the DTC robots

    This node is solely for debugging the visualizer
"""

import utm
import rospy
import random
from rospy.timer import TimerEvent
from sensor_msgs.msg import NavSatFix
from dtc_msgs.msg import NavSatFixArray

NUM_CASUALTIES = 10

class Spoofer:

    def __init__(self) -> None:
        self.starting_easting_ = 483048.67720218655
        self.starting_northing_ = 4421219.126722925

        self.northing_radius_ = 25
        self.easting_radius_ = 10

        self.zone_int_ = 18
        self.zone_str_ = 'S'

        self.casualty_ = 0

        self.use_array_ = rospy.get_param("/geoviz/app/use_array", False)
        
        self.phobos_pub_ = rospy.Publisher("/phobos/ublox/fix", NavSatFix, queue_size=10)
        self.deimos_pub_ = rospy.Publisher("/deimos/ublox/fix", NavSatFix, queue_size=10)
        self.oberon_pub_ = rospy.Publisher("/oberon/ublox/fix", NavSatFix, queue_size=10)
        self.titania_pub_ = rospy.Publisher("/titania/ublox/fix", NavSatFix, queue_size=10)
        self.dione_pub_ = rospy.Publisher("/dione/mavros/global_position/fix/raw", NavSatFix, queue_size=10)

        print("[SPOOFER] Using NavSatFixArray: ", self.use_array_)
        if not self.use_array_:
            self.cas_pub_ = rospy.Publisher("/dione/casualty", NavSatFix, queue_size=10)
        else:
            self.casualties_ = list()
            self.cas_pub_ = rospy.Publisher("/dione/casualty", NavSatFixArray, queue_size=1)

        rospy.Timer(rospy.Duration(2.0), self.phobos_callback)
        rospy.Timer(rospy.Duration(2.0), self.deimos_callback)
        rospy.Timer(rospy.Duration(5.0), self.oberon_callback)
        rospy.Timer(rospy.Duration(10.0), self.titania_callback)
        rospy.Timer(rospy.Duration(2.0), self.dione_callback)
        rospy.Timer(rospy.Duration(5.0), self.casualty_callback)

        print("[SPOOFER] Initialized GPS Spoofer for: Phobos, Diemos, Oberon, Titania, and Dione")

    def phobos_callback(self, event : TimerEvent) -> None:
        easting = self.starting_easting_ + random.uniform(-self.easting_radius_, self.easting_radius_)
        northing = self.starting_northing_ + random.uniform(-self.northing_radius_, self.northing_radius_)

        lat, lon = utm.to_latlon(easting, northing, self.zone_int_, self.zone_str_)

        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.header.frame_id = "phobos"

        self.phobos_pub_.publish(msg)

    def deimos_callback(self, event : TimerEvent) -> None: 
        easting = self.starting_easting_ + random.uniform(-self.easting_radius_, self.easting_radius_)
        northing = self.starting_northing_ + random.uniform(-self.northing_radius_, self.northing_radius_)

        lat, lon = utm.to_latlon(easting, northing, self.zone_int_, self.zone_str_)

        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.header.frame_id = "deimos"

        self.deimos_pub_.publish(msg)

    def oberon_callback(self, event : TimerEvent) -> None:
        easting = self.starting_easting_ + random.uniform(-self.easting_radius_, self.easting_radius_)
        northing = self.starting_northing_ + random.uniform(-self.northing_radius_, self.northing_radius_)

        lat, lon = utm.to_latlon(easting, northing, self.zone_int_, self.zone_str_)

        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.header.frame_id = "oberon"

        self.oberon_pub_.publish(msg)
    
    def titania_callback(self, event : TimerEvent) -> None:
        easting = self.starting_easting_ + random.uniform(-self.easting_radius_, self.easting_radius_)
        northing = self.starting_northing_ + random.uniform(-self.northing_radius_, self.northing_radius_)

        lat, lon = utm.to_latlon(easting, northing, self.zone_int_, self.zone_str_)

        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.header.frame_id = "titania"

        self.titania_pub_.publish(msg)
    
    def dione_callback(self, event : TimerEvent) -> None: 
        easting = self.starting_easting_ + random.uniform(-self.easting_radius_, self.easting_radius_)
        northing = self.starting_northing_ + random.uniform(-self.northing_radius_, self.northing_radius_)

        lat, lon = utm.to_latlon(easting, northing, self.zone_int_, self.zone_str_)

        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.header.frame_id = "dione"

        self.dione_pub_.publish(msg)

    def casualty_callback(self, event : TimerEvent) -> None: 
        if self.casualty_ < NUM_CASUALTIES:
            print("[SPOOFER] Spoofing Casualty ID: ", self.casualty_)
            easting = self.starting_easting_ + random.uniform(-self.easting_radius_, self.easting_radius_)
            northing = self.starting_northing_ + random.uniform(-self.northing_radius_, self.northing_radius_)

            lat, lon = utm.to_latlon(easting, northing, self.zone_int_, self.zone_str_)

            msg = NavSatFix()
            msg.latitude = lat
            msg.longitude = lon
            msg.header.seq = self.casualty_
            msg.header.frame_id = f"casualty {self.casualty_}"
        
            if not self.use_array_:
                self.cas_pub_.publish(msg)
            else:
                self.casualties_.append(msg)
                msg_array = NavSatFixArray()
                msg_array.coordinates = self.casualties_
                self.cas_pub_.publish(msg_array)

            self.casualty_ += 1


if __name__ == "__main__":
    rospy.init_node("gps_spoofer")
    
    node = Spoofer()

    rospy.spin()
