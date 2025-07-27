"""
    Jason Hughes
    March 2025

    Plot things on a map
"""
import utm
import rospy
from sensor_msgs.msg import NavSatFix
from dtc_msgs.msg import NavSatFixArray

from geoviz.map_app import MapApp

class MapNode:

    def __init__(self) -> None:

        ip = rospy.get_param("/geoviz/app/ip_address")
        port = rospy.get_param("/geoviz/app/port")
        use_array = rospy.get_param("geoviz/app/use_array", False)

        path = rospy.get_param("/geoviz/path")

        self.zone_num_ = rospy.get_param("/geoviz/map/zone_number")
        self.zone_id_ = rospy.get_param("/geoviz/map/zone_id")

        self.app_ = MapApp(path, ip = ip, port = port)
        self.app_.run_in_thread()

        robots = rospy.get_param("/geoviz/app/robots")

        if "phobos" in robots:
            rospy.Subscriber("/phobos/ublox/fix", NavSatFix, self.phobos_callback)
        if "deimos" in robots:
            rospy.Subscriber("/deimos/ublox/fix", NavSatFix, self.deimos_callback)
        if "oberon" in robots:
            rospy.Subscriber("/oberon/ublox/fix", NavSatFix, self.oberon_callback)
        if "titania" in robots:
            rospy.Subscriber("/titania/ublox/fix", NavSatFix, self.titania_callback)
        if "dione" in robots:
            rospy.Subscriber("/dione/mavros/global_position/fix/raw", NavSatFix, self.dione_callback)

        print("[VISUALIZER] Using NavSatFixArray: ", use_array)
        if not use_array:
            rospy.Subscriber("/dione/casualty", NavSatFix, self.casualty_callback)
        else:
            rospy.Subscriber("dione/casualty", NavSatFixArray, self.casualty_array_callback)
        
        print("[VISUALIZER] starting visualizer with robots: ", robots)

    def phobos_callback(self, msg : NavSatFix) -> None:
        self.app_.update_jackal(msg.latitude, msg.longitude, "phobos") 

    def deimos_callback(self, msg : NavSatFix) -> None:
        self.app_.update_jackal(msg.latitude, msg.longitude, "deimos")

    def oberon_callback(self, msg : NavSatFix) -> None:
        self.app_.update_jackal(msg.latitude, msg.longitude, "oberon")

    def titania_callback(self, msg : NavSatFix) -> None: 
        self.app_.update_jackal(msg.latitude, msg.longitude, "titania")

    def dione_callback(self, msg : NavSatFix) -> None:
        self.app_.update_falcon(msg.latitude, msg.longitude, "dione")

    def casualty_callback(self, msg : NavSatFix) -> None: 
        self.app_.update_casualty(msg.latitude, msg.longitude, msg.header.seq)

    def casualty_array_callback(self, msg : NavSatFixArray) -> None:
        for coord in msg.coordinates:
            self.app_.update_casualty(coord.latitude, coord.longitude, coord.header.seq)
