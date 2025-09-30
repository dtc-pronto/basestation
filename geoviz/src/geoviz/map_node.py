"""
    Jason Hughes
    March 2025

    Plot things on a map
"""
import utm
import json
import rospy
from sensor_msgs.msg import NavSatFix
from dtc_msgs.msg import CasualtyFix, JackalStatus, FalconStatus
from basestation_msgs.msg import ServerReport, InitialReport, FullReport
from std_msgs.msg import Int32

from geoviz.map_app import MapApp

class MapNode:

    def __init__(self) -> None:

        ip = rospy.get_param("/geoviz/app/ip_address")
        port = rospy.get_param("/geoviz/app/port")
        use_array = rospy.get_param("geoviz/app/use_array", False)

        path = rospy.get_param("/geoviz/path")

        self.zone_num_ = rospy.get_param("/geoviz/map/zone_number")
        self.zone_id_ = rospy.get_param("/geoviz/map/zone_id")
        tile_path = rospy.get_param("~tile_path")
        rospy.loginfo("Using tiles at: %s" %tile_path)
        self.app_ = MapApp(path, ip = ip, port = port, tile_path = tile_path)
        self.app_.run_in_thread()

        robots = rospy.get_param("/robots")

        if "phobos" in robots:
            rospy.Subscriber("/phobos/ublox/fix", NavSatFix, self.phobos_callback)
            rospy.Subscriber("/phobos/sensor_status", JackalStatus, self.phobos_health_callback)
            rospy.Subscriber("/ddb/rajant/rssi/phobos", Int32, self.phobos_rssi_callback)
        if "deimos" in robots:
            rospy.Subscriber("/deimos/ublox/fix", NavSatFix, self.deimos_callback)
            rospy.Subscriber("/deimos/sensor_status", JackalStatus, self.deimos_health_callback)
            rospy.Subscriber("/ddb/rajant/rssi/deimos", Int32, self.deimos_rssi_callback)
        if "oberon" in robots:
            rospy.Subscriber("/oberon/ublox/fix", NavSatFix, self.oberon_callback)
            rospy.Subscriber("/oberon/sensor_status", JackalStatus, self.oberon_health_callback)
            rospy.Subscriber("/ddb/rajant/rssi/oberon", Int32, self.oberon_rssi_callback)
        if "titania" in robots:
            rospy.Subscriber("/titania/ublox/fix", NavSatFix, self.titania_callback)
            rospy.Subscriber("/titania/sensor_status", JackalStatus, self.titania_health_callback)
            rospy.Subscriber("/ddb/rajant/rssi/titania", Int32, self.titania_rssi_callback)
        if "dione" in robots:
            rospy.Subscriber("/dione/mavros/global_position/raw/fix", NavSatFix, self.dione_callback)
            rospy.Subscriber("/dione/sensor_status", FalconStatus, self.dione_health_callback)
            rospy.Subscriber("/ddb/rajant/rssi/dione", Int32, self.dione_rssi_callback)

        rospy.Subscriber("/basestation/scoring_server_report", ServerReport, self.server_callback)
        rospy.Subscriber("/basestation/initial_report", InitialReport, self.initial_report_callback)
        rospy.Subscriber("/basestation/full_report", FullReport, self.full_report_callback)

        print("[VISUALIZER] starting visualizer with robots: ", robots)

    def phobos_callback(self, msg : NavSatFix) -> None:
        self.app_.update_jackal(msg.latitude, msg.longitude, "phobos") 
    
    def phobos_health_callback(self, msg : JackalStatus) -> None:
        status = {"robot_name": "phobos", "rgb": msg.rgb, "thermal": msg.thermal, "rtk":msg.rtk, "jeti":msg.jeti, "ouster":msg.ouster, "mic":msg.mic}
        self.app_.update_status(status)

    def deimos_callback(self, msg : NavSatFix) -> None:
        self.app_.update_jackal(msg.latitude, msg.longitude, "deimos")
    
    def deimos_health_callback(self, msg : JackalStatus) -> None:
        status = {"robot_name": "deimos", "rgb": msg.rgb, "thermal": msg.thermal, "rtk":msg.rtk, "jeti":msg.jeti, "ouster":msg.ouster, "mic":msg.mic}
        self.app_.update_status(status)

    def oberon_callback(self, msg : NavSatFix) -> None:
        self.app_.update_jackal(msg.latitude, msg.longitude, "oberon")
    
    def oberon_health_callback(self, msg : JackalStatus) -> None:
        status = {"robot_name": "oberon", "rgb": msg.rgb, "thermal": msg.thermal, "rtk":msg.rtk, "jeti":msg.jeti, "ouster":msg.ouster, "mic":msg.mic}
        self.app_.update_status(status)

    def titania_callback(self, msg : NavSatFix) -> None: 
        self.app_.update_jackal(msg.latitude, msg.longitude, "titania")
    
    def titania_health_callback(self, msg : JackalStatus) -> None:
        status = {"robot_name": "titania", "rgb": msg.rgb, "thermal": msg.thermal, "rtk":msg.rtk, "jeti":msg.jeti, "ouster":msg.ouster, "mic":msg.mic}
        self.app_.update_status(status)

    def dione_callback(self, msg : NavSatFix) -> None:
        self.app_.update_falcon(msg.latitude, msg.longitude, "dione")

    def dione_health_callback(self, msg : FalconStatus) -> None:
        status = {"robot_name": "dione", "rgb": msg.rgb, "thermal": msg.thermal, "gps":msg.gps, "rtk":msg.rtk}
        self.app_.update_status(status)

    def server_callback(self, msg : ServerReport) -> None: 
        data = {"code": msg.code, "message": msg.report.data, "robot_name": msg.robot.data}
        self.app_.update_server_report(data)

    def initial_report_callback(self, msg : InitialReport) -> None:
        self.app_.update_casualty(msg.latitude, msg.longitude, msg.casualty_id)

    def full_report_callback(self, msg : FullReport) -> None:
        cid = msg.casualty_id
        report = json.loads(msg.report.data)
        lat = report["location"]["latitude"]
        lon = report["location"]["longitude"]
        self.app_.update_casualty(lat, lon, cid)

    def phobos_rssi_callback(self, msg : Int32):
        rssi = msg.data
        self.app_.update_rssi({"robot_name": "phobos", "rssi": rssi})

    def deimos_rssi_callback(self, msg : Int32):
        rssi = msg.data
        self.app_.update_rssi({"robot_name": "deimos", "rssi": rssi})

    def titania_rssi_callback(self, msg : Int32):
        rssi = msg.data
        self.app_.update_rssi({"robot_name": "titania", "rssi": rssi})

    def oberon_rssi_callback(self, msg : Int32):
        rssi = msg.data
        self.app_.update_rssi({"robot_name": "oberon", "rssi": rssi})

    def dione_rssi_callback(self, msg : Int32):
        rssi = msg.data
        self.app_.update_rssi({"robot_name": "dione", "rssi": rssi})
