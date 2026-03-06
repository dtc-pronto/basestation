#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import json

from dtc_msgs.msg import CasualtyFixArray, CasualtyFix
from helpers import gps_distance
from submission import location_report


UAV = ['dione']
UGV = ['deimos', 'phobos', 'titania', 'oberon']


class CasualtyLocationSub(Node):
    def __init__(self):
        super().__init__('gate1_node')

        self.declare_parameter('robot_names', ['dione', 'deimos', 'phobos', 'titania', 'oberon'])
        self.declare_parameter('gps_threshold', 5.0)

        self.robots = self.get_parameter('robot_names').value
        self.threshold = self.get_parameter('gps_threshold').value
        self.get_logger().info(f"GPS match threshold: {self.threshold}m")

        # UAV detections: list of {casualty_id, lat, lon}
        self.uav_detections = []
        self.mutex = threading.Lock()

        self.subscriptions_list = []
        for robot in self.robots:
            if robot in UAV:
                self.subscriptions_list.append(self.create_subscription(
                    CasualtyFixArray,
                    f"/{robot}/casualty_info",
                    lambda msg, r=robot: self.uav_callback(msg, r),
                    10))
                self.get_logger().info(f"Subscribed to /{robot}/casualty_info (UAV)")
            elif robot in UGV:
                self.subscriptions_list.append(self.create_subscription(
                    CasualtyFix,
                    f"/{robot}/casualty_report",
                    lambda msg, r=robot: self.ugv_callback(msg, r),
                    10))
                self.get_logger().info(f"Subscribed to /{robot}/casualty_report (UGV)")

    def uav_callback(self, msg: CasualtyFixArray, robot: str):
        with self.mutex:
            self.uav_detections = [
                {
                    "casualty_id": c.casualty_id,
                    "lat": c.location.latitude,
                    "lon": c.location.longitude,
                }
                for c in msg.casualties
            ]
        self.get_logger().info(f"[{robot}] Updated matching table: {len(self.uav_detections)} casualties")

    def _match_or_create(self, lat, lon):
        """
        GPS-match against UAV detections. If no match within threshold, create a new
        entry with the next available ID so future reports near the same spot reuse it.
        Returns (casualty_id, dist, is_new).
        """
        with self.mutex:
            min_dist = float('inf')
            matched = None
            for det in self.uav_detections:
                try:
                    d = gps_distance(lat, lon, det['lat'], det['lon'])
                except ValueError:
                    continue
                if d < min_dist:
                    min_dist = d
                    matched = det

            if matched is not None and min_dist <= self.threshold:
                return matched['casualty_id'], min_dist, False

            # No match — assign a new ID and record it
            new_id = len(self.uav_detections)
            self.uav_detections.append({"casualty_id": new_id, "lat": lat, "lon": lon})
            return new_id, min_dist, True

    def ugv_callback(self, msg: CasualtyFix, robot: str):
        lat = msg.location.latitude
        lon = msg.location.longitude

        casualty_id, dist, is_new = self._match_or_create(lat, lon)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] No UAV match within {self.threshold}m "
                f"(closest: {dist:.1f}m) — creating new casualty ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Matched casualty {casualty_id} at {dist:.1f}m, submitting location"
            )

        r = location_report(lat=lat, lon=lon, level=1, id=casualty_id, system=robot)
        if r.status_code != 200:
            self.get_logger().error(
                f"[{robot}] location_report failed: {r.status_code} {r.text}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CasualtyLocationSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
