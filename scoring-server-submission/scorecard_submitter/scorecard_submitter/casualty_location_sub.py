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
        self.declare_parameter('threshold_path', '')

        self.robots = self.get_parameter('robot_names').value

        self.threshold = 5.0
        threshold_path = self.get_parameter('threshold_path').value
        if threshold_path:
            with open(threshold_path, 'r') as f:
                config = json.load(f)
            self.threshold = config.get('ugv_uav_distance_threshold', 5.0)
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

        # Submit location for each UAV detection directly
        for det in self.uav_detections:
            r = location_report(
                lat=det['lat'],
                lon=det['lon'],
                level=1,
                id=det['casualty_id'],
                system=robot
            )
            if r.status_code != 200:
                self.get_logger().error(
                    f"[{robot}] location_report failed for casualty {det['casualty_id']}: "
                    f"{r.status_code} {r.text}"
                )

    def ugv_callback(self, msg: CasualtyFix, robot: str):
        lat = msg.location.latitude
        lon = msg.location.longitude

        with self.mutex:
            detections = list(self.uav_detections)

        if not detections:
            self.get_logger().warn(f"[{robot}] No UAV detections yet, cannot match casualty")
            return

        min_dist = float('inf')
        matched = None
        for det in detections:
            try:
                d = gps_distance(lat, lon, det['lat'], det['lon'])
            except ValueError:
                continue
            if d < min_dist:
                min_dist = d
                matched = det

        if matched is None or min_dist > self.threshold:
            self.get_logger().warn(
                f"[{robot}] No UAV match within {self.threshold}m "
                f"(closest: {min_dist:.1f}m)"
            )
            return

        self.get_logger().info(
            f"[{robot}] Matched casualty {matched['casualty_id']} at {min_dist:.1f}m, submitting location"
        )
        r = location_report(lat=lat, lon=lon, level=1, id=matched['casualty_id'], system=robot)
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
