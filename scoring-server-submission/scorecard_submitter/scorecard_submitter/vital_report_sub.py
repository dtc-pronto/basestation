#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json

from dtc_msgs.msg import HeartRate, RespirationRate
from helpers import gps_distance
from submission import vitals_report


UGV = ['deimos', 'phobos', 'titania', 'oberon']


class VitalsReportNode(Node):
    def __init__(self):
        super().__init__('gate4_node')

        self.declare_parameter('robot_names', ['deimos', 'phobos', 'titania', 'oberon'])
        self.declare_parameter('casualty_id_path', '')
        self.declare_parameter('gps_threshold', 5.0)

        self.robots = self.get_parameter('robot_names').value
        self.gps_threshold = self.get_parameter('gps_threshold').value

        casualty_id_path = self.get_parameter('casualty_id_path').value
        self.casualty_ids = []
        if casualty_id_path:
            with open(casualty_id_path, 'r') as f:
                self.casualty_ids = json.load(f)
            self.get_logger().info(
                f"Loaded {len(self.casualty_ids)} casualty IDs from {casualty_id_path}"
            )
        else:
            self.get_logger().warn("No casualty_id_path set — GPS matching will always fail")

        self.subscriptions_list = []
        for robot in self.robots:
            if robot in UGV:
                self.subscriptions_list.append(self.create_subscription(
                    HeartRate,
                    f"/{robot}/heart_rate",
                    lambda msg, r=robot: self.hr_callback(msg, r),
                    10))
                self.subscriptions_list.append(self.create_subscription(
                    RespirationRate,
                    f"/{robot}/respiration_rate",
                    lambda msg, r=robot: self.rr_callback(msg, r),
                    10))
                self.get_logger().info(
                    f"Subscribed to /{robot}/heart_rate and /{robot}/respiration_rate"
                )

    def _resolve_casualty_id(self, lat, lon):
        min_dist = float('inf')
        matched_id = None
        for entry in self.casualty_ids:
            try:
                d = gps_distance(lat, lon, entry['lat'], entry['lon'])
            except ValueError:
                continue
            if d < min_dist:
                min_dist = d
                matched_id = entry['casualty_id']
        if matched_id is None or min_dist > self.gps_threshold:
            return None, min_dist
        return matched_id, min_dist

    def _submit_vitals(self, lat: float, lon: float, vital_type: str, rate: float,
                       time_ago_stamp, robot: str):
        casualty_id, dist = self._resolve_casualty_id(lat, lon)
        if casualty_id is None:
            self.get_logger().warn(
                f"[{robot}] No casualty match within {self.gps_threshold}m "
                f"(closest: {dist:.1f}m) — dropping {vital_type} report"
            )
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        measurement_sec = time_ago_stamp.sec + time_ago_stamp.nanosec / 1e9
        time_ago = max(now_sec - measurement_sec, 0.0)

        self.get_logger().info(
            f"[{robot}] Matched casualty {casualty_id} at {dist:.1f}m, "
            f"{vital_type}={rate:.2f} time_ago={time_ago:.1f}s"
        )
        r = vitals_report(type=vital_type, value=rate, time_ago=time_ago,
                          id=casualty_id, system=robot)
        if r.status_code != 200:
            self.get_logger().error(
                f"[{robot}] vitals_report ({vital_type}) failed: {r.status_code} {r.text}"
            )

    def hr_callback(self, msg: HeartRate, robot: str):
        self._submit_vitals(
            lat=msg.location.location.latitude,
            lon=msg.location.location.longitude,
            vital_type='hr',
            rate=msg.rate.data,
            time_ago_stamp=msg.location.time_ago,
            robot=robot
        )

    def rr_callback(self, msg: RespirationRate, robot: str):
        self._submit_vitals(
            lat=msg.location.location.latitude,
            lon=msg.location.location.longitude,
            vital_type='rr',
            rate=msg.rate.data,
            time_ago_stamp=msg.location.time_ago,
            robot=robot
        )


def main(args=None):
    rclpy.init(args=args)
    node = VitalsReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
