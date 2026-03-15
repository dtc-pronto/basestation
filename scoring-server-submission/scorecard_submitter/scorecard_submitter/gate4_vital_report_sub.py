#!/usr/bin/env python3
import csv
import json
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from dtc_msgs.msg import Gate1, Gate4
from helpers import gps_distance
from submission import vitals_report, submit_image


UGV = ['deimos', 'phobos', 'titania', 'oberon']


class VitalsReportNode(Node):
    def __init__(self):
        super().__init__('gate4_node')

        self.declare_parameter(
            'robot_names',
            ['deimos', 'phobos', 'titania', 'oberon']
        )
        self.declare_parameter(
            'ground_truth_gps_database_path',
            '/home/dtc/data/casualty_gt_db_gate4.csv'
        )
        self.declare_parameter(
            'submitted_ids_path',
            '/tmp/gate4_submitted_ids.json'
        )
        self.declare_parameter(
            'image_cache_path',
            '/tmp/gate4_casualty_images'
        )
        self.declare_parameter('gps_threshold', 5.0)

        self.robots = self.get_parameter('robot_names').value
        self.gps_threshold = float(self.get_parameter('gps_threshold').value)
        self.ground_truth_path = self.get_parameter('ground_truth_gps_database_path').value
        self.submitted_ids_path = self.get_parameter('submitted_ids_path').value
        self.image_cache_path = self.get_parameter('image_cache_path').value

        os.makedirs(self.image_cache_path, exist_ok=True)

        self.current_gps = {
            robot: {"latitude": None, "longitude": None}
            for robot in self.robots
        }

        self.ground_truth_database = []
        self.submitted_ids = set()

        self._load_ground_truth_database()
        self._load_submitted_ids()

        self.subscriptions_list = []
        for robot in self.robots:
            if robot in UGV:
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate1,
                        f"/{robot}/triage_report/gate1",
                        lambda msg, r=robot: self.gps_callback(msg, r),
                        10
                    )
                )
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate4,
                        f"/{robot}/triage_report/gate4",
                        lambda msg, r=robot: self.gate4_callback(msg, r),
                        10
                    )
                )
                self.get_logger().info(
                    f"Subscribed to /{robot}/triage_report/gate1 and /{robot}/triage_report/gate4"
                )

    def _load_ground_truth_database(self):
        self.ground_truth_database = []

        if not self.ground_truth_path:
            self.get_logger().error("No ground_truth_gps_database_path set")
            return

        if not os.path.exists(self.ground_truth_path):
            self.get_logger().error(
                f"Ground truth DB not found: {self.ground_truth_path}"
            )
            return

        try:
            with open(self.ground_truth_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    casualty_id = row.get('casualty_id')
                    longitude = row.get('longitude')
                    latitude = row.get('latitude')

                    if casualty_id is None or longitude is None or latitude is None:
                        continue

                    self.ground_truth_database.append({
                        "casualty_id": int(casualty_id),
                        "lon": float(longitude),
                        "lat": float(latitude),
                    })

            self.get_logger().info(
                f"Loaded {len(self.ground_truth_database)} ground-truth casualties "
                f"from {self.ground_truth_path}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to load ground-truth DB from {self.ground_truth_path}: {e}"
            )

    def _load_submitted_ids(self):
        self.submitted_ids = set()

        if not self.submitted_ids_path or not os.path.exists(self.submitted_ids_path):
            return

        try:
            with open(self.submitted_ids_path, 'r') as f:
                raw = json.load(f)

            if isinstance(raw, list):
                self.submitted_ids = {int(x) for x in raw}

            self.get_logger().info(
                f"Loaded {len(self.submitted_ids)} submitted casualty IDs "
                f"from {self.submitted_ids_path}"
            )
        except Exception as e:
            self.get_logger().warn(
                f"Failed to load submitted IDs from {self.submitted_ids_path}: {e}"
            )

    def _save_submitted_ids(self):
        if not self.submitted_ids_path:
            return

        try:
            with open(self.submitted_ids_path, 'w') as f:
                json.dump(sorted(list(self.submitted_ids)), f, indent=2)
        except Exception as e:
            self.get_logger().error(
                f"Failed to save submitted IDs to {self.submitted_ids_path}: {e}"
            )

    def _resolve_casualty_id(self, lat, lon):
        min_dist = float('inf')
        matched_id = None

        for casualty in self.ground_truth_database:
            cid = casualty["casualty_id"]

            if cid in self.submitted_ids:
                continue

            try:
                dist = gps_distance(lat, lon, casualty["lat"], casualty["lon"])
            except ValueError:
                continue

            if dist < min_dist:
                min_dist = dist
                matched_id = cid

        if matched_id is not None and min_dist <= self.gps_threshold:
            return matched_id, min_dist

        return None, min_dist

    def gps_callback(self, msg: Gate1, robot: str):
        self.current_gps[robot]["latitude"] = float(msg.latitude)
        self.current_gps[robot]["longitude"] = float(msg.longitude)

        self.get_logger().info(
            f"[{robot}] Updated Gate1 GPS: "
            f"lat={self.current_gps[robot]['latitude']:.8f}, "
            f"lon={self.current_gps[robot]['longitude']:.8f}"
        )

    def _submit_vital(self, robot: str, casualty_id: int, vital_type: str, value: float, time_ago: float):
        response = vitals_report(
            type=vital_type,
            value=float(value),
            time_ago=float(time_ago),
            id=casualty_id,
            system=robot
        )

        if response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] vitals_report failed for {vital_type}: "
                f"{response.status_code} {response.text}"
            )
            return False

        self.get_logger().info(
            f"[{robot}] Submitted {vital_type}={value:.2f} for casualty {casualty_id}"
        )
        return True

    def _submit_gate4_image(self, robot: str, casualty_id: int, msg: Gate4, time_ago: float):
        if not msg.image.data:
            self.get_logger().warn(
                f"[{robot}] Gate4 image is empty for casualty {casualty_id}"
            )
            return True

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

        ext = '.jpg'
        if msg.image.format:
            fmt = msg.image.format.lower()
            if 'png' in fmt:
                ext = '.png'
            elif 'jpeg' in fmt or 'jpg' in fmt:
                ext = '.jpg'

        image_path = os.path.join(
            self.image_cache_path,
            f"{robot}_casualty_{casualty_id}_{timestamp}{ext}"
        )

        try:
            with open(image_path, 'wb') as f:
                f.write(msg.image.data)
        except Exception as e:
            self.get_logger().error(
                f"[{robot}] Failed to save Gate4 image for casualty {casualty_id}: {e}"
            )
            return False

        try:
            time_now = self.get_clock().now().to_msg().sec
            response = submit_image(
                image_path=image_path,
                time=time_ago,
                time_now=time_now,
                id=casualty_id
            )
        except Exception as e:
            self.get_logger().error(
                f"[{robot}] submit_image raised exception for casualty {casualty_id}: {e}"
            )
            return False

        if response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] submit_image failed: {response.status_code} {response.text}"
            )
            return False

        self.get_logger().info(
            f"[{robot}] Submitted Gate4 image for casualty {casualty_id}"
        )
        return True

    def gate4_callback(self, msg: Gate4, robot: str):
        lat = self.current_gps[robot]["latitude"]
        lon = self.current_gps[robot]["longitude"]

        if lat is None or lon is None:
            self.get_logger().warn(
                f"[{robot}] Received Gate4 before Gate1 GPS; cannot resolve casualty ID"
            )
            return

        casualty_id, dist = self._resolve_casualty_id(lat, lon)
        if casualty_id is None:
            if dist == float('inf'):
                self.get_logger().warn(
                    f"[{robot}] No unresolved casualty match available"
                )
            else:
                self.get_logger().warn(
                    f"[{robot}] No unresolved casualty match within {self.gps_threshold}m "
                    f"(closest unresolved: {dist:.1f}m)"
                )
            return

        time_ago = 0.0

        self.get_logger().info(
            f"[{robot}] Matched casualty {casualty_id} at {dist:.1f}m, "
            f"submitting Gate4 block hr={msg.hr:.2f}, rr={msg.rr:.2f}"
        )

        all_ok = True

        ok = self._submit_vital(
            robot=robot,
            casualty_id=casualty_id,
            vital_type='rr',
            value=msg.rr,
            time_ago=time_ago
        )
        if not ok:
            all_ok = False

        ok = self._submit_vital(
            robot=robot,
            casualty_id=casualty_id,
            vital_type='hr',
            value=msg.hr,
            time_ago=time_ago
        )
        if not ok:
            all_ok = False

        ok = self._submit_gate4_image(
            robot=robot,
            casualty_id=casualty_id,
            msg=msg,
            time_ago=time_ago
        )
        if not ok:
            all_ok = False

        if not all_ok:
            self.get_logger().error(
                f"[{robot}] One or more Gate4 submissions failed for casualty {casualty_id}"
            )
            return

        self.submitted_ids.add(casualty_id)
        self._save_submitted_ids()

        self.get_logger().info(
            f"[{robot}] Completed Gate4 submissions for casualty {casualty_id}. "
            f"This casualty is now marked as submitted and will be ignored in future matches."
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