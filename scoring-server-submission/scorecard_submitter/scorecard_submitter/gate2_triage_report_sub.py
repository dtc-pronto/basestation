#!/usr/bin/env python3
import csv
import json
import os

import rclpy
from rclpy.node import Node

from dtc_msgs.msg import Gate1, Gate2
from helpers import gps_distance
from submission import triage_report, submit_image


UGV = ['deimos', 'phobos', 'titania', 'oberon', 'ares', 'aphrodite']


class TriageReportSub(Node):
    def __init__(self):
        super().__init__('gate2_node')

        self.declare_parameter(
            'robot_names',
            ['deimos', 'phobos', 'titania', 'oberon', 'ares', 'aphrodite']
        )
        self.declare_parameter(
            'ground_truth_gps_database_path',
            '/home/dtc/data/casualty_gt_db_gate2.csv'
        )
        self.declare_parameter(
            'submitted_ids_path',
            '/tmp/gate2_submitted_ids.json'
        )
        self.declare_parameter('gps_threshold', 5.0)
        self.declare_parameter('image_save_dir', '/home/dtc/data/triage_images')

        self.robots = self.get_parameter('robot_names').value
        self.gps_threshold = float(self.get_parameter('gps_threshold').value)
        self.ground_truth_path = self.get_parameter('ground_truth_gps_database_path').value
        self.submitted_ids_path = self.get_parameter('submitted_ids_path').value
        self.image_save_dir = self.get_parameter('image_save_dir').value

        os.makedirs(self.image_save_dir, exist_ok=True)

        # Latest Gate1 GPS per robot
        self.current_gps = {
            robot: {"latitude": None, "longitude": None}
            for robot in self.robots
        }

        # Ground truth entries loaded from CSV
        self.ground_truth_database = []

        # Already-submitted casualty IDs
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
                        Gate2,
                        f"/{robot}/triage_report/gate2",
                        lambda msg, r=robot: self.triage_callback(msg, r),
                        10
                    )
                )
                self.get_logger().info(
                    f"Subscribed to /{robot}/triage_report/gate1 and /{robot}/triage_report/gate2"
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

        if not self.submitted_ids_path:
            return

        if not os.path.exists(self.submitted_ids_path):
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
        """
        Match current GPS to the nearest unresolved ground-truth casualty.
        Any casualty_id already in submitted_ids is ignored.
        """
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

    def save_compressed_image(self, msg: Gate2, casualty_id: int, robot: str) -> str | None:
        try:
            if not msg.image.data:
                self.get_logger().warn(f"[{robot}] No compressed image data for casualty {casualty_id}")
                return None

            filename = f"{casualty_id}_{robot}_gate2.jpg"
            img_path = os.path.join(self.image_save_dir, filename)

            with open(img_path, "wb") as f:
                f.write(bytes(msg.image.data))

            self.get_logger().info(f"[{robot}] Saved Gate2 image to {img_path}")
            return img_path

        except Exception as e:
            self.get_logger().error(
                f"[{robot}] Failed to save image for casualty {casualty_id}: {e}"
            )
            return None

    def gps_callback(self, msg: Gate1, robot: str):
        self.current_gps[robot]["latitude"] = float(msg.latitude)
        self.current_gps[robot]["longitude"] = float(msg.longitude)

        self.get_logger().info(
            f"[{robot}] Updated Gate1 GPS: "
            f"lat={self.current_gps[robot]['latitude']:.8f}, "
            f"lon={self.current_gps[robot]['longitude']:.8f}"
        )

    def triage_callback(self, msg: Gate2, robot: str):
        lat = self.current_gps[robot]["latitude"]
        lon = self.current_gps[robot]["longitude"]

        if lat is None or lon is None:
            self.get_logger().warn(
                f"[{robot}] Received Gate2 before Gate1 GPS; cannot resolve casualty ID"
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
                    f"[{robot}] No unresolved casualty match within {self.gps_threshold} m "
                    f"(closest unresolved: {dist:.2f} m)"
                )
            return

        self.get_logger().info(
            f"[{robot}] Matched unresolved casualty {casualty_id} at {dist:.2f} m, submitting triage"
        )

        try:
            response = triage_report(
                category=msg.category,
                id=casualty_id,
                system=robot
            )
        except Exception as e:
            self.get_logger().error(
                f"[{robot}] triage_report exception for casualty {casualty_id}: {e}"
            )
            return

        if response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] triage_report failed: {response.status_code} {response.text}"
            )
            return

        self.get_logger().info(
            f"[{robot}] Submitted triage for casualty {casualty_id}"
        )

        img_path = self.save_compressed_image(msg, casualty_id, robot)
        if img_path is None:
            self.get_logger().error(
                f"[{robot}] Image save failed for casualty {casualty_id}; not marking as resolved"
            )
            return

        try:
            image_response = submit_image(
                image_path=img_path,
                time=0,
                time_now=0,
                id=casualty_id
            )
        except Exception as e:
            self.get_logger().error(
                f"[{robot}] submit_image exception for casualty {casualty_id}: {e}"
            )
            return

        if image_response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] submit_image failed: {image_response.status_code} {image_response.text}"
            )
            return

        self.get_logger().info(
            f"[{robot}] Submitted image for casualty {casualty_id}"
        )

        # Mark as consumed only after both triage + image submission succeed
        self.submitted_ids.add(casualty_id)
        self._save_submitted_ids()

        self.get_logger().info(
            f"[{robot}] Casualty {casualty_id} fully submitted and now marked as resolved"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TriageReportSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()