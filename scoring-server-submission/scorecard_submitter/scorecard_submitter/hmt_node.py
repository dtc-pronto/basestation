#!/usr/bin/env python3
import csv
import json
import os
from copy import deepcopy

import rclpy
from rclpy.node import Node

from dtc_msgs.msg import Gate1, Gate2, Gate3
from helpers import gps_distance
from submission import hmt_location_report, hmt_assessment_report


UGV = ['deimos', 'phobos', 'titania', 'oberon']


class HMTNode(Node):
    def __init__(self):
        super().__init__('hmt_node')

        self.declare_parameter(
            'robot_names',
            ['deimos', 'phobos', 'titania', 'oberon']
        )
        self.declare_parameter(
            'ground_truth_gps_database_path',
            '/home/dtc/data/casualty_gt_db.csv'
        )
        self.declare_parameter('gps_threshold', 5.0)
        self.declare_parameter('hmt_db_path', '/tmp/hmt_detected_people.json')
        self.declare_parameter('hmt_submitted_path', '/tmp/hmt_submitted_people.json')

        self.robots = self.get_parameter('robot_names').value
        self.gps_threshold = float(self.get_parameter('gps_threshold').value)
        self.ground_truth_path = self.get_parameter('ground_truth_gps_database_path').value
        self.hmt_db_path = self.get_parameter('hmt_db_path').value
        self.hmt_submitted_path = self.get_parameter('hmt_submitted_path').value

        # Latest Gate1 GPS per robot
        self.current_gps = {
            robot: {"latitude": None, "longitude": None}
            for robot in self.robots
        }

        # Ground-truth casualty DB
        self.ground_truth_database = []

        # HMT detected people DB
        # keyed by casualty_id string for easy JSON persistence
        self.detected_people = {}

        # HMT submitted DB
        # {
        #   "1": {
        #       "casualty_submitted": true,
        #       "assessment_submitted": true
        #   }
        # }
        self.submitted_people = {}

        self._load_ground_truth_database()
        self._load_detected_people()
        self._load_submitted_people()

        self.subscriptions_list = []
        for robot in self.robots:
            if robot in UGV:
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate1,
                        f"/{robot}/triage_report/gate1",
                        lambda msg, r=robot: self.gate1_callback(msg, r),
                        10
                    )
                )
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate2,
                        f"/{robot}/triage_report/gate2",
                        lambda msg, r=robot: self.gate2_callback(msg, r),
                        10
                    )
                )
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate3,
                        f"/{robot}/triage_report/gate3",
                        lambda msg, r=robot: self.gate3_callback(msg, r),
                        10
                    )
                )
                self.get_logger().info(
                    f"Subscribed to /{robot}/triage_report/gate1, gate2, gate3"
                )

    # -------------------------------------------------
    # Load / save helpers
    # -------------------------------------------------

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

    def _load_detected_people(self):
        self.detected_people = {}
        if not self.hmt_db_path or not os.path.exists(self.hmt_db_path):
            return
        try:
            with open(self.hmt_db_path, 'r') as f:
                raw = json.load(f)
            if isinstance(raw, dict):
                self.detected_people = raw
        except Exception as e:
            self.get_logger().warn(f"Failed to load HMT DB {self.hmt_db_path}: {e}")

    def _save_detected_people(self):
        try:
            with open(self.hmt_db_path, 'w') as f:
                json.dump(self.detected_people, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed to save HMT DB {self.hmt_db_path}: {e}")

    def _load_submitted_people(self):
        self.submitted_people = {}
        if not self.hmt_submitted_path or not os.path.exists(self.hmt_submitted_path):
            return
        try:
            with open(self.hmt_submitted_path, 'r') as f:
                raw = json.load(f)
            if isinstance(raw, dict):
                self.submitted_people = raw
        except Exception as e:
            self.get_logger().warn(
                f"Failed to load HMT submitted DB {self.hmt_submitted_path}: {e}"
            )

    def _save_submitted_people(self):
        try:
            with open(self.hmt_submitted_path, 'w') as f:
                json.dump(self.submitted_people, f, indent=2)
        except Exception as e:
            self.get_logger().error(
                f"Failed to save HMT submitted DB {self.hmt_submitted_path}: {e}"
            )

    # -------------------------------------------------
    # Matching / state helpers
    # -------------------------------------------------

    def _resolve_casualty_id(self, lat, lon):
        min_dist = float('inf')
        matched_id = None

        for casualty in self.ground_truth_database:
            try:
                dist = gps_distance(lat, lon, casualty["lat"], casualty["lon"])
            except ValueError:
                continue

            if dist < min_dist:
                min_dist = dist
                matched_id = casualty["casualty_id"]

        if matched_id is not None and min_dist <= self.gps_threshold:
            return matched_id, min_dist

        return None, min_dist

    def _ensure_detected_entry(self, casualty_id: int):
        cid = str(casualty_id)

        if cid not in self.detected_people:
            self.detected_people[cid] = {
                "casualty_id": casualty_id,
                "latitude": None,
                "longitude": None,
                "robot": None,
                "category": None,
                "assessment": {},
                "casualty_submitted": False,
                "assessment_submitted": False,
            }

        if cid not in self.submitted_people:
            self.submitted_people[cid] = {
                "casualty_submitted": False,
                "assessment_submitted": False,
            }

        return cid

    def _mark_casualty_submitted(self, cid: str):
        self.detected_people[cid]["casualty_submitted"] = True
        self.submitted_people[cid]["casualty_submitted"] = True
        self._save_detected_people()
        self._save_submitted_people()

    def _mark_assessment_submitted(self, cid: str):
        self.detected_people[cid]["assessment_submitted"] = True
        self.submitted_people[cid]["assessment_submitted"] = True
        self._save_detected_people()
        self._save_submitted_people()

    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------

    def gate1_callback(self, msg: Gate1, robot: str):
        self.current_gps[robot]["latitude"] = float(msg.latitude)
        self.current_gps[robot]["longitude"] = float(msg.longitude)

        self.get_logger().info(
            f"[{robot}] Updated Gate1 GPS: "
            f"lat={msg.latitude:.8f}, lon={msg.longitude:.8f}"
        )

    def gate2_callback(self, msg: Gate2, robot: str):
        lat = self.current_gps[robot]["latitude"]
        lon = self.current_gps[robot]["longitude"]

        if lat is None or lon is None:
            self.get_logger().warn(
                f"[{robot}] Received Gate2 before Gate1 GPS; cannot submit HMT casualty"
            )
            return

        casualty_id, dist = self._resolve_casualty_id(lat, lon)
        if casualty_id is None:
            if dist == float('inf'):
                self.get_logger().warn(f"[{robot}] No casualty match available for HMT casualty")
            else:
                self.get_logger().warn(
                    f"[{robot}] No casualty match within {self.gps_threshold}m "
                    f"(closest: {dist:.2f}m)"
                )
            return

        cid = self._ensure_detected_entry(casualty_id)
        self.detected_people[cid]["latitude"] = lat
        self.detected_people[cid]["longitude"] = lon
        self.detected_people[cid]["robot"] = robot
        self.detected_people[cid]["category"] = int(msg.category)
        self._save_detected_people()

        if self.submitted_people[cid]["casualty_submitted"]:
            self.get_logger().info(
                f"[{robot}] HMT casualty already submitted for casualty {casualty_id}; ignoring"
            )
            return

        self.get_logger().info(
            f"[{robot}] Matched casualty {casualty_id} at {dist:.2f}m, submitting HMT casualty"
        )

        response = hmt_location_report(
            lat=lat,
            lon=lon,
            category=int(msg.category),
            time_ago=0.0,
            id=casualty_id,
            system=robot
        )

        if response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] hmt/casualty failed: {response.status_code} {response.text}"
            )
            return

        self._mark_casualty_submitted(cid)
        self.get_logger().info(
            f"[{robot}] Submitted HMT casualty for casualty {casualty_id}"
        )

    def gate3_callback(self, msg: Gate3, robot: str):
        lat = self.current_gps[robot]["latitude"]
        lon = self.current_gps[robot]["longitude"]

        if lat is None or lon is None:
            self.get_logger().warn(
                f"[{robot}] Received Gate3 before Gate1 GPS; cannot submit HMT assessment"
            )
            return

        casualty_id, dist = self._resolve_casualty_id(lat, lon)
        if casualty_id is None:
            if dist == float('inf'):
                self.get_logger().warn(f"[{robot}] No casualty match available for HMT assessment")
            else:
                self.get_logger().warn(
                    f"[{robot}] No casualty match within {self.gps_threshold}m "
                    f"(closest: {dist:.2f}m)"
                )
            return

        cid = self._ensure_detected_entry(casualty_id)
        self.detected_people[cid]["latitude"] = lat
        self.detected_people[cid]["longitude"] = lon
        self.detected_people[cid]["robot"] = robot
        self.detected_people[cid]["assessment"] = {
            "trauma_head": int(msg.trauma_head),
            "trauma_torso_back": int(msg.trauma_torso_back),
            "trauma_torso_front": int(msg.trauma_torso_front),
            "trauma_leg_right": int(msg.trauma_leg_right),
            "trauma_leg_left": int(msg.trauma_leg_left),
            "trauma_arm_right": int(msg.trauma_arm_right),
            "trauma_arm_left": int(msg.trauma_arm_left),
            "alertness_ocular": int(msg.alertness_ocular),
            "alertness_verbal": int(msg.alertness_verbal),
            "alertness_motor": int(msg.alertness_motor),
            "second_pass_category": int(msg.second_pass_category),
        }
        self._save_detected_people()

        if self.submitted_people[cid]["assessment_submitted"]:
            self.get_logger().info(
                f"[{robot}] HMT assessment already submitted for casualty {casualty_id}; ignoring"
            )
            return

        self.get_logger().info(
            f"[{robot}] Matched casualty {casualty_id} at {dist:.2f}m, submitting HMT assessment block"
        )

        fields = [
            ("trauma_head", msg.trauma_head),
            ("trauma_torso_back", msg.trauma_torso_back),
            ("trauma_torso_front", msg.trauma_torso_front),
            ("trauma_leg_right", msg.trauma_leg_right),
            ("trauma_leg_left", msg.trauma_leg_left),
            ("trauma_arm_right", msg.trauma_arm_right),
            ("trauma_arm_left", msg.trauma_arm_left),
            ("alertness_ocular", msg.alertness_ocular),
            ("alertness_verbal", msg.alertness_verbal),
            ("alertness_motor", msg.alertness_motor),
            ("second_pass_category", msg.second_pass_category),
        ]

        all_ok = True
        for field_type, value in fields:
            response = hmt_assessment_report(
                type=field_type,
                value=int(value),
                time_ago=0.0,
                id=casualty_id,
                system=robot
            )
            if response.status_code != 200:
                self.get_logger().error(
                    f"[{robot}] hmt/assessment failed for {field_type}: "
                    f"{response.status_code} {response.text}"
                )
                all_ok = False

        if not all_ok:
            return

        self._mark_assessment_submitted(cid)
        self.get_logger().info(
            f"[{robot}] Submitted HMT assessment for casualty {casualty_id}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HMTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()