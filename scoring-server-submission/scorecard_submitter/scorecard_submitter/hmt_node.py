#!/usr/bin/env python3
import os
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node

from dtc_msgs.msg import Gate1, Gate2, Gate3
from helpers import gps_distance
from submission import hmt_location_report, hmt_assessment_report, submit_image


UGV = ['deimos', 'phobos', 'titania', 'oberon', 'ares', 'aphrodite']


class HMTNode(Node):
    def __init__(self):
        super().__init__('hmt_node')

        self.declare_parameter(
            'robot_names',
            ['deimos', 'phobos', 'titania', 'oberon', 'ares', 'aphrodite']
        )
        self.declare_parameter('gps_threshold', 5.0)
        self.declare_parameter('debug_data_path', '/tmp/hmt_debug')
        self.declare_parameter('image_save_dir', '/home/dtc/data/hmt_images')

        self.robots = self.get_parameter('robot_names').value
        self.threshold = float(self.get_parameter('gps_threshold').value)
        debug_data_path = self.get_parameter('debug_data_path').value
        self.image_save_dir = self.get_parameter('image_save_dir').value

        os.makedirs(self.image_save_dir, exist_ok=True)

        self.get_logger().info(f"HMT GPS match threshold: {self.threshold} m")

        self.current_gps = {
            robot: {"latitude": None, "longitude": None}
            for robot in self.robots
        }

        # Detected people DB:
        # {
        #   "0": {
        #       "casualty_id": 0,
        #       "latitude": ...,
        #       "longitude": ...,
        #       "robot": "deimos",
        #       "category": 2,
        #       "assessment": {...},
        #       "casualty_submitted": false,
        #       "assessment_submitted": false
        #   }
        # }
        self.detected_people = {}
        self.mutex = threading.Lock()
        self.subscriptions_list = []

        # Debug JSON paths
        folder_name = f"hmt_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.debug_path = os.path.join(debug_data_path, folder_name)
        os.makedirs(self.debug_path, exist_ok=True)

        self.detected_people_json = os.path.join(self.debug_path, "hmt_detected_people.json")
        self.submitted_people_json = os.path.join(self.debug_path, "hmt_submitted_people.json")
        self.match_log_json = os.path.join(self.debug_path, "hmt_match_log.json")

        self.write_json(self.detected_people_json, {})
        self.write_json(self.submitted_people_json, {})
        self.write_json(self.match_log_json, [])

        self.get_logger().info(f"HMT debug JSON path: {self.debug_path}")

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
    # JSON helpers
    # -------------------------------------------------

    def write_json(self, path, data):
        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed writing {path}: {e}")

    def append_match_log(self, entry):
        try:
            logs = []
            if os.path.exists(self.match_log_json):
                with open(self.match_log_json, "r") as f:
                    try:
                        logs = json.load(f)
                    except json.JSONDecodeError:
                        logs = []

            logs.append(entry)

            with open(self.match_log_json, "w") as f:
                json.dump(logs, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Failed writing HMT match log: {e}")

    def _save_detected_people(self):
        self.write_json(self.detected_people_json, self.detected_people)
        submitted_flags = {
            cid: {
                "casualty_submitted": entry.get("casualty_submitted", False),
                "assessment_submitted": entry.get("assessment_submitted", False),
            }
            for cid, entry in self.detected_people.items()
        }
        self.write_json(self.submitted_people_json, submitted_flags)

    # -------------------------------------------------
    # Matching helpers
    # -------------------------------------------------

    def _next_casualty_id(self) -> int:
        ids = [int(cid) for cid in self.detected_people.keys()]
        return max(ids, default=-1) + 1

    def _best_match(self, lat: float, lon: float):
        best_id = None
        best_dist = float('inf')

        for cid, det in self.detected_people.items():
            try:
                dist = gps_distance(lat, lon, det["latitude"], det["longitude"])
            except ValueError:
                continue

            if dist < best_dist:
                best_dist = dist
                best_id = int(cid)

        if best_id is not None and best_dist <= self.threshold:
            return best_id, best_dist

        return None, float('inf')

    def _ensure_detected_entry(self, casualty_id: int, lat: float, lon: float, robot: str):
        cid = str(casualty_id)

        if cid not in self.detected_people:
            self.detected_people[cid] = {
                "casualty_id": casualty_id,
                "latitude": float(lat),
                "longitude": float(lon),
                "robot": robot,
                "category": None,
                "assessment": {},
                "casualty_submitted": False,
                "assessment_submitted": False,
            }
        else:
            self.detected_people[cid]["latitude"] = float(lat)
            self.detected_people[cid]["longitude"] = float(lon)
            self.detected_people[cid]["robot"] = robot

        return cid

    def _match_or_create(self, lat: float, lon: float, robot: str):
        with self.mutex:
            casualty_id, dist = self._best_match(lat, lon)

            if casualty_id is not None:
                self._ensure_detected_entry(casualty_id, lat, lon, robot)
                self._save_detected_people()

                self.append_match_log({
                    "timestamp": self.get_clock().now().to_msg().sec,
                    "robot": robot,
                    "lat": float(lat),
                    "lon": float(lon),
                    "matched_id": int(casualty_id),
                    "distance": float(dist),
                    "is_new": False,
                    "source": "detected_people",
                })
                return casualty_id, dist, False

            new_id = self._next_casualty_id()
            self._ensure_detected_entry(new_id, lat, lon, robot)
            self._save_detected_people()

            self.append_match_log({
                "timestamp": self.get_clock().now().to_msg().sec,
                "robot": robot,
                "lat": float(lat),
                "lon": float(lon),
                "matched_id": int(new_id),
                "distance": None,
                "is_new": True,
                "source": None,
            })
            return new_id, float('inf'), True

    # -------------------------------------------------
    # Image helper
    # -------------------------------------------------

    def save_compressed_image(self, image_msg, casualty_id: int, robot: str, suffix: str) -> str | None:
        try:
            if not image_msg.data:
                self.get_logger().warn(f"[{robot}] No compressed image data for casualty {casualty_id}")
                return None

            filename = f"{casualty_id}_{robot}_{suffix}.jpg"
            img_path = os.path.join(self.image_save_dir, filename)

            with open(img_path, "wb") as f:
                f.write(bytes(image_msg.data))

            self.get_logger().info(f"[{robot}] Saved image to {img_path}")
            return img_path

        except Exception as e:
            self.get_logger().error(
                f"[{robot}] Failed to save image for casualty {casualty_id}: {e}"
            )
            return None

    # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------

    def gate1_callback(self, msg: Gate1, robot: str):
        lat = float(msg.latitude)
        lon = float(msg.longitude)

        self.current_gps[robot]["latitude"] = lat
        self.current_gps[robot]["longitude"] = lon

        casualty_id, dist, is_new = self._match_or_create(lat, lon, robot)

        if is_new:
            self.get_logger().info(
                f"[{robot}] HMT Gate1 created new detected person {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] HMT Gate1 matched detected person {casualty_id} at {dist:.2f} m"
            )

    def gate2_callback(self, msg: Gate2, robot: str):
        lat = self.current_gps[robot]["latitude"]
        lon = self.current_gps[robot]["longitude"]

        if lat is None or lon is None:
            self.get_logger().warn(
                f"[{robot}] Received Gate2 before Gate1 GPS; cannot submit HMT casualty"
            )
            return

        casualty_id, dist = self._best_match(lat, lon)
        if casualty_id is None:
            self.get_logger().warn(
                f"[{robot}] No detected person match within {self.threshold}m for HMT casualty"
            )
            return

        cid = str(casualty_id)

        with self.mutex:
            self.detected_people[cid]["category"] = int(msg.category)
            self._save_detected_people()

            if self.detected_people[cid]["casualty_submitted"]:
                self.get_logger().info(
                    f"[{robot}] HMT casualty already submitted for detected person {casualty_id}; ignoring"
                )
                return

        self.get_logger().info(
            f"[{robot}] Matched detected person {casualty_id} at {dist:.2f}m, submitting HMT casualty"
        )

        try:
            response = hmt_location_report(
                lat=lat,
                lon=lon,
                category=int(msg.category),
                time_ago=0.0,
                id=casualty_id,
                system=robot
            )
        except Exception as e:
            self.get_logger().error(
                f"[{robot}] hmt_location_report exception for casualty {casualty_id}: {e}"
            )
            return

        if response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] hmt/casualty failed: {response.status_code} {response.text}"
            )
            return

        img_path = self.save_compressed_image(msg.image, casualty_id, robot, "gate2_hmt")
        if img_path is None:
            self.get_logger().error(
                f"[{robot}] Image save failed for HMT casualty {casualty_id}; not marking submitted"
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
                f"[{robot}] submit_image exception for HMT casualty {casualty_id}: {e}"
            )
            return

        if image_response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] submit_image failed for HMT casualty {casualty_id}: "
                f"{image_response.status_code} {image_response.text}"
            )
            return

        with self.mutex:
            self.detected_people[cid]["casualty_submitted"] = True
            self._save_detected_people()

        self.get_logger().info(
            f"[{robot}] Submitted HMT casualty and image for detected person {casualty_id}"
        )

    def gate3_callback(self, msg: Gate3, robot: str):
        lat = self.current_gps[robot]["latitude"]
        lon = self.current_gps[robot]["longitude"]

        if lat is None or lon is None:
            self.get_logger().warn(
                f"[{robot}] Received Gate3 before Gate1 GPS; cannot submit HMT assessment"
            )
            return

        casualty_id, dist = self._best_match(lat, lon)
        if casualty_id is None:
            self.get_logger().warn(
                f"[{robot}] No detected person match within {self.threshold}m for HMT assessment"
            )
            return

        cid = str(casualty_id)

        assessment = {
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

        with self.mutex:
            self.detected_people[cid]["assessment"] = assessment
            self._save_detected_people()

            if self.detected_people[cid]["assessment_submitted"]:
                self.get_logger().info(
                    f"[{robot}] HMT assessment already submitted for detected person {casualty_id}; ignoring"
                )
                return

        self.get_logger().info(
            f"[{robot}] Matched detected person {casualty_id} at {dist:.2f}m, submitting HMT assessment block"
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
            try:
                response = hmt_assessment_report(
                    type=field_type,
                    value=int(value),
                    time_ago=0.0,
                    id=casualty_id,
                    system=robot
                )
            except Exception as e:
                self.get_logger().error(
                    f"[{robot}] hmt_assessment_report exception for {field_type}: {e}"
                )
                all_ok = False
                continue

            if response.status_code != 200:
                self.get_logger().error(
                    f"[{robot}] hmt/assessment failed for {field_type}: "
                    f"{response.status_code} {response.text}"
                )
                all_ok = False

        if not all_ok:
            return

        img_path = self.save_compressed_image(msg.image, casualty_id, robot, "gate3_hmt")
        if img_path is None:
            self.get_logger().error(
                f"[{robot}] Image save failed for HMT assessment {casualty_id}; not marking submitted"
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
                f"[{robot}] submit_image exception for HMT assessment {casualty_id}: {e}"
            )
            return

        if image_response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] submit_image failed for HMT assessment {casualty_id}: "
                f"{image_response.status_code} {image_response.text}"
            )
            return

        with self.mutex:
            self.detected_people[cid]["assessment_submitted"] = True
            self._save_detected_people()

        self.get_logger().info(
            f"[{robot}] Submitted HMT assessment and image for detected person {casualty_id}"
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