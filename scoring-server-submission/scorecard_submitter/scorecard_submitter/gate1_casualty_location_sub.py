#!/usr/bin/env python3
import os
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node

from dtc_msgs.msg import CasualtyFixArray, Gate1
from helpers import gps_distance
from submission import location_report


UAV = ['dione']
UGV = ['deimos', 'phobos', 'titania', 'oberon']


class CasualtyLocationSub(Node):
    def __init__(self):
        super().__init__('gate1_node')

        self.declare_parameter(
            'robot_names',
            ['dione', 'deimos', 'phobos', 'titania', 'oberon']
        )
        self.declare_parameter('gps_threshold', 5.0)
        self.declare_parameter('debug_data_path', '/tmp/casualty_debug')

        self.robots = self.get_parameter('robot_names').value
        self.threshold = self.get_parameter('gps_threshold').value
        debug_data_path = self.get_parameter('debug_data_path').value

        self.get_logger().info(f"GPS match threshold: {self.threshold} m")

        # In-memory detection tables
        self.uav_detections = []   # [{"casualty_id": int, "lat": float, "lon": float}]
        self.ugv_detections = []   # [{"casualty_id": int, "lat": float, "lon": float, "robot": str}]
        self.mutex = threading.Lock()
        self.subscriptions_list = []

        # Debug JSON paths
        folder_name = f"gate1_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.debug_path = os.path.join(debug_data_path, folder_name)
        os.makedirs(self.debug_path, exist_ok=True)

        self.uav_json = os.path.join(self.debug_path, "uav_detections.json")
        self.ugv_json = os.path.join(self.debug_path, "ugv_detections.json")
        self.match_log_json = os.path.join(self.debug_path, "match_log.json")

        # initialize empty files
        self.write_json(self.uav_json, [])
        self.write_json(self.ugv_json, [])
        self.write_json(self.match_log_json, [])

        self.get_logger().info(f"Debug JSON path: {self.debug_path}")

        for robot in self.robots:
            if robot in UAV:
                self.subscriptions_list.append(
                    self.create_subscription(
                        CasualtyFixArray,
                        f"/{robot}/casualty_info",
                        lambda msg, r=robot: self.uav_callback(msg, r),
                        10
                    )
                )
                self.get_logger().info(f"Subscribed to /{robot}/casualty_info (UAV)")

            elif robot in UGV:
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate1,
                        f"/{robot}/triage_report/gate1",
                        lambda msg, r=robot: self.ugv_callback(msg, r),
                        10
                    )
                )
                self.get_logger().info(f"Subscribed to /{robot}/triage_report/gate1 (UGV)")

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
            self.get_logger().error(f"Failed writing match log: {e}")

    def uav_callback(self, msg: CasualtyFixArray, robot: str) -> None:
        detections = []
        for casualty in msg.casualties:
            detections.append({
                "casualty_id": int(casualty.casualty_id),
                "lat": float(casualty.location.latitude),
                "lon": float(casualty.location.longitude),
            })

        with self.mutex:
            self.uav_detections = detections
            self.write_json(self.uav_json, self.uav_detections)

        self.get_logger().info(
            f"[{robot}] Updated UAV detections: {len(self.uav_detections)} casualties"
        )

    def _next_casualty_id(self) -> int:
        ids = [det["casualty_id"] for det in self.uav_detections]
        ids.extend(det["casualty_id"] for det in self.ugv_detections)
        return max(ids, default=-1) + 1

    def _best_match(self, lat: float, lon: float):
        """
        Search both UAV and prior UGV detections for nearest match.
        Returns:
            (matched_id, distance, source) or (None, inf, None)
        """
        best_id = None
        best_dist = float('inf')
        best_source = None

        for det in self.uav_detections:
            try:
                dist = gps_distance(lat, lon, det["lat"], det["lon"])
            except ValueError:
                continue

            if dist < best_dist:
                best_dist = dist
                best_id = det["casualty_id"]
                best_source = "uav"

        for det in self.ugv_detections:
            try:
                dist = gps_distance(lat, lon, det["lat"], det["lon"])
            except ValueError:
                continue

            if dist < best_dist:
                best_dist = dist
                best_id = det["casualty_id"]
                best_source = f'ugv:{det["robot"]}'

        if best_id is not None and best_dist <= self.threshold:
            return best_id, best_dist, best_source

        return None, float('inf'), None

    def _match_or_create(self, lat: float, lon: float, robot: str):
        with self.mutex:
            casualty_id, dist, source = self._best_match(lat, lon)

            if casualty_id is not None:
                is_new = False
                self.ugv_detections.append({
                    "casualty_id": int(casualty_id),
                    "lat": float(lat),
                    "lon": float(lon),
                    "robot": robot,
                })
                self.write_json(self.ugv_json, self.ugv_detections)

                self.append_match_log({
                    "timestamp": self.get_clock().now().to_msg().sec,
                    "robot": robot,
                    "lat": float(lat),
                    "lon": float(lon),
                    "matched_id": int(casualty_id),
                    "distance": float(dist),
                    "is_new": False,
                    "source": source,
                })

                return casualty_id, dist, is_new, source

            new_id = self._next_casualty_id()
            is_new = True

            self.ugv_detections.append({
                "casualty_id": int(new_id),
                "lat": float(lat),
                "lon": float(lon),
                "robot": robot,
            })
            self.write_json(self.ugv_json, self.ugv_detections)

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

            return new_id, float('inf'), is_new, None

    def ugv_callback(self, msg: Gate1, robot: str) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)

        casualty_id, dist, is_new, source = self._match_or_create(lat, lon, robot)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] No UAV/UGV match within {self.threshold} m "
                f"— creating new casualty ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Matched casualty {casualty_id} at {dist:.2f} m from {source}"
            )

        response = location_report(
            lat=lat,
            lon=lon,
            level=1,   # TODO: replace if level becomes available in Gate1
            id=casualty_id,
            system=robot
        )

        self.get_logger().info(f"Response Code: {response.status_code}")

        if response.status_code != 200:
            self.get_logger().error(
                f"[{robot}] location_report failed: "
                f"{response.status_code} {response.text}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Submitted location report for casualty {casualty_id}"
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