#!/usr/bin/env python3
import os
import json
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node

from dtc_msgs.msg import CasualtyFixArray, Gate1
from helpers import gps_distance
from submission import location_report, submit_image


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
        self.declare_parameter('image_save_dir', '/home/dtc/data/casualty_images')

        self.robots = self.get_parameter('robot_names').value
        self.threshold = float(self.get_parameter('gps_threshold').value)
        debug_data_path = self.get_parameter('debug_data_path').value
        self.image_save_dir = self.get_parameter('image_save_dir').value
        os.makedirs(self.image_save_dir, exist_ok=True)

        self.get_logger().info(f"GPS match threshold: {self.threshold} m")
        self.get_logger().info(f"Image save dir: {self.image_save_dir}")

        self.uav_detections = []
        self.ugv_detections = []
        self.mutex = threading.Lock()
        self.subscriptions_list = []

        folder_name = f"gate1_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.debug_path = os.path.join(debug_data_path, folder_name)
        os.makedirs(self.debug_path, exist_ok=True)

        self.uav_json = os.path.join(self.debug_path, "uav_detections.json")
        self.ugv_json = os.path.join(self.debug_path, "ugv_detections.json")
        self.match_log_json = os.path.join(self.debug_path, "match_log.json")

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

                return casualty_id, dist, False, source

            new_id = self._next_casualty_id()

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

            return new_id, float('inf'), True, None

    def save_compressed_image(self, msg: Gate1, casualty_id: int, robot: str) -> str | None:
        try:
            if not msg.image.data:
                self.get_logger().warn(f"[{robot}] No compressed image data for casualty {casualty_id}")
                return None

            filename = f"{casualty_id}_{robot}.jpg"
            img_path = os.path.join(self.image_save_dir, filename)

            with open(img_path, "wb") as f:
                f.write(bytes(msg.image.data))

            self.get_logger().info(f"[{robot}] Saved image to {img_path}")
            return img_path

        except Exception as e:
            self.get_logger().error(
                f"[{robot}] Failed to save image for casualty {casualty_id}: {e}"
            )
            return None

    def ugv_callback(self, msg: Gate1, robot: str) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)

        casualty_id, dist, is_new, source = self._match_or_create(lat, lon, robot)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] No UAV/UGV match within {self.threshold} m — creating new casualty ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Matched casualty {casualty_id} at {dist:.2f} m from {source}"
            )

        try:
            response = location_report(
                lat=lat,
                lon=lon,
                level=1,
                id=casualty_id,
                system=robot
            )
            self.get_logger().info(f"[{robot}] location_report status: {response.status_code}")
            if response.status_code != 200:
                self.get_logger().error(
                    f"[{robot}] location_report failed: {response.status_code} {response.text}"
                )
        except Exception as e:
            self.get_logger().error(f"[{robot}] location_report exception: {e}")

        img_path = self.save_compressed_image(msg, casualty_id, robot)
        if img_path is None:
            return

        try:
            response = submit_image(
                image_path=img_path,
                time=0,
                time_now=0,
                id=casualty_id
            )
            self.get_logger().info(f"[{robot}] submit_image status: {response.status_code}")
            if response.status_code != 200:
                self.get_logger().error(
                    f"[{robot}] submit_image failed: {response.status_code} {response.text}"
                )
            else:
                self.get_logger().info(
                    f"[{robot}] Submitted image for casualty {casualty_id}"
                )
        except Exception as e:
            self.get_logger().error(f"[{robot}] submit_image exception: {e}")


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