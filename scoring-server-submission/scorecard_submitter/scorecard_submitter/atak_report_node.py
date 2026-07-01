#!/usr/bin/env python3
import json
import threading

import rclpy
from rclpy.node import Node
import cv2

from std_msgs.msg import String
from dtc_msgs.msg import CasualtyFixArray, Gate1, Gate2, Gate3, Gate4, CasualtyImage, FullReport
from sensor_msgs.msg import CompressedImage
from helpers import gps_distance
from cv_bridge import CvBridge


UAV = ['dione']
UGV = ['deimos', 'phobos', 'titania', 'oberon', 'ares', 'aphrodite']


class ATAKReportNode(Node):
    """
    Aggregates Gate1/Gate2/Gate3/Gate4 information into a single FullReport
    per matched casualty for ATAK consumption.

    Record contents:
      - lat/lon      : latest matched GPS
      - robot        : latest reporting robot
      - category     : Gate2 category
      - assessments  : Gate3 fields
      - vitals       : Gate4 fields (hr, rr)
      - image        : latest image from CasualtyImage
    """

    def __init__(self):
        super().__init__('atak_report_node')

        self.declare_parameter(
            'robot_names',
            ['dione', 'deimos', 'phobos', 'titania', 'oberon', 'ares', 'aphrodite']
        )
        self.declare_parameter('gps_threshold', 5.0)

        self.robots = self.get_parameter('robot_names').value
        self.threshold = float(self.get_parameter('gps_threshold').value)

        self.get_logger().info(f"GPS match threshold: {self.threshold} m")

        self.uav_detections = []    # [{"casualty_id": int, "lat": float, "lon": float}]
        self.ugv_detections = []    # [{"casualty_id": int, "lat": float, "lon": float, "robot": str}]
        self.casualty_records = {}  # casualty_id -> record dict

        self.mutex = threading.RLock()
        self.bridge = CvBridge()

        self.full_report_pub = self.create_publisher(
            FullReport,
            '/basestation/full_report',
            10
        )

        self.subscriptions_list = []
        for robot in self.robots:
            if robot in UAV:
                self.subscriptions_list.append(
                    self.create_subscription(
                        CasualtyFixArray,
                        f'/{robot}/glider/casualty/fix',
                        lambda msg, r=robot: self.uav_callback(msg, r),
                        10
                    )
                )
                self.get_logger().info(f"Subscribed to /{robot}/casualty_info (UAV)")

            elif robot in UGV:
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate1,
                        f'/{robot}/triage_report/gate1',
                        lambda msg, r=robot: self.gate1_callback(msg, r),
                        10
                    )
                )
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate2,
                        f'/{robot}/triage_report/gate2',
                        lambda msg, r=robot: self.gate2_callback(msg, r),
                        10
                    )
                )
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate3,
                        f'/{robot}/triage_report/gate3',
                        lambda msg, r=robot: self.gate3_callback(msg, r),
                        10
                    )
                )
                self.subscriptions_list.append(
                    self.create_subscription(
                        Gate4,
                        f'/{robot}/triage_report/gate4',
                        lambda msg, r=robot: self.gate4_callback(msg, r),
                        10
                    )
                )
                #self.subscriptions_list.append(
                #    self.create_subscription(
                #        CasualtyImage,
                #        f'/{robot}/casualty_image',
                #        lambda msg, r=robot: self.image_callback(msg, r),
                #        10
                #    )
                #)
                self.get_logger().info(f"Subscribed to /{robot} Gate1/Gate2/Gate3/Gate4/image topics")

    # -------------------------------------------------------------------------
    # UAV callback
    # -------------------------------------------------------------------------

    def uav_callback(self, msg: CasualtyFixArray, robot: str):
        reports_to_publish = []

        with self.mutex:
            self.uav_detections = []

            for c in msg.casualties:
                casualty_id = int(c.casualty_id)
                lat = float(c.location.latitude)
                lon = float(c.location.longitude)

                self.uav_detections.append({
                    "casualty_id": casualty_id,
                    "lat": lat,
                    "lon": lon,
                })

                old_record = self.casualty_records.get(casualty_id)
                should_publish = True

                if old_record is None:
                    should_publish = True
                else:
                    old_lat = old_record["lat"]
                    old_lon = old_record["lon"]
                    if old_lat is None or old_lon is None:
                        should_publish = True
                    else:
                        try:
                            moved = gps_distance(lat, lon, old_lat, old_lon)
                            if moved > 1.0:
                                should_publish = True
                        except ValueError:
                            should_publish = True

                record = self._get_or_create_record(casualty_id, lat, lon, robot)

                if record.get("robot") is None or record.get("robot") in UAV:
                    record["robot"] = robot

                if should_publish:
                    report_msg = self._build_full_report(casualty_id)
                    if report_msg is not None:
                        reports_to_publish.append((report_msg, casualty_id))

        self.get_logger().info(
            f"[{robot}] Updated UAV detections: {len(self.uav_detections)} casualties"
        )

        for report_msg, casualty_id in reports_to_publish:
            self._publish(report_msg, casualty_id, robot)

    # -------------------------------------------------------------------------
    # Matching helpers
    # -------------------------------------------------------------------------

    def _next_casualty_id(self) -> int:
        ids = [det["casualty_id"] for det in self.uav_detections]
        ids.extend(det["casualty_id"] for det in self.ugv_detections)
        ids.extend(int(cid) for cid in self.casualty_records.keys())
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
        casualty_id, dist, source = self._best_match(lat, lon)

        if casualty_id is not None:
            self.ugv_detections.append({
                "casualty_id": int(casualty_id),
                "lat": float(lat),
                "lon": float(lon),
                "robot": robot,
            })
            return casualty_id, dist, False, source

        new_id = self._next_casualty_id()
        self.ugv_detections.append({
            "casualty_id": int(new_id),
            "lat": float(lat),
            "lon": float(lon),
            "robot": robot,
        })
        return new_id, float('inf'), True, None

    # -------------------------------------------------------------------------
    # Record helpers
    # -------------------------------------------------------------------------

    def _get_or_create_record(self, casualty_id: int, lat, lon, robot):
        key = int(casualty_id)

        if key not in self.casualty_records:
            self.casualty_records[key] = {
                "casualty_id": key,
                "lat": lat,
                "lon": lon,
                "robot": robot,
                "category": None,
                "assessments": {},
                "vitals": {},
                "image": None,
            }
        else:
            self.casualty_records[key]["lat"] = lat
            self.casualty_records[key]["lon"] = lon
            self.casualty_records[key]["robot"] = robot

        return self.casualty_records[key]

    def _build_full_report(self, casualty_id: int):
        record = self.casualty_records.get(casualty_id)
        if record is None:
            return None

        source = "ugv_confirmed"
        if record["robot"] in UAV:
            source = "uav_provisional"

        msg = FullReport()
        msg.casualty_id = int(casualty_id)
        msg.report = String(
            data=json.dumps({
                "casualty_id": record["casualty_id"],
                "lat": record["lat"],
                "lon": record["lon"],
                "robot": record["robot"],
                "source": source,
                "category": record["category"],
                "assessments": record["assessments"],
                "vitals": record["vitals"],
            })
        )

        if record["image"] is not None:
            msg.image = record["image"]
        else:
            msg.image = CompressedImage()

        return msg

    def _publish(self, report_msg, casualty_id: int, robot: str):
        if report_msg is not None:
            self.full_report_pub.publish(report_msg)
            self.get_logger().info(
                f"[{robot}] Published FullReport for casualty {casualty_id}"
            )

    # -------------------------------------------------------------------------
    # Gate1 / Gate2 / Gate3 / Gate4 callbacks
    # -------------------------------------------------------------------------

    def gate1_callback(self, msg: Gate1, robot: str):
        lat = float(msg.latitude)
        lon = float(msg.longitude)

        with self.mutex:
            casualty_id, dist, is_new, source = self._match_or_create(lat, lon, robot)
            self._get_or_create_record(casualty_id, lat, lon, robot)
            report_msg = self._build_full_report(casualty_id)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] Gate1: no UAV/UGV match within {self.threshold} m — new ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Gate1: matched casualty {casualty_id} at {dist:.2f} m from {source}"
            )

        self._publish(report_msg, casualty_id, robot)

    def gate2_callback(self, msg: Gate2, robot: str):
        candidates = [det for det in self.ugv_detections if det["robot"] == robot]
        if not candidates:
            self.get_logger().warn(
                f"[{robot}] Gate2 received before any Gate1 location for this robot"
            )
            return

        latest = candidates[-1]
        lat = latest["lat"]
        lon = latest["lon"]

        with self.mutex:
            casualty_id, dist, is_new, source = self._match_or_create(lat, lon, robot)
            record = self._get_or_create_record(casualty_id, lat, lon, robot)
            record["category"] = int(msg.category)
            report_msg = self._build_full_report(casualty_id)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] Gate2: no match within {self.threshold} m — new ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Gate2: matched casualty {casualty_id} at {dist:.2f} m, category={int(msg.category)}"
            )

        self._publish(report_msg, casualty_id, robot)

    def gate3_callback(self, msg: Gate3, robot: str):
        candidates = [det for det in self.ugv_detections if det["robot"] == robot]
        if not candidates:
            self.get_logger().warn(
                f"[{robot}] Gate3 received before any Gate1 location for this robot"
            )
            return

        latest = candidates[-1]
        lat = latest["lat"]
        lon = latest["lon"]

        with self.mutex:
            casualty_id, dist, is_new, source = self._match_or_create(lat, lon, robot)
            record = self._get_or_create_record(casualty_id, lat, lon, robot)
            record["assessments"] = {
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
            report_msg = self._build_full_report(casualty_id)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] Gate3: no match within {self.threshold} m — new ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Gate3: matched casualty {casualty_id} at {dist:.2f} m"
            )

        self._publish(report_msg, casualty_id, robot)

    def gate4_callback(self, msg: Gate4, robot: str):
        candidates = [det for det in self.ugv_detections if det["robot"] == robot]
        if not candidates:
            self.get_logger().warn(
                f"[{robot}] Gate4 received before any Gate1 location for this robot"
            )
            return

        latest = candidates[-1]
        lat = latest["lat"]
        lon = latest["lon"]

        with self.mutex:
            casualty_id, dist, is_new, source = self._match_or_create(lat, lon, robot)
            record = self._get_or_create_record(casualty_id, lat, lon, robot)
            record["vitals"]["hr"] = float(msg.hr)
            record["vitals"]["rr"] = float(msg.rr)
            report_msg = self._build_full_report(casualty_id)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] Gate4: no match within {self.threshold} m — new ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Gate4: matched casualty {casualty_id} at {dist:.2f} m, "
                f"hr={float(msg.hr):.1f}, rr={float(msg.rr):.1f}"
            )

        self._publish(report_msg, casualty_id, robot)

    def image_callback(self, msg: CasualtyImage, robot: str):
        casualty_id = int(msg.casualty_id)

        try:
            # If msg.image is sensor_msgs/Image
            cv_img = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
            _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 90])

            cimg = CompressedImage()
            cimg.header.stamp = self.get_clock().now().to_msg()
            cimg.format = 'jpg'
            cimg.data = buf.tobytes()
        except Exception as e:
            self.get_logger().error(f"[{robot}] Image conversion failed: {e}")
            return

        with self.mutex:
            if casualty_id not in self.casualty_records:
                self.casualty_records[casualty_id] = {
                    "casualty_id": casualty_id,
                    "lat": None,
                    "lon": None,
                    "robot": robot,
                    "category": None,
                    "assessments": {},
                    "vitals": {},
                    "image": None,
                }

            self.casualty_records[casualty_id]["image"] = cimg
            report_msg = self._build_full_report(casualty_id)

        self.get_logger().info(
            f"[{robot}] Updated image for casualty {casualty_id}"
        )
        self._publish(report_msg, casualty_id, robot)


def main(args=None):
    rclpy.init(args=args)
    node = ATAKReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()