#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import json
import cv2

from dtc_msgs.msg import CasualtyFixArray, Triage, Assessment, HeartRate, RespirationRate, CasualtyImage
from sensor_msgs.msg import CompressedImage
from basestation_msgs.msg import FullReport
from helpers import gps_distance
from cv_bridge import CvBridge


UAV = ['dione']
UGV = ['deimos', 'phobos', 'titania', 'oberon']


class ATAKReportNode(Node):
    """
    Aggregates per-casualty data from the HMT message types and publishes
    FullReport on /basestation/full_report for ATAK consumption.

    Per-casualty record:
      - lat/lon      : GPS from the most-recent UGV report
      - robot        : reporting UGV
      - category     : triage category (Triage.category.value)
      - assessments  : {type: value} accumulated from Assessment messages
      - vitals       : {'hr': float, 'rr': float}
      - image        : CompressedImage from CasualtyImage (most recent)
    """

    def __init__(self):
        super().__init__('atak_report_node')

        self.declare_parameter('robot_names', ['dione', 'deimos', 'phobos', 'titania', 'oberon'])
        self.declare_parameter('gps_threshold', 5.0)

        self.robots = self.get_parameter('robot_names').value
        self.threshold = self.get_parameter('gps_threshold').value
        self.get_logger().info(f"GPS match threshold: {self.threshold}m")

        self.uav_detections = []    # list of {casualty_id, lat, lon}
        self.casualty_records = {}  # casualty_id -> record dict
        self.mutex = threading.RLock()  # re-entrant: callbacks nest into _match_or_create
        self.bridge = CvBridge()

        self.full_report_pub = self.create_publisher(FullReport, '/basestation/full_report', 10)

        self.subscriptions_list = []
        for robot in self.robots:
            if robot in UAV:
                self.subscriptions_list.append(self.create_subscription(
                    CasualtyFixArray,
                    f'/{robot}/casualty_info',
                    lambda msg, r=robot: self.uav_callback(msg, r),
                    10))
                self.get_logger().info(f"Subscribed to /{robot}/casualty_info (UAV)")
            elif robot in UGV:
                self.subscriptions_list.append(self.create_subscription(
                    Triage,
                    f'/{robot}/triage_report',
                    lambda msg, r=robot: self.triage_callback(msg, r),
                    10))
                self.subscriptions_list.append(self.create_subscription(
                    Assessment,
                    f'/{robot}/assessment_report',
                    lambda msg, r=robot: self.assessment_callback(msg, r),
                    10))
                self.subscriptions_list.append(self.create_subscription(
                    HeartRate,
                    f'/{robot}/heart_rate',
                    lambda msg, r=robot: self.hr_callback(msg, r),
                    10))
                self.subscriptions_list.append(self.create_subscription(
                    RespirationRate,
                    f'/{robot}/respiration_rate',
                    lambda msg, r=robot: self.rr_callback(msg, r),
                    10))
                self.subscriptions_list.append(self.create_subscription(
                    CasualtyImage,
                    f'/{robot}/casualty_image',
                    lambda msg, r=robot: self.image_callback(msg, r),
                    10))
                self.get_logger().info(f"Subscribed to /{robot} topics (UGV)")

    # -------------------------------------------------------------------------
    # UAV callback
    # -------------------------------------------------------------------------

    def uav_callback(self, msg: CasualtyFixArray, robot: str):
        with self.mutex:
            self.uav_detections = [
                {
                    'casualty_id': c.casualty_id,
                    'lat': c.location.latitude,
                    'lon': c.location.longitude,
                }
                for c in msg.casualties
            ]
        self.get_logger().info(f"[{robot}] Updated UAV table: {len(self.uav_detections)} casualties")

    # -------------------------------------------------------------------------
    # GPS matching — same pattern as hmt_node.py
    # -------------------------------------------------------------------------

    def _match_or_create(self, lat, lon):
        """
        GPS-match against UAV detections. On no match within threshold, create a
        new entry with the next available ID. Returns (casualty_id, dist, is_new).
        Must be called with self.mutex held (RLock allows re-entry).
        """
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

        new_id = len(self.uav_detections)
        self.uav_detections.append({'casualty_id': new_id, 'lat': lat, 'lon': lon})
        return new_id, min_dist, True

    # -------------------------------------------------------------------------
    # Per-casualty record management
    # -------------------------------------------------------------------------

    def _get_or_create_record(self, casualty_id, lat, lon, robot):
        """Return the record for casualty_id, creating it if needed. Must hold mutex."""
        if casualty_id not in self.casualty_records:
            self.casualty_records[casualty_id] = {
                'casualty_id': casualty_id,
                'lat': lat,
                'lon': lon,
                'robot': robot,
                'category': None,
                'assessments': {},
                'vitals': {},
                'image': None,
            }
        else:
            self.casualty_records[casualty_id]['lat'] = lat
            self.casualty_records[casualty_id]['lon'] = lon
            self.casualty_records[casualty_id]['robot'] = robot
        return self.casualty_records[casualty_id]

    def _build_full_report(self, casualty_id):
        """Build a FullReport message from the current record. Must hold mutex."""
        record = self.casualty_records.get(casualty_id)
        if record is None:
            return None

        msg = FullReport()
        msg.casualty_id = casualty_id
        msg.report = json.dumps({
            'casualty_id': record['casualty_id'],
            'lat': record['lat'],
            'lon': record['lon'],
            'robot': record['robot'],
            'category': record['category'],
            'assessments': dict(record['assessments']),
            'vitals': dict(record['vitals']),
        })
        msg.image = record['image'] if record['image'] is not None else CompressedImage()
        return msg

    def _publish(self, report_msg, casualty_id, robot):
        """Publish report_msg and log. Called outside the mutex."""
        if report_msg is not None:
            self.full_report_pub.publish(report_msg)
            self.get_logger().info(f"[{robot}] Published FullReport for casualty {casualty_id}")

    # -------------------------------------------------------------------------
    # UGV callbacks
    # -------------------------------------------------------------------------

    def triage_callback(self, msg: Triage, robot: str):
        lat = msg.location.location.latitude
        lon = msg.location.location.longitude

        with self.mutex:
            casualty_id, dist, is_new = self._match_or_create(lat, lon)
            record = self._get_or_create_record(casualty_id, lat, lon, robot)
            record['category'] = msg.category.value
            report_msg = self._build_full_report(casualty_id)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] Triage: no UAV match (closest: {dist:.1f}m) — new ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Triage: matched casualty {casualty_id} at {dist:.1f}m, category={msg.category.value}"
            )
        self._publish(report_msg, casualty_id, robot)

    def assessment_callback(self, msg: Assessment, robot: str):
        lat = msg.location.location.latitude
        lon = msg.location.location.longitude

        with self.mutex:
            casualty_id, dist, is_new = self._match_or_create(lat, lon)
            record = self._get_or_create_record(casualty_id, lat, lon, robot)
            record['assessments'][msg.type.data] = msg.value.value
            report_msg = self._build_full_report(casualty_id)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] Assessment: no UAV match (closest: {dist:.1f}m) — new ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Assessment: matched casualty {casualty_id} at {dist:.1f}m, "
                f"{msg.type.data}={msg.value.value}"
            )
        self._publish(report_msg, casualty_id, robot)

    def hr_callback(self, msg: HeartRate, robot: str):
        lat = msg.location.location.latitude
        lon = msg.location.location.longitude

        with self.mutex:
            casualty_id, dist, is_new = self._match_or_create(lat, lon)
            record = self._get_or_create_record(casualty_id, lat, lon, robot)
            record['vitals']['hr'] = msg.rate.data
            report_msg = self._build_full_report(casualty_id)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] HR: no UAV match (closest: {dist:.1f}m) — new ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] HR: matched casualty {casualty_id} at {dist:.1f}m, hr={msg.rate.data:.1f}"
            )
        self._publish(report_msg, casualty_id, robot)

    def rr_callback(self, msg: RespirationRate, robot: str):
        lat = msg.location.location.latitude
        lon = msg.location.location.longitude

        with self.mutex:
            casualty_id, dist, is_new = self._match_or_create(lat, lon)
            record = self._get_or_create_record(casualty_id, lat, lon, robot)
            record['vitals']['rr'] = msg.rate.data
            report_msg = self._build_full_report(casualty_id)

        if is_new:
            self.get_logger().warn(
                f"[{robot}] RR: no UAV match (closest: {dist:.1f}m) — new ID {casualty_id}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] RR: matched casualty {casualty_id} at {dist:.1f}m, rr={msg.rate.data:.1f}"
            )
        self._publish(report_msg, casualty_id, robot)

    def image_callback(self, msg: CasualtyImage, robot: str):
        casualty_id = msg.casualty_id

        try:
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
                    'casualty_id': casualty_id,
                    'lat': None,
                    'lon': None,
                    'robot': robot,
                    'category': None,
                    'assessments': {},
                    'vitals': {},
                    'image': None,
                }
            self.casualty_records[casualty_id]['image'] = cimg
            report_msg = self._build_full_report(casualty_id)

        self.get_logger().info(f"[{robot}] Updated image for casualty {casualty_id}")
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
