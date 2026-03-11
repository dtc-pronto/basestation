#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import cv2

from dtc_msgs.msg import CasualtyImage
from submission import submit_image
from cv_bridge import CvBridge


UGV = ['deimos', 'phobos', 'titania', 'oberon']


class CasualtyImageNode(Node):
    def __init__(self):
        super().__init__('casualty_image_node')

        self.declare_parameter('robot_names', ['deimos', 'phobos', 'titania', 'oberon'])
        self.declare_parameter('image_save_path', '/tmp/casualty_images')

        self.robots = self.get_parameter('robot_names').value
        self.image_dir = self.get_parameter('image_save_path').value
        os.makedirs(self.image_dir, exist_ok=True)

        self.bridge = CvBridge()

        self.subscriptions_list = []
        for robot in self.robots:
            if robot in UGV:
                self.subscriptions_list.append(self.create_subscription(
                    CasualtyImage,
                    f'/{robot}/triage_report/gate3',
                    lambda msg, r=robot: self.image_callback(msg, r),
                    10))
                self.get_logger().info(f"Subscribed to /{robot}/triage_report/gate3")

    def image_callback(self, msg: CasualtyImage, robot: str):
        casualty_id = msg.casualty_id
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        time_now = self.get_clock().now().nanoseconds / 1e9

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"[{robot}] Image conversion failed: {e}")
            return

        image_path = os.path.join(
            self.image_dir,
            f"{robot}_casualty{casualty_id}_{msg.header.stamp.sec}.jpg"
        )
        cv2.imwrite(image_path, cv_img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.get_logger().info(f"[{robot}] Saved image to {image_path}")

        r = submit_image(image_path, stamp_sec, time_now, casualty_id)
        if r.status_code != 200:
            self.get_logger().error(
                f"[{robot}] submit_image failed: {r.status_code} {r.text}"
            )
        else:
            self.get_logger().info(
                f"[{robot}] Image submitted for casualty {casualty_id}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CasualtyImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
