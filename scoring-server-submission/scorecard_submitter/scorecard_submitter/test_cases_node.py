#!/usr/bin/env python3
import os
import sys
import json
import glob
from typing import Optional, List, Dict

import rclpy
from rclpy.node import Node

from dtc_msgs.msg import CasualtyFixArray, CasualtyFix, Gate1


class Gate1DummyIntegrationTest(Node):
    def __init__(self):
        super().__init__('gate1_dummy_test_node')

        self.declare_parameter('debug_data_path', '/tmp/casualty_debug')
        self.declare_parameter('uav_robot', 'dione')
        self.declare_parameter('ugv_robots', ['deimos', 'phobos', 'titania', 'oberon'])

        self.debug_data_path = self.get_parameter('debug_data_path').value
        self.uav_robot = self.get_parameter('uav_robot').value
        self.ugv_robots = list(self.get_parameter('ugv_robots').value)

        self.uav_pub = self.create_publisher(
            CasualtyFixArray,
            f'/{self.uav_robot}/casualty_info',
            10
        )

        self.ugv_pubs = {
            robot: self.create_publisher(Gate1, f'/{robot}/triage_report/gate1', 10)
            for robot in self.ugv_robots
        }

        self.test_failed = False
        self.test_fail_reasons: List[str] = []
        self.latest_debug_dir: Optional[str] = None

        self.steps = [
            self.step_1_publish_uav_seed,
            self.step_2_publish_ugv_match_uav,
            self.step_3_assert_first_match,
            self.step_4_publish_ugv_match_prior_same_area,
            self.step_5_assert_second_match_same_id,
            self.step_6_publish_ugv_far_new_id,
            self.step_7_assert_third_is_new,
            self.step_8_publish_ugv_match_prior_ugv,
            self.step_9_assert_fourth_matches_third,
            self.step_10_finish,
        ]
        self.current_step = 0

        self.get_logger().info('Gate1 dummy integration test node started')
        self.get_logger().info(f'Watching debug path: {self.debug_data_path}')

        self.timer = self.create_timer(1.0, self.run_next_step)

    # ----------------------------
    # Utilities
    # ----------------------------

    def run_next_step(self):
        if self.current_step >= len(self.steps):
            return

        try:
            self.steps[self.current_step]()
        except Exception as e:
            self.fail(f'Exception during step {self.current_step + 1}: {e}')
            self.shutdown_with_result()
            return

        self.current_step += 1

    def fail(self, reason: str):
        self.test_failed = True
        self.test_fail_reasons.append(reason)
        self.get_logger().error(reason)

    def latest_gate1_debug_dir(self) -> Optional[str]:
        pattern = os.path.join(self.debug_data_path, 'gate1_debug_*')
        matches = glob.glob(pattern)
        if not matches:
            return None
        matches.sort(key=os.path.getmtime, reverse=True)
        return matches[0]

    def read_json(self, path: str):
        with open(path, 'r') as f:
            return json.load(f)

    def get_match_log(self) -> List[Dict]:
        debug_dir = self.latest_gate1_debug_dir()
        if debug_dir is None:
            raise RuntimeError(
                f'No gate1 debug directory found under {self.debug_data_path}. '
                f'Is gate1_node running?'
            )
        self.latest_debug_dir = debug_dir
        return self.read_json(os.path.join(debug_dir, 'match_log.json'))

    def get_uav_json(self) -> List[Dict]:
        if self.latest_debug_dir is None:
            self.latest_debug_dir = self.latest_gate1_debug_dir()
        if self.latest_debug_dir is None:
            raise RuntimeError('No debug directory available')
        return self.read_json(os.path.join(self.latest_debug_dir, 'uav_detections.json'))

    def get_ugv_json(self) -> List[Dict]:
        if self.latest_debug_dir is None:
            self.latest_debug_dir = self.latest_gate1_debug_dir()
        if self.latest_debug_dir is None:
            raise RuntimeError('No debug directory available')
        return self.read_json(os.path.join(self.latest_debug_dir, 'ugv_detections.json'))

    def publish_uav_array(self, casualties: List[Dict]):
        msg = CasualtyFixArray()
        msg.casualties = []

        for c in casualties:
            fix = CasualtyFix()
            fix.casualty_id = int(c['casualty_id'])
            fix.location.latitude = float(c['lat'])
            fix.location.longitude = float(c['lon'])
            msg.casualties.append(fix)

        self.uav_pub.publish(msg)
        self.get_logger().info(f'Published UAV array with {len(msg.casualties)} casualties')

    def publish_gate1(self, robot: str, lat: float, lon: float):
        if robot not in self.ugv_pubs:
            raise RuntimeError(f'No publisher configured for robot "{robot}"')

        msg = Gate1()
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        self.ugv_pubs[robot].publish(msg)
        self.get_logger().info(
            f'Published Gate1 from {robot}: lat={lat:.7f}, lon={lon:.7f}'
        )

    def assert_true(self, condition: bool, message: str):
        if not condition:
            self.fail(message)

    # ----------------------------
    # Scripted test steps
    # ----------------------------

    def step_1_publish_uav_seed(self):
        """
        Seed one UAV casualty with ID 42.
        """
        self.get_logger().info('TEST 1: Publishing UAV seed casualty')
        self.publish_uav_array([
            {
                'casualty_id': 42,
                'lat': 35.1563000,
                'lon': -79.5641000,
            }
        ])

    def step_2_publish_ugv_match_uav(self):
        """
        Publish a UGV report close to the UAV casualty.
        Expected: match ID 42.
        """
        self.get_logger().info('TEST 2: Publishing UGV report near UAV casualty')
        self.publish_gate1('deimos', 35.1563010, -79.5640990)

    def step_3_assert_first_match(self):
        logs = self.get_match_log()
        self.assert_true(len(logs) >= 1, 'Expected at least 1 entry in match_log.json')

        last = logs[-1]
        self.assert_true(last['matched_id'] == 42, f'Expected first match to ID 42, got {last}')
        self.assert_true(last['is_new'] is False, f'Expected first match to not be new, got {last}')
        self.assert_true(last['source'] == 'uav', f'Expected source=uav, got {last}')

        self.get_logger().info('PASS: First UGV report matched UAV ID 42')

    def step_4_publish_ugv_match_prior_same_area(self):
        """
        Publish another UGV near the same spot.
        Expected: match same casualty ID (42), via UAV or prior UGV.
        """
        self.get_logger().info('TEST 3: Publishing second UGV near same area')
        self.publish_gate1('phobos', 35.1563020, -79.5641010)

    def step_5_assert_second_match_same_id(self):
        logs = self.get_match_log()
        self.assert_true(len(logs) >= 2, 'Expected at least 2 entries in match_log.json')

        last = logs[-1]
        self.assert_true(last['matched_id'] == 42, f'Expected second match to ID 42, got {last}')
        self.assert_true(last['is_new'] is False, f'Expected second match to not be new, got {last}')

        self.get_logger().info('PASS: Second UGV report reused casualty ID 42')

    def step_6_publish_ugv_far_new_id(self):
        """
        Publish a far-away UGV report.
        Expected: new casualty ID created.
        """
        self.get_logger().info('TEST 4: Publishing far-away UGV report')
        self.publish_gate1('oberon', 35.1605000, -79.5700000)

    def step_7_assert_third_is_new(self):
        logs = self.get_match_log()
        self.assert_true(len(logs) >= 3, 'Expected at least 3 entries in match_log.json')

        last = logs[-1]
        self.assert_true(last['is_new'] is True, f'Expected far-away report to be new, got {last}')
        self.assert_true(last['matched_id'] != 42, f'Expected new ID not equal to 42, got {last}')

        self.new_far_id = last['matched_id']
        self.get_logger().info(f'PASS: Far-away UGV report created new casualty ID {self.new_far_id}')

    def step_8_publish_ugv_match_prior_ugv(self):
        """
        Publish another UGV near the far-away UGV report.
        Expected: match the prior UGV-created ID.
        """
        self.get_logger().info('TEST 5: Publishing UGV near prior far-away UGV casualty')
        self.publish_gate1('titania', 35.1605010, -79.5699990)

    def step_9_assert_fourth_matches_third(self):
        logs = self.get_match_log()
        self.assert_true(len(logs) >= 4, 'Expected at least 4 entries in match_log.json')

        last = logs[-1]
        self.assert_true(
            last['matched_id'] == self.new_far_id,
            f'Expected fourth report to match prior UGV ID {self.new_far_id}, got {last}'
        )
        self.assert_true(
            last['is_new'] is False,
            f'Expected fourth report to reuse prior UGV casualty, got {last}'
        )

        ugv_data = self.get_ugv_json()
        self.assert_true(len(ugv_data) >= 4, 'Expected ugv_detections.json to contain entries')

        uav_data = self.get_uav_json()
        self.assert_true(len(uav_data) >= 1, 'Expected uav_detections.json to contain UAV seed')

        self.get_logger().info('PASS: Prior UGV matching works')
        self.get_logger().info(f'Debug dir used: {self.latest_debug_dir}')

    def step_10_finish(self):
        if self.test_failed:
            self.shutdown_with_result()
            return

        self.get_logger().info('All integration tests passed')
        self.shutdown_with_result()

    def shutdown_with_result(self):
        if self.test_failed:
            self.get_logger().error('Integration test FAILED')
            for reason in self.test_fail_reasons:
                self.get_logger().error(f' - {reason}')
            code = 1
        else:
            self.get_logger().info('Integration test PASSED')
            code = 0

        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(code)


def main(args=None):
    rclpy.init(args=args)
    node = Gate1DummyIntegrationTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()