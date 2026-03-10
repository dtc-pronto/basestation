#!/usr/bin/env python3
import os
import sys
import json
import glob
from typing import Optional, List, Dict

import rclpy
from rclpy.node import Node

from dtc_msgs.msg import CasualtyFixArray, CasualtyFix, Gate1, Gate2, Gate3, Gate4


GT_1 = {"casualty_id": 1, "lon": -75.20125345, "lat": 39.94148424}
GT_2 = {"casualty_id": 2, "lon": -75.20025345, "lat": 39.94148424}


class ScorecardIntegrationTest(Node):
    def __init__(self):
        super().__init__('scorecard_integration_test_node')

        self.declare_parameter('debug_data_path', '/tmp/casualty_debug')
        self.declare_parameter('uav_robot', 'dione')
        self.declare_parameter('ugv_robot', 'deimos')
        self.declare_parameter('test_gate', 4)

        self.debug_data_path = self.get_parameter('debug_data_path').value
        self.uav_robot = self.get_parameter('uav_robot').value
        self.ugv_robot = self.get_parameter('ugv_robot').value
        self.test_gate = int(self.get_parameter('test_gate').value)

        self.uav_pub = self.create_publisher(
            CasualtyFixArray,
            f'/{self.uav_robot}/casualty_info',
            10
        )
        self.gate1_pub = self.create_publisher(
            Gate1,
            f'/{self.ugv_robot}/triage_report/gate1',
            10
        )
        self.gate2_pub = self.create_publisher(
            Gate2,
            f'/{self.ugv_robot}/triage_report/gate2',
            10
        )
        self.gate3_pub = self.create_publisher(
            Gate3,
            f'/{self.ugv_robot}/triage_report/gate3',
            10
        )
        self.gate4_pub = self.create_publisher(
            Gate4,
            f'/{self.ugv_robot}/triage_report/gate4',
            10
        )

        self.test_failed = False
        self.test_fail_reasons: List[str] = []
        self.latest_debug_dir: Optional[str] = None

        self.steps = self._build_steps(self.test_gate)
        self.current_step = 0

        self.get_logger().info(f'Scorecard integration test node started for gate {self.test_gate}')
        self.get_logger().info(f'Watching debug path: {self.debug_data_path}')

        self.timer = self.create_timer(1.0, self.run_next_step)

    # ----------------------------
    # Step routing
    # ----------------------------

    def _build_steps(self, gate: int):
        if gate == 1:
            return [
                self.g1_step_1_publish_uav_seed,
                self.g1_step_2_publish_gate1_match_uav,
                self.g1_step_3_assert_first_match,
                self.g1_step_4_publish_gate1_far_new_id,
                self.g1_step_5_assert_second_is_new,
                self.finish,
            ]
        if gate == 2:
            return [
                self.g2_step_1_publish_gate1_for_gt1,
                self.g2_step_2_publish_gate2_for_gt1,
                self.g2_step_3_publish_gate2_again_for_gt1_should_be_ignored,
                self.g2_step_4_finish,
            ]
        if gate == 3:
            return [
                self.g3_step_1_publish_gate1_for_gt1,
                self.g3_step_2_publish_gate3_for_gt1,
                self.g3_step_3_publish_gate3_again_for_gt1_should_be_ignored,
                self.g3_step_4_finish,
            ]
        if gate == 4:
            return [
                self.g4_step_1_publish_gate1_for_gt2,
                self.g4_step_2_publish_gate4_for_gt2,
                self.g4_step_3_publish_gate4_again_for_gt2_should_be_ignored,
                self.g4_step_4_finish,
            ]
        raise ValueError(f'Unsupported test_gate={gate}')

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

    # ----------------------------
    # Helpers
    # ----------------------------

    def fail(self, reason: str):
        self.test_failed = True
        self.test_fail_reasons.append(reason)
        self.get_logger().error(reason)

    def assert_true(self, condition: bool, message: str):
        if not condition:
            self.fail(message)

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

    def publish_gate1(self, lat: float, lon: float):
        msg = Gate1()
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        self.gate1_pub.publish(msg)
        self.get_logger().info(
            f'Published Gate1 on /{self.ugv_robot}/triage_report/gate1: lat={lat:.8f}, lon={lon:.8f}'
        )

    def publish_gate2(self, category_value: int):
        msg = Gate2()
        msg.category = int(category_value)
        self.gate2_pub.publish(msg)
        self.get_logger().info(
            f'Published Gate2 on /{self.ugv_robot}/triage_report/gate2: category={category_value}'
        )

    def publish_gate3(self):
        msg = Gate3()
        msg.trauma_head = 1
        msg.trauma_torso_back = 0
        msg.trauma_torso_front = 1
        msg.trauma_leg_right = 0
        msg.trauma_leg_left = 1
        msg.trauma_arm_right = 0
        msg.trauma_arm_left = 1
        msg.alertness_ocular = 1
        msg.alertness_verbal = 2
        msg.alertness_motor = 0
        msg.second_pass_category = 3
        self.gate3_pub.publish(msg)
        self.get_logger().info(
            f'Published Gate3 on /{self.ugv_robot}/triage_report/gate3'
        )

    def publish_gate4(self, rr: float, hr: float):
        msg = Gate4()
        msg.rr = float(rr)
        msg.hr = float(hr)
        self.gate4_pub.publish(msg)
        self.get_logger().info(
            f'Published Gate4 on /{self.ugv_robot}/triage_report/gate4: rr={rr}, hr={hr}'
        )

    def finish(self):
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

    # ----------------------------
    # Gate 1 tests
    # ----------------------------

    def g1_step_1_publish_uav_seed(self):
        self.get_logger().info('GATE1 TEST: publish UAV seed for casualty 1')
        self.publish_uav_array([GT_1])

    def g1_step_2_publish_gate1_match_uav(self):
        self.get_logger().info('GATE1 TEST: publish Gate1 near casualty 1')
        self.publish_gate1(lat=GT_1['lat'], lon=GT_1['lon'])

    def g1_step_3_assert_first_match(self):
        logs = self.get_match_log()
        self.assert_true(len(logs) >= 1, 'Expected at least 1 entry in match_log.json')

        last = logs[-1]
        self.assert_true(last['matched_id'] == 1, f'Expected match to ID 1, got {last}')
        self.assert_true(last['is_new'] is False, f'Expected not new, got {last}')
        self.get_logger().info('PASS: Gate1 matched GT casualty 1')

    def g1_step_4_publish_gate1_far_new_id(self):
        self.get_logger().info('GATE1 TEST: publish Gate1 far away to create a new ID')
        self.publish_gate1(lat=39.95000000, lon=-75.21000000)

    def g1_step_5_assert_second_is_new(self):
        logs = self.get_match_log()
        self.assert_true(len(logs) >= 2, 'Expected at least 2 entries in match_log.json')
        last = logs[-1]
        self.assert_true(last['is_new'] is True, f'Expected far report to be new, got {last}')
        self.get_logger().info('PASS: Gate1 created a new ID for far-away report')

    # ----------------------------
    # Gate 2 tests
    # ----------------------------

    def g2_step_1_publish_gate1_for_gt1(self):
        self.get_logger().info('GATE2 TEST: publish Gate1 GPS for GT casualty 1')
        self.publish_gate1(lat=GT_1['lat'], lon=GT_1['lon'])

    def g2_step_2_publish_gate2_for_gt1(self):
        self.get_logger().info('GATE2 TEST: publish Gate2 for casualty 1')
        self.publish_gate2(category_value=2)

    def g2_step_3_publish_gate2_again_for_gt1_should_be_ignored(self):
        self.get_logger().info(
            'GATE2 TEST: publish Gate2 again for same location; node should ignore because casualty 1 is already consumed'
        )
        self.publish_gate1(lat=GT_1['lat'], lon=GT_1['lon'])
        self.publish_gate2(category_value=3)

    def g2_step_4_finish(self):
        self.get_logger().info(
            'Gate2 manual verification point: first submission should go through, second should log no unresolved match or be ignored'
        )
        self.finish()

    # ----------------------------
    # Gate 3 tests
    # ----------------------------

    def g3_step_1_publish_gate1_for_gt1(self):
        self.get_logger().info('GATE3 TEST: publish Gate1 GPS for GT casualty 1')
        self.publish_gate1(lat=GT_1['lat'], lon=GT_1['lon'])

    def g3_step_2_publish_gate3_for_gt1(self):
        self.get_logger().info('GATE3 TEST: publish Gate3 block for casualty 1')
        self.publish_gate3()

    def g3_step_3_publish_gate3_again_for_gt1_should_be_ignored(self):
        self.get_logger().info(
            'GATE3 TEST: publish Gate3 again for same location; node should ignore because casualty 1 is already consumed'
        )
        self.publish_gate1(lat=GT_1['lat'], lon=GT_1['lon'])
        self.publish_gate3()

    def g3_step_4_finish(self):
        self.get_logger().info(
            'Gate3 manual verification point: first submission should go through, second should be ignored'
        )
        self.finish()

    # ----------------------------
    # Gate 4 tests
    # ----------------------------

    def g4_step_1_publish_gate1_for_gt2(self):
        self.get_logger().info('GATE4 TEST: publish Gate1 GPS for GT casualty 2')
        self.publish_gate1(lat=GT_2['lat'], lon=GT_2['lon'])

    def g4_step_2_publish_gate4_for_gt2(self):
        self.get_logger().info('GATE4 TEST: publish Gate4 vitals for casualty 2')
        self.publish_gate4(rr=18.0, hr=88.0)

    def g4_step_3_publish_gate4_again_for_gt2_should_be_ignored(self):
        self.get_logger().info(
            'GATE4 TEST: publish Gate4 again for same location; node should ignore because casualty 2 is already consumed'
        )
        self.publish_gate1(lat=GT_2['lat'], lon=GT_2['lon'])
        self.publish_gate4(rr=20.0, hr=92.0)

    def g4_step_4_finish(self):
        self.get_logger().info(
            'Gate4 manual verification point: first submission should go through, second should be ignored'
        )
        self.finish()


def main(args=None):
    rclpy.init(args=args)
    node = ScorecardIntegrationTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()