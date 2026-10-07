"""视频问题的回归：中心旁侧线、连续弯道、宽线尾和短丢线。

直接调用实际 C 控制，输入是刻意构造的八路检测序列。该测试证明状态与
轮速命令的回归行为，不声称知道视频中每路探头的实际电平。
"""
import unittest
import math
from native_api import fw, step, pwm, pivot
from track_model import SquarePlant


def warm(mask=0x18):
    fw.Native_Start(mask)
    for tick in range(20, 420, 20):
        step(tick, mask, (2, 2))
    return 400


def target_pair():
    return fw.Native_LeftTarget(), fw.Native_RightTarget()


class TurnRegressionTests(unittest.TestCase):
    def test_01_persistent_side_branch_overrides_stale_center(self):
        for mask, direction in [(0x19, -1), (0x1B, -1), (0x98, 1), (0xD8, 1)]:
            with self.subTest(mask=hex(mask)):
                tick = warm()
                for _ in range(5):
                    tick += 20
                    step(tick, mask, (2, 2))
                self.assertGreaterEqual(direction * fw.Native_Error(), 5)
                left, right = target_pair()
                self.assertGreater(direction * (left - right), 8)
                self.assertEqual(fw.Native_State(), 1)

    def test_02_single_side_noise_does_not_pull_car_off_center(self):
        for mask in (0x19, 0x98):
            tick = warm()
            step(tick + 20, mask, (2, 2))
            self.assertEqual(fw.Native_Error(), 0)
            self.assertAlmostEqual(*target_pair(), places=5)
            step(tick + 40, 0x18, (2, 2))
            self.assertEqual(fw.Native_Error(), 0)
            self.assertEqual(fw.Native_State(), 1)

    def test_03_symmetric_side_patches_do_not_choose_a_branch(self):
        tick = warm()
        for _ in range(8):
            tick += 20
            step(tick, 0x99, (2, 2))
            self.assertEqual(fw.Native_Error(), 0)
            self.assertAlmostEqual(*target_pair(), places=5)

    def test_04_edge_line_creates_prompt_command_differential(self):
        for mask, direction in [(0x01, -1), (0x80, 1)]:
            tick = warm()
            step(tick + 20, mask, (2, 2))
            left = fw.Native_LeftCommand()
            right = fw.Native_RightCommand()
            # FIX4保留及时正确差速，首帧5RPM以上，同时限制旧版急跳变。
            self.assertGreaterEqual(direction * (left - right), 5.0)
            self.assertEqual(fw.Native_State(), 1)
            self.assertGreaterEqual(min(target_pair()), 0)

    def test_05_central_wide_patch_does_not_arm_axle_advance(self):
        for mask in (0x3C, 0x7E):
            tick = warm()
            for _ in range(5):
                tick += 20
                step(tick, mask, (2, 2))
                self.assertEqual(fw.Native_State(), 1)
                self.assertEqual(fw.Native_WideActive(), 0)
            for _ in range(2):
                tick += 20
                step(tick, 0, (2, 2))
                self.assertNotEqual(fw.Native_State(), 14)

    def test_06_weak_wide_tail_returns_to_bend_control(self):
        for broad, tail, direction in [(0x1F, 0x03, -1), (0xF8, 0xC0, 1)]:
            tick = warm()
            for _ in range(3):
                tick += 20
                step(tick, broad, (2, 2))
            for _ in range(5):
                tick += 20
                step(tick, tail, (2, 2))
            self.assertEqual(fw.Native_State(), 1)
            self.assertEqual(fw.Native_WideActive(), 0)
            left, right = target_pair()
            self.assertGreater(direction * (left - right), 8)
            step(tick + 20, 0, (2, 2))
            step(tick + 40, 0, (2, 2))
            self.assertNotEqual(fw.Native_State(), 14)

    def test_07_strong_corner_tail_keeps_axle_geometry(self):
        for broad, tail, direction in [(0x3F, 0x07, -1), (0xFC, 0xE0, 1)]:
            tick = warm(0x3C)
            for _ in range(4):
                tick += 20
                step(tick, broad, (2, 2))
            for _ in range(8):
                tick += 20
                step(tick, tail, (2, 2))
            step(tick + 20, 0, (2, 2))
            step(tick + 40, 0, (2, 2))
            self.assertEqual(fw.Native_State(), 14)
            self.assertEqual(fw.Native_Dir(), direction)
            self.assertGreater(fw.Native_Goal(), fw.Native_FrontOffset() - 50)

    def test_08_one_center_frame_inside_corner_keeps_memory(self):
        tick = warm()
        for _ in range(4):
            tick += 20
            step(tick, 0xFF, (2, 2))
        tick += 20
        step(tick, 0x18, (2, 2))
        self.assertEqual(fw.Native_WideActive(), 1)
        step(tick + 20, 0, (2, 2))
        step(tick + 40, 0, (2, 2))
        self.assertEqual(fw.Native_State(), 14)

    def test_09_real_straight_after_crossbar_cancels_memory(self):
        tick = warm()
        for _ in range(4):
            tick += 20
            step(tick, 0xFF, (2, 2))
        for _ in range(8):
            tick += 20
            step(tick, 0x18, (2, 2))
        self.assertEqual(fw.Native_State(), 1)
        self.assertEqual(fw.Native_WideActive(), 0)
        self.assertAlmostEqual(*target_pair(), places=5)

    def test_10_short_white_gap_keeps_bend_speed_ratio(self):
        for mask in (0x03, 0xC0):
            tick = warm()
            for _ in range(8):
                tick += 20
                step(tick, mask, (2, 2))
            before = target_pair()
            self.assertGreater(abs(before[0] - before[1]), 8)
            step(tick + 20, 0, (2, 2))
            after = target_pair()
            self.assertGreater(abs(after[0] - after[1]), 5)
            self.assertAlmostEqual(before[0] / before[1], after[0] / after[1], places=4)
            self.assertEqual(fw.Native_State(), 1)
            step(tick + 40, mask, (2, 2))
            self.assertEqual(fw.Native_State(), 1)

    def test_11_persistent_edge_enters_finite_alignment_on_correct_side(self):
        for mask, direction in [(0x03, -1), (0xC0, 1)]:
            tick = warm()
            while fw.Native_State() == 1 and tick < 1000:
                tick += 20
                step(tick, mask, (2, 2))
            self.assertEqual(fw.Native_State(), 2)
            self.assertEqual(fw.Native_Dir(), direction)
            scan_start = tick
            step(scan_start + 20, 0x18, (0, 0))
            self.assertEqual(fw.Native_State(), 2)
            self.assertTrue(any(pwm()))
            step(scan_start + 40, 0x18, (0, 0))
            self.assertEqual(fw.Native_State(),2)
            for frame in range(3,7): step(scan_start+frame*20,0x18,(1,1))
            self.assertEqual(fw.Native_State(),17) # 完整6帧之后才低速ALIGN。
            for frame in range(7,17): step(scan_start+frame*20,0x18,(2,2))
            self.assertEqual(fw.Native_State(),15)

    def test_12_brief_edge_reads_do_not_force_alignment(self):
        tick = warm()
        for _ in range(10):
            for mask in (0x01, 0x01, 0x18, 0x18, 0x18, 0x18):
                tick += 20
                step(tick, mask, (2, 2))
                self.assertEqual(fw.Native_State(), 1)

    def test_13_opposite_edges_reset_alignment_confirmation(self):
        tick = warm()
        for mask in (0x03, 0xC0, 0x03, 0xC0):
            for _ in range(7):
                tick += 20
                step(tick, mask, (2, 2))
                self.assertEqual(fw.Native_State(), 1)

    def test_14_strong_corner_tail_does_not_align_before_axle_arrives(self):
        for broad, tail in [(0x3F, 0x03), (0xFC, 0xC0)]:
            tick = warm()
            for _ in range(4):
                tick += 20
                step(tick, broad, (2, 2))
            for _ in range(25):
                tick += 20
                step(tick, tail, (2, 2))
                self.assertNotIn(fw.Native_State(), (2, 11, 14))
                self.assertEqual(fw.Native_WideActive(), 1)
            step(tick + 20, 0, (2, 2))
            step(tick + 40, 0, (2, 2))
            self.assertEqual(fw.Native_State(), 14)

    def test_15_video_like_low_inner_wheel_cannot_stay_in_run_forever(self):
        # 视频读数模式：外轮约108真实RPM、内轮0RPM（旧1000计数显示约27），右侧探头持续黑。
        # 这是反馈序列复现，不等同于测量视频所有控制周期。
        tick = warm()
        for _ in range(32):
            tick += 20
            step(tick, 0xC0, (9, 0))
            if fw.Native_State() != 1:
                break
        self.assertEqual(fw.Native_State(), 2)
        self.assertEqual(fw.Native_Dir(), 1)

    def test_16_single_center_frame_does_not_erase_recent_turn_direction(self):
        for mask, direction in [(0x04, -1), (0x20, 1)]:
            tick = warm()
            for _ in range(7):
                tick += 20
                step(tick, mask, (2, 2))
            tick += 20
            step(tick, 0x18, (2, 2))
            self.assertEqual(fw.Native_Error(), 0)
            for _ in range(4):
                tick += 20
                step(tick, 0, (2, 2))
            self.assertEqual(fw.Native_State(), 2)
            self.assertEqual(fw.Native_Dir(), direction)

    def test_17_next_corner_rearms_on_stable_off_center_narrow_line(self):
        for mask in (0x04, 0x20):
            tick = pivot(0x1F)
            for _ in range(200):
                tick+=20
                step(tick,0x18,(-2,2) if fw.Native_State() in (11,16) else (3,3))
                if fw.Native_State()==1: break
            self.assertEqual(fw.Native_State(),1)
            for _ in range(40):
                tick += 20
                step(tick, mask, (3, 3))
                self.assertEqual(fw.Native_State(), 1)
            for _ in range(4):
                tick += 20
                step(tick, 0xFF, (2, 2))
            step(tick + 20, 0, (2, 2))
            step(tick + 40, 0, (2, 2))
            self.assertEqual(fw.Native_State(), 14)

    def test_18_single_wide_frame_resets_old_edge_timeout(self):
        for edge, wide in [(0x03, 0xFF), (0xC0, 0xFF), (0x03, 0x0F), (0xC0, 0xF0)]:
            tick = warm()
            for _ in range(15):
                tick += 20
                step(tick, edge, (2, 2))
                self.assertEqual(fw.Native_State(), 1)
            tick += 20
            step(tick, wide, (2, 2))
            for frame in range(14):
                tick += 20
                step(tick, edge, (2, 2))
                self.assertIn(fw.Native_State(), (1,12)) if frame<2 else self.assertEqual(fw.Native_State(),1)

    def test_19_opposite_center_single_probe_does_not_replace_turn_hint(self):
        for bend, center_noise, direction in [(0x04, 0x10, -1), (0x20, 0x08, 1)]:
            tick = warm()
            for _ in range(7):
                tick += 20
                step(tick, bend, (2, 2))
            tick += 20
            step(tick, center_noise, (2, 2))
            for _ in range(4):
                tick += 20
                step(tick, 0, (2, 2))
            self.assertEqual(fw.Native_State(), 2)
            self.assertEqual(fw.Native_Dir(), direction)

    def test_20_four_probe_patch_lost_keeps_differential_and_never_long_advances(self):
        for mask, direction in [(0x0F, -1), (0xF0, 1)]:
            tick = warm()
            for _ in range(3):
                tick += 20
                step(tick, mask, (2, 2))
            before = target_pair()
            self.assertGreater(direction * (before[0] - before[1]), 8)
            for _ in range(2):
                tick += 20
                step(tick, 0, (2, 2))
                self.assertIn(fw.Native_State(), (1,12))
                after = target_pair()
                self.assertGreater(direction * (after[0] - after[1]), 5)
                self.assertAlmostEqual(before[0] / before[1], after[0] / after[1], places=4)
            step(tick + 20, 0, (2, 2))
            self.assertIn(fw.Native_State(), (1,12))
            step(tick + 40, 0, (2, 2))
            self.assertEqual(fw.Native_State(), 2)
            self.assertEqual(fw.Native_Dir(), direction)

    def test_21_real_motor_model_scans_both_sides_of_missing_track_then_stops(self):
        # 八路始终全白，但编码器脉冲来自独立电机动力学模型。
        # 44mm轮径下8秒旧超时略短；10秒应允许先后完成左右60度扫描。
        for initial_mask, direction in [(0x04, -1), (0x20, 1)]:
            plant = SquarePlant()
            fw.Native_Start(initial_mask)
            scan_heading = None
            first_sweep_yaw = None
            for tick in range(20, 11000, 20):
                counts = plant.advance(*pwm(), dt=.02)
                step(tick, 0, counts)
                if fw.Native_State() == 2 and scan_heading is None:
                    scan_heading = plant.heading
                if fw.Native_Sweep() == 2 and first_sweep_yaw is None:
                    first_sweep_yaw = plant.heading - scan_heading
                if not fw.Track_IsRunning():
                    break
            self.assertIsNotNone(first_sweep_yaw)
            self.assertGreaterEqual(-direction * math.degrees(first_sweep_yaw), 59.5)
            self.assertGreaterEqual(direction * math.degrees(plant.heading - scan_heading), 59.5)
            self.assertEqual(fw.Native_State(), 8)
            self.assertEqual(pwm(), (0, 0))
            self.assertLess(tick, 10500)


if __name__ == '__main__':
    unittest.main(verbosity=2)
