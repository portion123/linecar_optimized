"""P1～P5 的独立行为检查，执行真实 Track.c，使用构造的反馈与探头序列。

这些测试检查控制逻辑与 OLED；反馈序列不代表实车摩擦、惯性或起转门槛。
不修改原有回归断言，也不要求新增控制变量的测试读取接口。
"""
import ctypes as C
import math
import unittest

from native_api import fw, step, pwm, approach, pivot


fw.Native_LoadComp.restype = C.c_float


class FaultDetailTests(unittest.TestCase):
    def tearDown(self):
        self.assertEqual(fw.Native_OLEDInvalid(), 0, "OLED 写入不得越界")

    def refresh(self, tick, mask=0, counts=(0, 0)):
        """OLED 分四行刷新，继续调度到第一行明确反映当前状态。"""
        for _ in range(14):
            tick += 20
            step(tick, mask, counts)
        return tick

    def stopped(self, tick, state, detail):
        self.assertEqual(fw.Native_State(), state)
        self.assertFalse(fw.Track_IsRunning())
        self.assertEqual(pwm(), (0, 0), "真故障必须撤去两轮 PWM")
        tick = self.refresh(tick)
        prefix = "ENC" if state == 9 else "LOST"
        line = fw.Native_Screen(0).decode("ascii")
        self.assertEqual(line.rstrip(), f"{prefix} STOP D:{detail:02d}")
        self.assertNotIn("D:+", line)
        self.assertNotIn("D:-", line)
        self.assertEqual(len(line), 16)
        return tick

    def search(self):
        fw.Native_Start(0x18)
        tick = 0
        for _ in range(10):
            tick += 20
            step(tick, 0x18, (2, 2))
        for _ in range(4):
            tick += 20
            step(tick, 0, (2, 2))
        self.assertEqual(fw.Native_State(), 2)
        self.assertAlmostEqual(fw.Native_Angle(), 0, places=6)
        # 80 ms 刹停等待不应参与居中确认。
        for _ in range(4):
            tick += 20
            step(tick, 0, (0, 0))
        self.assertEqual(fw.Native_State(), 2)
        return tick

    def edge_align(self):
        fw.Native_Start(0x18)
        tick = 0
        while fw.Native_State() == 1 and tick < 1200:
            tick += 20
            step(tick, 0x03, (2, 2))
        self.assertEqual(fw.Native_State(), 2)
        self.assertEqual(fw.Native_Dir(), -1)
        return tick

    def test_01_stalled_wheels_identify_left_and_right_and_restart_clears_display(self):
        for wheel, counts, detail in [(0, (0, 2), 1), (1, (2, 0), 2)]:
            with self.subTest(wheel=wheel):
                fw.Native_Start(0x18)
                reached_ceiling = False
                tick = 0
                for tick in range(20, 1801, 20):
                    step(tick, 0x18, counts)
                    reached_ceiling |= abs(pwm()[wheel]) == 40
                    if not fw.Track_IsRunning():
                        break
                self.assertTrue(reached_ceiling, "700 ms 停车前应已尝试 40% 起转")
                tick = self.stopped(tick, 9, detail)
                # 直接重启同一固件实例，不能借助 Native_Reset 清除旧诊断。
                fw.Native_Sensors(0x18)
                fw.Track_Start()
                self.assertEqual(fw.Native_State(), 1)
                tick = self.refresh(tick, 0x18, (2, 2))
                self.assertEqual(fw.Native_Screen(0).decode("ascii").rstrip(), "RUN  K1:STOP")
                self.assertNotIn("D:", fw.Native_Screen(0).decode("ascii"))

    def test_02_reverse_wheels_identify_left_and_right(self):
        for counts, detail in [((-2, 2), 3), ((2, -2), 4)]:
            with self.subTest(detail=detail):
                fw.Native_Start(0x18)
                tick = 0
                for tick in range(20, 1801, 20):
                    step(tick, 0x18, counts)
                    if not fw.Track_IsRunning():
                        break
                self.stopped(tick, 9, detail)

    def test_03_manual_key5_keeps_fixed_30_with_no_pulses(self):
        fw.Native_Start(0)
        fw.Track_HandleKey(5)
        for tick in range(20, 2501, 20):
            step(tick, 0 if tick % 40 else 0xFF, (0, 0))
            self.assertEqual(fw.Native_State(), 10)
            self.assertEqual(pwm(), (30, 30))
        step(121000, 0, (0, 0))
        self.assertEqual(fw.Native_State(), 10)
        self.assertEqual(pwm(), (30, 30))
        self.assertNotIn("D:", fw.Native_Screen(0).decode("ascii"))
        fw.Track_HandleKey(5)
        self.assertEqual(pwm(), (0, 0))

    def test_04_feedback_kick_can_rearm_after_one_moving_frame(self):
        for wheel in (0, 1):
            with self.subTest(wheel=wheel):
                fw.Native_Reset()
                for _ in range(3):
                    outputs = [fw.Native_SpeedFrame(wheel, 20, 0, 20) for _ in range(14)]
                    self.assertLess(outputs[1], 18)
                    self.assertEqual(outputs[2:], list(range(18, 41, 2)))
                    self.assertLess(fw.Native_SpeedFrame(wheel, 20, 20, 20), 18)
                self.assertAlmostEqual(fw.Native_LoadComp(wheel), 0, places=6)
                # 目标低于 10 RPM 后必须复位；再次起转重新从 18% 开始。
                self.assertEqual(fw.Native_SpeedFrame(wheel, 9, 0, 20), 0)
                outputs = [fw.Native_SpeedFrame(wheel, 20, 0, 20) for _ in range(3)]
                self.assertLess(outputs[1], 18)
                self.assertEqual(outputs[2], 18)

    def test_05_kick_releases_at_motion_without_preloading_integral(self):
        for wheel in (0, 1):
            with self.subTest(wheel=wheel):
                fw.Native_Reset()
                for _ in range(14):
                    out = fw.Native_SpeedFrame(wheel, 20, 0, 20)
                self.assertEqual(out, 40, "PID upper 必须容许助推超过闭环 30% 上限")
                # 8 RPM 的第一帧立即撤销起转下限，不等待 100 ms 重武装。
                released = fw.Native_SpeedFrame(wheel, 20, 8, 20)
                self.assertLess(released, 18)
                # 仅自然 PI 积分约 1%：灌入起转时的 40% 会明显超过此上界。
                regulated = fw.Native_SpeedFrame(wheel, 20, 20, 20)
                self.assertLessEqual(regulated, 10)
                self.assertAlmostEqual(fw.Native_LoadComp(wheel), 0, places=6)

    def test_06_approach_yaw_feedback_has_correct_sign_and_limit(self):
        for counts, sign in [((0, 8), 1), ((8, 0), -1)]:
            with self.subTest(sign=sign):
                tick = approach(0x3F)
                self.assertAlmostEqual(fw.Native_Angle(), 0, places=6)
                tick += 20
                step(tick, 0, counts)
                angle = (counts[1] / 265.35 - counts[0] / 251.0) * math.pi * 44 / 133
                self.assertAlmostEqual(fw.Native_Angle(), angle, delta=0.00001)
                self.assertAlmostEqual(fw.Native_LeftTarget(), 50 + 40 * angle, delta=0.0001)
                self.assertAlmostEqual(fw.Native_RightTarget(), 50 - 40 * angle, delta=0.0001)
                self.assertGreater(sign * (fw.Native_LeftTarget() - fw.Native_RightTarget()), 0)
                for _ in range(10):
                    tick += 20
                    step(tick, 0, counts)
                self.assertEqual(fw.Native_State(), 14)
                self.assertAlmostEqual(fw.Native_LeftTarget(), 50 + sign * 10, places=4)
                self.assertAlmostEqual(fw.Native_RightTarget(), 50 - sign * 10, places=4)

    def test_07_corner_scan_preserves_approach_yaw_and_uses_original_heading(self):
        tick = approach(0x3F)
        expected = 0.0
        delta = (12 / 265.35 - 4 / 251.0) * math.pi * 44 / 133
        while fw.Native_State() == 14 and tick < 2000:
            tick += 20
            expected += delta
            step(tick, 0, (4, 12))
        self.assertEqual(fw.Native_State(), 11)
        self.assertGreater(expected, math.radians(55))
        self.assertLess(expected, math.radians(105))
        self.assertAlmostEqual(fw.Native_Angle(), expected, delta=0.0001)
        for _ in range(4):
            tick += 20
            step(tick, 0, (0, 0))
            self.assertEqual(fw.Native_State(), 11)
        # 相对原直线已超过 55°，旋转阶段不必再重复转 55°。
        for n in range(2):
            tick += 20
            step(tick, 0x18, (0, 0))
            self.assertEqual(fw.Native_State(), 11 if n == 0 else 15)
        tick += 80
        step(tick, 0x18, (0, 0))
        self.assertEqual(fw.Native_State(), 1)
        for _ in range(4):
            tick += 20
            step(tick, 0, (2, 2))
        self.assertEqual(fw.Native_State(), 2)
        self.assertAlmostEqual(fw.Native_Angle(), 0, places=6, msg="普通 SEARCH 仍以自身开始方向为基准")

    def test_08_scan_requires_strict_first_frame_then_accepts_near_center(self):
        tick = self.search()
        for mask in (0x20, 0x20, 0x18):
            tick += 20
            step(tick, mask, (0, 0))
            self.assertEqual(fw.Native_State(), 2)
        tick += 20
        step(tick, 0x20, (0, 0))
        self.assertEqual(fw.Native_State(), 1)

    def test_09_scan_accepts_one_missing_frame(self):
        tick = self.search()
        for mask in (0x18, 0):
            tick += 20
            step(tick, mask, (0, 0))
            self.assertEqual(fw.Native_State(), 2)
        tick += 20
        step(tick, 0x20, (0, 0))
        self.assertEqual(fw.Native_State(), 1)

    def test_10_scan_rejects_two_missing_frames(self):
        tick = self.search()
        for mask in (0x18, 0, 0, 0x20, 0x20, 0x18):
            tick += 20
            step(tick, mask, (0, 0))
            self.assertEqual(fw.Native_State(), 2)
        tick += 20
        step(tick, 0x20, (0, 0))
        self.assertEqual(fw.Native_State(), 1)

    def test_11_scan_rejects_outer_and_separated_masks(self):
        for bad_mask in (0x01, 0x80, 0x24, 0x81):
            with self.subTest(mask=hex(bad_mask)):
                tick = self.search()
                for mask in (bad_mask, bad_mask, 0x18, bad_mask, bad_mask, 0x20, 0x20):
                    tick += 20
                    step(tick, mask, (0, 0))
                    self.assertEqual(fw.Native_State(), 2)

    def test_12_scan_reversal_drops_partial_center_confirmation(self):
        tick = self.search()
        tick += 20
        step(tick, 0x18, (0, 0))
        self.assertEqual(fw.Native_State(), 2)
        # 直接提供达到第一侧 60° 上限的脉冲；此处是角度逻辑测试。
        tick += 20
        step(tick, 0, (-130, 130))
        self.assertEqual(fw.Native_Sweep(), 2)
        self.assertEqual(fw.Native_State(), 2)
        tick += 20
        step(tick, 0x20, (0, 0))
        self.assertEqual(fw.Native_State(), 2)
        self.assertTrue(fw.Track_IsRunning())

    def test_13_approach_corner_and_search_timeouts_have_distinct_details(self):
        for setup, timeout, detail in [(approach, 6000, 5), (pivot, 12000, 6), (self.search, 10000, 8)]:
            with self.subTest(detail=detail):
                tick = setup()
                # 长调度空档不累计编码器故障，用于独立触发阶段超时。
                tick += timeout
                step(tick, 0, (0, 0))
                self.stopped(tick, 8, detail)

    def test_14_both_scan_sides_exhausted_have_distinct_details(self):
        for setup, detail in [(pivot, 7), (self.search, 9)]:
            with self.subTest(detail=detail):
                tick = setup()
                second_seen = False
                for _ in range(450):
                    tick += 20
                    # 依据实际 PWM 符号产生脉冲，不复制扫描状态机。
                    counts = tuple(2 if v > 0 else -2 if v < 0 else 0 for v in pwm())
                    step(tick, 0, counts)
                    second_seen |= fw.Native_Sweep() == 2
                    if not fw.Track_IsRunning():
                        break
                self.assertTrue(second_seen)
                self.stopped(tick, 8, detail)

    def test_15_edge_alignment_angle_and_timeout_have_distinct_details(self):
        tick = self.edge_align()
        for _ in range(100):
            tick += 20
            step(tick, 0x03, (0, 3))
            if not fw.Track_IsRunning():
                break
        self.stopped(tick, 8, 10)
        tick = self.edge_align()
        tick += 2500
        step(tick, 0x03, (0, 0))
        self.stopped(tick, 8, 11)


if __name__ == "__main__":
    unittest.main(verbosity=2)
