"""关键回归：直接执行本次修改的 C 控制、按键、探头、电机及编码器代码。
电脑中的 GPIO/定时器/中断屏蔽为替身，测试不代表 ARM 固件烧录或实车通过。
"""
import ctypes as C
import math
import unittest
from native_api import fw, step, pwm, feedback, approach, pivot

class ControlTests(unittest.TestCase):
    def test_01_all_256_sensor_masks_and_start(self):
        for mask in range(256):
            fw.Native_Start(mask)
            self.assertEqual(fw.Gray_Read(), mask)
            self.assertEqual(bool(fw.Track_IsRunning()), bool(mask))
            step(20, mask, (5, 5))
            self.assertLessEqual(max(map(abs, pwm())), 65)
        fw.Native_Reset()
        self.assertEqual(fw.Native_GrayPins(), 255)
        self.assertEqual(fw.Native_GrayMode(), 0x48)

    def test_02_physical_left_and_right_steering(self):
        for channel in range(8):
            fw.Native_Start(1 << channel)
            step(20, 1 << channel, (5, 5))
            self.assertEqual(fw.Native_Error(), channel * 2 - 7)
            left, right = fw.Native_LeftTarget(), fw.Native_RightTarget()
            self.assertLess(left, right) if channel < 4 else self.assertGreater(left, right)
            self.assertGreater(left, 0); self.assertGreater(right, 0)

    def test_03_separated_black_patches_not_false_center(self):
        fw.Native_Start(0x81); step(20, 0x81, (5, 5))
        self.assertEqual(abs(fw.Native_Error()), 7)
        self.assertEqual(fw.Native_State(), 1)

    def test_04_all_black_then_white_waits_for_axle(self):
        tick = approach(255)
        goal = fw.Native_Goal()
        front = fw.Native_FrontOffset()
        self.assertGreater(goal, front - 30); self.assertLess(goal, front + 10)
        self.assertEqual(fw.Native_LeftTarget(), fw.Native_RightTarget())
        while fw.Native_Approach() < goal - 4:
            tick += 20; step(tick, 0, (3, 3))
            self.assertEqual(fw.Native_State(), 14)
        while fw.Native_State() == 14:
            tick += 20; step(tick, 0, (3, 3))
        self.assertEqual(fw.Native_State(), 11)
        self.assertEqual(pwm(), (0, 0))

    def test_05_single_sided_corner_direction(self):
        for mask, direction in [(0x1F, -1), (0xF8, 1)]:
            approach(mask); self.assertEqual(fw.Native_Dir(), direction)

    def test_06_crossbar_followed_by_straight_not_corner(self):
        fw.Native_Start(0x18)
        for tick in range(20, 160, 20): step(tick, 255, (2, 2))
        for tick in range(160, 1200, 20):
            step(tick, 0x18, (2, 2)); self.assertEqual(fw.Native_State(), 1)

    def test_07_one_wide_frame_is_not_corner(self):
        fw.Native_Start(0x18); step(20, 255, (2, 2))
        step(40, 0, (2, 2)); step(60, 0, (2, 2))
        self.assertNotEqual(fw.Native_State(), 14)
        step(80, 0, (2, 2)); self.assertNotEqual(fw.Native_State(), 14)
        step(100, 0, (2, 2)); self.assertEqual(fw.Native_State(), 2)

    def test_08_one_missing_frame_does_not_pivot(self):
        fw.Native_Start(0x18)
        for tick in range(20, 200, 20): step(tick, 0x18, (2, 2))
        step(200, 0, (2, 2))
        self.assertNotEqual(fw.Native_State(), 2)
        self.assertGreaterEqual(min(pwm()), 0)
        step(220, 0x18, (2, 2)); self.assertEqual(fw.Native_State(), 1)

    def test_09_no_early_exit_on_old_line_and_signed_angle(self):
        tick = pivot(0x1F)
        for _ in range(8):
            tick += 20; step(tick, 0x18, (0, 0)); self.assertEqual(fw.Native_State(), 11)
        # 正常向左转，达到 55°之前即使中心有黑线，也不能结束转弯。
        while fw.Native_Angle() < math.radians(53):
            tick += 20; step(tick, 0x18, (-2, 2)); self.assertEqual(fw.Native_State(), 11)
        while fw.Native_State() == 11:
            tick += 20; step(tick, 0x18, (-2, 2))
        self.assertEqual(fw.Native_State(), 15)

    def test_10_search_sweeps_both_sides_then_stops(self):
        fw.Native_Start(0x18)
        for tick in range(20, 180, 20): step(tick, 0x18, (2, 2))
        seen_second = False
        for tick in range(180, 12000, 20):
            step(tick, 0)
            seen_second |= fw.Native_Sweep() == 2
            if not fw.Track_IsRunning(): break
        self.assertTrue(seen_second)
        self.assertEqual(fw.Native_State(), 8); self.assertEqual(pwm(), (0, 0))

    def test_11_encoder_fault_stops_not_infinite_boost(self):
        fw.Native_Start(0x18)
        for tick in range(20, 1800, 20):
            step(tick, 0x18, (0, 0))
            if not fw.Track_IsRunning(): break
        self.assertEqual(fw.Native_State(), 9); self.assertEqual(pwm(), (0, 0))

    def test_12_speed_feedback_reduces_fast_wheel_drive(self):
        fw.Native_Start(0x18)
        for tick in range(20, 500, 20): step(tick, 0x18, (6, 2))
        self.assertEqual(fw.Native_LeftTarget(), fw.Native_RightTarget())
        self.assertLess(pwm()[0], pwm()[1])

    def test_13_manual_stop_all_moving_phases(self):
        for key in (1, 2, 3, 5):
            for factory in (lambda: fw.Native_Start(0x18), approach, pivot):
                factory(); fw.Track_HandleKey(key)
                self.assertFalse(fw.Track_IsRunning()); self.assertEqual(pwm(), (0, 0))

    def test_14_forward_mode_motor_polarity_and_no_line_dependency(self):
        fw.Native_Start(0); fw.Track_HandleKey(5)
        self.assertEqual(pwm(), (30, 30))
        self.assertEqual(tuple(fw.Native_Pin(x) for x in (4, 5, 6, 7)), (1, 0, 1, 0))
        for tick, mask in [(20, 0), (40, 255), (60, 1), (121000, 0)]:
            step(tick, mask, (0, 0)); self.assertEqual(fw.Native_State(), 10)
            self.assertEqual(pwm(), (30, 30))
        fw.Track_HandleKey(5); self.assertEqual(pwm(), (0, 0))

    def test_15_nonblocking_button_debounce_and_stop_priority(self):
        fw.Native_Reset()
        for tick, pressed, expected in [(20, 0, 0), (30, 8, 0), (35, 0, 0),
                                       (40, 8, 0), (59, 8, 0), (60, 8, 2),
                                       (500, 8, 0), (510, 12, 0), (530, 12, 3)]:
            self.assertEqual(fw.Native_Keys(tick, pressed), expected)

    def test_16_encoder_phase_decode_and_atomic_pair(self):
        fw.Native_Reset()
        fw.Native_EncoderEdge(0, 0); self.assertEqual(fw.Native_Count(0), -1)
        fw.Native_EncoderEdge(1, 0); self.assertEqual(fw.Native_Count(0), 0)
        fw.Native_EncoderEdge(0, 2); self.assertEqual(fw.Native_Count(0), 0)
        fw.Native_EncoderEdge(14, 0); self.assertEqual(fw.Native_Count(1), -1)
        fw.Native_EncoderEdge(0, 0)
        left, right = C.c_int16(), C.c_int16()
        fw.Native_SetMask(1); fw.Encoder_GetPair(C.byref(left), C.byref(right))
        self.assertEqual((left.value, right.value), (-1, -1))
        self.assertEqual(fw.Native_GetMask(), 1)
        self.assertEqual((fw.Encoder_Get(1), fw.Encoder_Get(2)), (0, 0))

    def test_17_real_clock_parameters_and_oled_bounds(self):
        fw.Native_Reset()
        self.assertEqual(fw.Native_Prescaler(0), 719)
        self.assertEqual(fw.Native_Prescaler(1), 71)
        fw.Native_Start(0x18)
        for tick in range(20, 500, 20): step(tick, 0x18, (2, 2))
        fw.Track_Stop()
        for tick in range(500, 900, 20): step(tick, 0, (2, 2))
        self.assertEqual(fw.Native_OLEDInvalid(), 0)

    def test_18_millisecond_wrap_and_large_gap(self):
        fw.Native_Start(0x18)
        # 长调度空档不能被累计为连续 700ms 无脉冲，更不能误确认一个角点。
        step(0xFFFFFFF0, 255, (0, 0)); self.assertNotIn(fw.Native_State(), (8, 9, 14))
        for i in range(1, 12): step(0xFFFFFFF0 + i * 20, 0x18, (2, 2))
        self.assertEqual(fw.Native_State(), 1)

    def test_19_normal_wide_line_and_far_single_probes(self):
        for centered, shifted in [(0x3C, 0x0F), (0x3C, 0xF0), (0x3E, 0x1F), (0x7C, 0xF8)]:
            fw.Native_Start(centered)
            for tick in range(20, 400, 20):
                step(tick, shifted, (2, 2)); self.assertEqual(fw.Native_State(), 1)
        for mask in (1, 3, 7, 0x80, 0xC0, 0xE0):
            fw.Native_Start(mask)
            for tick in range(20, 200, 20):
                step(tick, mask, (2, 2)); self.assertEqual(fw.Native_State(), 1)

    def test_20_tapered_corner_tail_keeps_direction_and_width(self):
        fw.Native_Start(0x3C)
        for tick in range(20, 100, 20): step(tick, 0xFC, (2, 2))
        for tick in range(100, 340, 20): step(tick, 0xF8 if tick<240 else 0xF0, (2, 2))
        self.assertEqual(fw.Native_LineWidth(), 4)
        self.assertEqual(fw.Native_WideActive(), 1)
        step(340, 0, (2, 2)); step(360, 0, (2, 2))
        self.assertEqual(fw.Native_State(), 14)
        self.assertEqual(fw.Native_Dir(), 1)

if __name__ == '__main__':
    unittest.main(verbosity=2)
