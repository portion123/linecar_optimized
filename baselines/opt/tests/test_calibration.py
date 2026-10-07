"""实测计数下检查距离/转角，并复现视频相关的断帧、低速及恢复问题。"""
import math
import ctypes as C
import unittest
from native_api import fw, step, pwm, approach, pivot
from track_model import SquarePlant

fw.Native_CountsPerRev.restype = C.c_float
fw.Native_BasePWM.restype = C.c_float
fw.Native_LoadComp.restype = C.c_float


class CalibrationTests(unittest.TestCase):
    def test_01_measured_counts_and_bench_speed(self):
        self.assertAlmostEqual(fw.Native_CountsPerRev(0), 251.0, places=3)
        self.assertAlmostEqual(fw.Native_CountsPerRev(1), 265.35, places=3)
        for left, right, expected in [(2011, 2244, (480.7171, 507.4053)),
                                      (2866, 3130, (685.0996, 707.7445))]:
            fw.Native_Reset(); fw.Track_StartForward()
            step(1000, 0, (left, right))
            self.assertAlmostEqual(fw.Native_LeftRPM(), expected[0], delta=.02)
            self.assertAlmostEqual(fw.Native_RightRPM(), expected[1], delta=.02)

    def test_02_bench_feedforward_matches_supplied_points(self):
        for i, low, high in [(0, 2011, 2866), (1, 2244, 3130)]:
            self.assertAlmostEqual(fw.Native_BasePWM(i, low), 30, delta=.01)
            self.assertAlmostEqual(fw.Native_BasePWM(i, high), 40, delta=.01)

    def test_03_axle_advance_uses_actual_wheel_distance(self):
        tick=approach(0x3F)
        goal=fw.Native_Goal()
        actual=0.0
        distance=(9/251.0+10/265.35)*math.pi*44/2
        for _ in range(50):
            tick+=20; actual+=distance; step(tick,0,(9,10))
            if fw.Native_State()==11: break
            self.assertEqual(fw.Native_State(),14)
            self.assertAlmostEqual(fw.Native_Approach(),actual,delta=.02)
        self.assertEqual(fw.Native_State(),11)
        self.assertGreaterEqual(actual,goal)
        self.assertLess(actual,goal+distance+.01)
        self.assertLess(actual,190)  # 旧1000会让真实推进达到约四倍距离

    def test_04_rotation_uses_both_actual_counts_per_rev(self):
        tick=pivot(0x1F)
        for _ in range(4):
            tick+=20; step(tick,0,(0,0))
        delta=(9/251.0+10/265.35)*math.pi*44/133
        for n in range(1,6):
            tick+=20; step(tick,0,(-9,10))
            self.assertAlmostEqual(fw.Native_Angle(),n*delta,delta=.002)
            self.assertEqual(fw.Native_State(),11)

    def test_05_wide_frames_with_one_missing_read_still_turn(self):
        for gap_frames in (1,2):
            fw.Native_Start(0x18)
            step(20,255,(2,2))
            tick=20
            for _ in range(gap_frames):
                tick+=20; step(tick,0,(2,2))
                self.assertNotEqual(fw.Native_State(),14)
            tick+=20; step(tick,255,(2,2))
            tick+=20; step(tick,0,(2,2))
            tick+=20; step(tick,0,(2,2))
            self.assertEqual(fw.Native_State(),14)

    def test_06_isolated_wide_frames_do_not_accumulate_across_long_gap(self):
        fw.Native_Start(0x18)
        step(20,255,(2,2))
        for tick in (40,60,80): step(tick,0x18,(2,2))
        step(100,255,(2,2))
        step(120,0,(2,2)); step(140,0,(2,2))
        self.assertNotEqual(fw.Native_State(),14)

    def test_07_short_central_wide_patch_does_not_change_line_width(self):
        fw.Native_Start(0x18)
        for tick in range(20,421,20): step(tick,0x3C,(2,2))
        self.assertEqual(fw.Native_LineWidth(),2)
        for tick in range(440,541,20): step(tick,0x3F,(2,2))
        step(560,0,(2,2)); step(580,0,(2,2))
        self.assertEqual(fw.Native_State(),14)

    def test_08_speed_output_cuts_zero_and_excess_speed(self):
        for i in (0,1):
            fw.Native_Reset()
            self.assertEqual(fw.Native_SpeedFrame(i,0,0,20),0)
            fw.Native_SetIntegral(i,20)
            self.assertEqual(fw.Native_SpeedFrame(i,30,200,20),0)
            fw.Native_Reset()
            output=fw.Native_SpeedFrame(i,60,0,20)
            self.assertGreater(output,0)
            self.assertLess(output,18)

    def test_09_start_boost_is_short_and_bounded(self):
        for i in (0,1):
            fw.Native_Reset()
            output=[fw.Native_SpeedFrame(i,60,0,20) for _ in range(20)]
            self.assertTrue(all(0<=v<=30 for v in output))
            # 仍是两帧有限助推，但PWM斜率限制不再允许直接18→22跳变。
            self.assertGreater(output[6],output[4])
            self.assertLessEqual(max(output[:10]),22)
            self.assertTrue(all(b-a<=3 for a,b in zip(output,output[1:])))
            fw.Native_Reset()
            for _ in range(6): fw.Native_SpeedFrame(i,60,0,20)
            self.assertLess(fw.Native_SpeedFrame(i,60,20,20),18)

    def test_10_speed_loop_regulates_two_different_fast_motors(self):
        plant=SquarePlant()
        fw.Native_Start(0x18)
        measured=[]
        for tick in range(20,4001,20):
            counts=plant.advance(*pwm(),dt=.02)
            step(tick,0x18,counts)
            self.assertEqual(fw.Native_State(),1)
            self.assertTrue(all(0<=v<=30 for v in pwm()))
            if tick>=3000: measured.append(plant.rpm[:])
        for wheel in (0,1):
            mean=sum(x[wheel] for x in measured)/len(measured)
            self.assertAlmostEqual(mean,60,delta=5)
        self.assertLess(max(abs(x[0]-x[1]) for x in measured),12)

    def test_11_load_compensation_survives_inner_stop_and_is_bounded(self):
        for i in (0,1):
            fw.Native_Reset()
            fw.Native_SpeedFrame(i,60,50,20)
            fw.Native_SetIntegral(i,5)
            fw.Native_SpeedFrame(i,0,50,20)
            self.assertAlmostEqual(fw.Native_LoadComp(i),5,delta=.02)
            fw.Native_SpeedFrame(i,60,50,20)
            fw.Native_SetIntegral(i,20)
            fw.Native_SpeedFrame(i,0,50,20)
            self.assertEqual(fw.Native_LoadComp(i),12)
            fw.Track_Stop()
            self.assertEqual(fw.Native_LoadComp(i),0)

    def test_12_overspeed_integral_is_not_saved_as_load(self):
        for i in (0,1):
            fw.Native_Reset()
            fw.Native_SpeedFrame(i,60,200,20)
            fw.Native_SetIntegral(i,20)
            fw.Native_SpeedFrame(i,0,200,20)
            self.assertEqual(fw.Native_LoadComp(i),0)


if __name__=='__main__':
    unittest.main(verbosity=2)
