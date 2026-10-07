"""低速和方向故障：真实C速度环/故障逻辑，非实车摩擦标定。"""
import unittest
from native_api import fw


class LowSpeedTests(unittest.TestCase):
    def test_nonzero_low_rpm_is_not_silently_stopped(self):
        for wheel in (0, 1):
            for sign in (-1, 1):
                fw.Native_Reset()
                values = [fw.Native_SpeedFrame(wheel, sign*5, sign*5, 20) for _ in range(10)]
                self.assertTrue(all(sign*v > 0 for v in values))
                self.assertEqual(fw.Native_SpeedFrame(wheel, 0, 0, 20), 0)

    def test_positive_and_negative_speed_path_is_symmetric(self):
        for wheel in (0, 1):
            traces=[]
            for sign in (-1, 1):
                fw.Native_Reset()
                traces.append([sign*fw.Native_SpeedFrame(wheel,sign*18,0,20) for _ in range(20)])
            self.assertEqual(*traces)

    def test_sparse_reverse_counts_eventually_fault(self):
        for wheel in (0, 1):
            for sign in (-1, 1):
                fw.Native_Reset()
                faults=[fw.Native_FaultFrame(wheel,-sign*(i%5!=4),sign*5,sign*9) for i in range(50)]
                self.assertTrue(any(faults))
                self.assertFalse(any(faults[:13]))

    def test_normal_reversal_coast_does_not_fault(self):
        for wheel in (0, 1):
            fw.Native_Reset()
            for _ in range(20):
                self.assertFalse(fw.Native_FaultFrame(wheel,2,30,12))
            for _ in range(10):
                self.assertFalse(fw.Native_FaultFrame(wheel,1,-18,-9))
            for _ in range(40):
                self.assertFalse(fw.Native_FaultFrame(wheel,-1,-18,-9))

    def test_zero_counts_preserve_reverse_evidence(self):
        fw.Native_Reset()
        for _ in range(13): fw.Native_FaultFrame(0,0,5,9)
        results=[fw.Native_FaultFrame(0,-1 if i%2==0 else 0,5,9) for i in range(25)]
        self.assertTrue(any(results))

    def test_reversal_does_not_inherit_other_direction_integral(self):
        for wheel in (0,1):
            fw.Native_Reset()
            for _ in range(10): fw.Native_SpeedFrame(wheel,30,20,20)
            fw.Native_SetIntegral(wheel,10)
            fw.Native_SpeedFrame(wheel,-18,0,20)
            outputs=[abs(fw.Native_SpeedFrame(wheel,-18,-18,20)) for _ in range(10)]
            self.assertLess(max(outputs),12)

    def test_reversal_grace_starts_after_actual_pwm_direction_change(self):
        fw.Native_Reset()
        for _ in range(20): self.assertFalse(fw.Native_FaultFrame(0,2,30,12))
        for _ in range(15): self.assertFalse(fw.Native_FaultFrame(0,1,-18,0))
        for _ in range(10): self.assertFalse(fw.Native_FaultFrame(0,1,-18,-9))
        for _ in range(40): self.assertFalse(fw.Native_FaultFrame(0,-1,-18,-9))


if __name__ == '__main__': unittest.main(verbosity=2)
