"""真实外设/控制代码的整数边界和输出界限；输入极值是故障场景。"""
import unittest
from native_api import fw, step, pwm

class SafetyBoundsTests(unittest.TestCase):
    def test_encoder_irq_saturates_without_count_polarity_wrap(self):
        fw.Native_Reset()
        for wheel,up,down in ((0,1,0),(1,15,14)):
            fw.Native_SetEncoder(wheel,32767); fw.Native_EncoderEdge(up,0)
            self.assertEqual(fw.Native_Count(wheel),32767)
            fw.Native_SetEncoder(wheel,-32768); fw.Native_EncoderEdge(down,0)
            self.assertEqual(fw.Native_Count(wheel),-32768)

    def test_total_counts_saturate_instead_of_overflow(self):
        fw.Native_Reset(); fw.Native_SetTotals(2147483640,-2147483640)
        step(20,0,(100,-100))
        self.assertEqual(fw.Native_Total(0),2147483647)
        self.assertEqual(fw.Native_Total(1),-2147483648)

    def test_extreme_raw_counts_cannot_escape_pwm_limits(self):
        for counts in ((32767,32767),(-32768,-32768),(32767,-32768),(-32768,32767)):
            fw.Native_Start(0x18)
            for tick in range(20,181,20): step(tick,0x18,counts)
            self.assertLessEqual(max(map(abs,pwm())),30)
        fw.Track_Stop(); self.assertEqual(pwm(),(0,0))

    def test_fault_stop_stays_latched_until_explicit_restart(self):
        fw.Native_Start(0x18)
        for tick in range(20,2001,20): step(tick,0x18,(0,0))
        self.assertEqual(fw.Native_State(),9)
        for tick in range(2020,2601,20): step(tick,0x18,(5,5))
        self.assertEqual(fw.Native_State(),9)
        self.assertEqual(pwm(),(0,0))
        fw.Track_Start(); self.assertEqual(fw.Native_State(),1)

if __name__=='__main__': unittest.main(verbosity=2)
