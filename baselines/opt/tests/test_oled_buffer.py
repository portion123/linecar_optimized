"""真实OLED驱动的有界分片、内存边界和INT32_MIN。"""
import ctypes as C
from pathlib import Path
import subprocess
import shutil
import unittest

ROOT=Path(__file__).resolve().parents[1]
LIB=ROOT/'tests/native/oled_native.so'
if not shutil.which('gcc'):
    raise unittest.SkipTest('真实OLED host测试需要GCC；未执行不能算通过')
subprocess.run(['gcc','-std=c99','-O2','-shared','-fPIC','-Wall','-Wextra','-Werror',
                '-Wno-missing-braces', # 原字库的二维扁平初始化，未改字形数据。
                '-DUSE_STDPERIPH_DRIVER','-DSTM32F10X_HD','-Istart','-Ilibrary','-Iuser','-Ihardware',
                'tests/native/oled_harness.c','-o',str(LIB)],cwd=ROOT,check=True)
oled=C.CDLL(str(LIB)); oled.OLED_ShowString.argtypes=[C.c_uint8,C.c_uint8,C.c_char_p]
oled.OLED_ShowSignedNum.argtypes=[C.c_uint8,C.c_uint8,C.c_int32,C.c_uint8]

class OLEDBufferTests(unittest.TestCase):
    def test_show_functions_only_touch_memory(self):
        oled.RealOLED_Reset()
        oled.OLED_ShowString(1,1,b'TRACK H+1 D-1')
        self.assertEqual(oled.RealOLED_Writes(),0)
        self.assertTrue(any(oled.RealOLED_Pixel(0,i) for i in range(128)))

    def test_one_slice_sends_one_eight_byte_tile(self):
        oled.RealOLED_Reset()
        self.assertEqual(oled.RealOLED_Dirty(),128)
        oled.OLED_Service(8)
        self.assertEqual(oled.RealOLED_Dirty(),127)
        self.assertGreater(oled.RealOLED_Writes(),0)
        self.assertLess(oled.RealOLED_Writes(),650)
        oled.RealOLED_ClearWrites(); oled.OLED_Service(255)
        self.assertEqual(oled.RealOLED_Dirty(),126)
        self.assertLess(oled.RealOLED_Writes(),650)

    def test_insufficient_budget_and_clean_buffer_send_nothing(self):
        oled.RealOLED_Reset(); oled.OLED_Service(7)
        self.assertEqual(oled.RealOLED_Writes(),0)
        for _ in range(128): oled.OLED_Service(8)
        self.assertEqual(oled.RealOLED_Dirty(),0)
        oled.RealOLED_ClearWrites(); oled.OLED_Service(8)
        self.assertEqual(oled.RealOLED_Writes(),0)

    def test_invalid_positions_and_negative_char_are_bounded(self):
        oled.RealOLED_Reset()
        for line,col in ((0,1),(1,0),(5,1),(1,17),(255,255)):
            oled.OLED_ShowChar(line,col,255)
        self.assertFalse(any(oled.RealOLED_Pixel(p,i) for p in range(8) for i in range(128)))
        oled.OLED_ShowChar(1,1,255)
        self.assertTrue(any(oled.RealOLED_Pixel(0,i) for i in range(8)))

    def test_int32_min_render_does_not_overflow(self):
        oled.RealOLED_Reset(); oled.OLED_ShowSignedNum(1,1,-2147483648,10)
        self.assertEqual(oled.RealOLED_Writes(),0)
        self.assertTrue(any(oled.RealOLED_Pixel(0,i) for i in range(88)))

    def test_long_strings_and_invalid_number_lengths_have_bounded_work(self):
        oled.RealOLED_Reset()
        oled.OLED_ShowString(1,1,b'A'*300)
        self.assertEqual(oled.RealOLED_Writes(),0)
        self.assertFalse(any(oled.RealOLED_Pixel(2,i) for i in range(128)))
        oled.RealOLED_Reset(); oled.OLED_ShowString(1,1,None)
        for function in (oled.OLED_ShowNum,oled.OLED_ShowSignedNum,oled.OLED_ShowHexNum,oled.OLED_ShowBinNum):
            for length in (0,32,255): function(1,1,123,length)
        self.assertFalse(any(oled.RealOLED_Pixel(p,i) for p in range(8) for i in range(128)))

if __name__=='__main__': unittest.main(verbosity=2)
