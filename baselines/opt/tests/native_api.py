"""真实 C 测试接口：GCC 原生执行，或 Windows ARMCC+Unicorn 回退。"""
from pathlib import Path
import ctypes as C
import subprocess
import os
import shutil

ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT / 'tests/native/control_native.so'
COMMAND = ['gcc', '-std=c99', '-O2', '-g', '-shared', '-fPIC', '-Wall', '-Wextra',
           '-Werror', '-DUSE_STDPERIPH_DRIVER', '-DSTM32F10X_HD',
           '-Itests/native', '-Istart', '-Ilibrary', '-Iuser', '-Ihardware', '-Isystem',
           'tests/native/harness.c', '-o', str(LIB)]
if shutil.which('gcc') and not os.environ.get('CONTROL_ARM_ELF'):
    subprocess.run(COMMAND, cwd=ROOT, check=True)
    fw = C.CDLL(str(LIB))
else:
    from arm_api import ArmApi
    fw = ArmApi(ROOT)
fw.Gray_Read.restype = C.c_uint8
fw.Track_IsRunning.restype = C.c_uint8
fw.Track_HandleKey.argtypes = [C.c_uint8]
for name in ['LeftTarget', 'RightTarget', 'LeftCommand', 'RightCommand', 'Error',
             'Approach', 'Goal', 'Angle', 'LeftRPM', 'RightRPM', 'FrontOffset', 'AxleTrack', 'RecoveryAngle', 'RecoveryDistance']:
    getattr(fw, 'Native_' + name).restype = C.c_float
fw.Native_Step.argtypes = [C.c_uint32, C.c_int, C.c_int, C.c_uint]
fw.Native_Keys.argtypes = [C.c_uint32, C.c_uint]
fw.Native_Screen.argtypes = [C.c_uint]
fw.Native_Screen.restype = C.c_char_p
fw.Encoder_GetPair.argtypes = [C.POINTER(C.c_int16), C.POINTER(C.c_int16)]
fw.Encoder_Get.restype = C.c_int16

def pwm():
    return fw.Native_LeftPWM(), fw.Native_RightPWM()

def feedback(count=2):
    """只用于状态回归；几何测试会使用独立电机模型产生实际反馈。"""
    return tuple(count if x > 0 else -count if x < 0 else 0 for x in pwm())

def step(tick, mask, counts=None):
    left, right = feedback() if counts is None else counts
    fw.Native_Step(tick & 0xFFFFFFFF, left, right, mask)

def approach(mask=255):
    fw.Native_Start(0x18)
    for t in range(20, 100, 20): step(t, mask, (2, 2))
    step(100, 0, (2, 2)); step(120, 0, (2, 2))
    assert fw.Native_State() == 14
    return 120

def pivot(mask=0x3F):
    tick = approach(mask)
    while fw.Native_State() == 14:
        tick += 20; step(tick, 0, (3, 3))
    assert fw.Native_State() == 11
    return tick
