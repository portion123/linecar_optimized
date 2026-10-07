"""真实 C 测试接口：GCC 原生执行，或 Windows ARMCC+Unicorn 回退。"""
from pathlib import Path
import ctypes as C
import subprocess
import os
import shutil
import atexit
import gzip
import json

ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT / ('tests/native/control_' + os.environ.get('CONTROL_BUILD_TAG', 'native') + '.so')
COMMAND = ['gcc', '-std=c99', '-O2', '-g', '-shared', '-fPIC', '-Wall', '-Wextra',
           '-Wdouble-promotion', '-ffp-contract=off', '-Werror', '-DUSE_STDPERIPH_DRIVER', '-DSTM32F10X_HD',
           '-Itests/native', '-Istart', '-Ilibrary', '-Iuser', '-Ihardware', '-Isystem',
           'tests/native/harness.c', '-o', str(LIB)]
if os.environ.get('CONTROL_CONFIG_HEADER'):
    COMMAND[1:1] = ['-include', os.environ['CONTROL_CONFIG_HEADER']]
if os.environ.get('CONTROL_SANITIZE'):
    COMMAND[1:1] = ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
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
             'Approach', 'Goal', 'Angle', 'LeftRPM', 'RightRPM', 'FrontOffset', 'AxleTrack', 'RecoveryAngle', 'RecoveryDistance',
             'ObservedError', 'LineFilteredError']:
    getattr(fw, 'Native_' + name).restype = C.c_float
fw.Native_Step.argtypes = [C.c_uint32, C.c_int, C.c_int, C.c_uint]
fw.Native_Keys.argtypes = [C.c_uint32, C.c_uint]
fw.Native_Screen.argtypes = [C.c_uint]
fw.Native_Screen.restype = C.c_char_p
fw.Encoder_GetPair.argtypes = [C.POINTER(C.c_int16), C.POINTER(C.c_int16)]
fw.Encoder_Get.restype = C.c_int16

# Optional observer: every existing controller scenario can be captured without
# changing C control behaviour. Floating fields use float.hex() and compare
# exactly for the same GCC/flags. Trace indexes, ticks and wall time are omitted.
_capture = None
_scenario = 'unspecified'
if os.environ.get('CONTROL_CAPTURE'):
    _capture_path = Path(os.environ['CONTROL_CAPTURE'])
    _capture_path.parent.mkdir(parents=True, exist_ok=True)
    _capture = gzip.open(_capture_path, 'wt', encoding='utf-8', compresslevel=6)
    atexit.register(_capture.close)

def set_scenario(name):
    global _scenario
    _scenario = name

def control_fields(api):
    integers = {
        'state': api.Native_State(), 'mode': api.Native_Mode(),
        'sensor_raw': api.Native_RawSensors(), 'sensor_interpreted': api.Native_SensorMask(),
        'sensor_count': api.Native_SensorCount(), 'line_width': api.Native_LineWidth(),
        'left_pwm': api.Native_LeftPWM(), 'right_pwm': api.Native_RightPWM(),
        'corner_dir': api.Native_Dir(), 'turn_hint': api.Native_Hint(),
        'corner_confidence': api.Native_Confidence(), 'sweep': api.Native_Sweep(),
        'wide_active': api.Native_WideActive(), 'wide_ms': api.Native_WideMS(),
        'lost_ms': api.Native_LostMS(), 'recovery_active': api.Native_RecoveryActive(),
        'recovery_attempts': api.Native_RecoveryAttempts(), 'stop_reason': api.Native_StopReason(),
    }
    floats = {
        'line_error': api.Native_Error(), 'observed_error': api.Native_ObservedError(),
        'line_filtered_error': api.Native_LineFilteredError(),
        'left_requested_rpm': api.Native_LeftTarget(), 'right_requested_rpm': api.Native_RightTarget(),
        'left_command_rpm': api.Native_LeftCommand(), 'right_command_rpm': api.Native_RightCommand(),
        'left_actual_rpm': api.Native_LeftRPM(), 'right_actual_rpm': api.Native_RightRPM(),
        'turn_angle_rad': api.Native_Angle(), 'approach_mm': api.Native_Approach(),
        'approach_goal_mm': api.Native_Goal(), 'recovery_angle_rad': api.Native_RecoveryAngle(),
        'recovery_distance_mm': api.Native_RecoveryDistance(),
    }
    return dict(integers, **{key: float(value).hex() for key, value in floats.items()})

if _capture:
    class _ObservedFunction:
        def __init__(self, function, name, api):
            object.__setattr__(self, '_function', function)
            object.__setattr__(self, '_name', name)
            object.__setattr__(self, '_api', api)
        def __getattr__(self, name):
            return getattr(self._function, name)
        def __setattr__(self, name, value):
            setattr(self._function, name, value)
        def __call__(self, *args):
            result = self._function(*args)
            record = {'scenario': _scenario, 'call': self._name, 'control': control_fields(self._api)}
            # Speed-loop and fault-window direct tests also have meaningful
            # returned PWM/fault outputs independent of the Track state.
            if self._name in ('Native_SpeedFrame', 'Native_FaultFrame'):
                record['return'] = int(result)
            _capture.write(json.dumps(record, sort_keys=True, separators=(',', ':')) + '\n')
            return result

    class _Observer:
        observed = {'Native_Reset', 'Native_Start', 'Native_Step', 'Native_Idle',
                    'Native_SpeedFrame', 'Native_FaultFrame', 'Native_RestartScan',
                    'Track_Init', 'Track_Start', 'Track_StartForward', 'Track_StartSpeedTest',
                    'Track_Stop', 'Track_HandleKey', 'Track_Task'}
        def __init__(self, api):
            self.api, self.functions = api, {}
        def __getattr__(self, name):
            if name not in self.observed:
                return getattr(self.api, name)
            if name not in self.functions:
                self.functions[name] = _ObservedFunction(getattr(self.api, name), name, self.api)
            return self.functions[name]
    fw = _Observer(fw)

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
