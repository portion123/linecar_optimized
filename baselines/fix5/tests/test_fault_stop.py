"""Execute the four real fatal handlers against shared TIM2 registers on host.

Each handler still latches in its original infinite loop. A forked child calls
the production handler; the parent verifies PWM withdrawal and terminates it.
This checks software register writes, not physical ARM exception delivery.
"""
import ctypes as C
import mmap
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(hasattr(os, 'fork'), 'fatal-handler execution requires host fork/shared mmap')
class FaultStopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('gcc'):
            raise unittest.SkipTest('fatal-handler host test requires GCC; not executed is not passed')
        cls.directory = tempfile.TemporaryDirectory(prefix='linecar-fatal-handler-')
        directory = Path(cls.directory.name)
        harness = directory / 'fault_harness.c'
        # Reuse the real TIM register layout and production exception source.
        # Only TIM2's address is replaced, after the real header defines it.
        harness.write_text('''#include <stddef.h>
#include "stm32f10x.h"
static TIM_TypeDef *fault_test_tim2;
#undef TIM2
#define TIM2 fault_test_tim2
#include "''' + str(ROOT / 'user/stm32f10x_it.c') + '''"
void FaultTest_Bind(void *registers) { fault_test_tim2=registers; }
unsigned FaultTest_Size(void) { return (unsigned)sizeof(TIM_TypeDef); }
unsigned FaultTest_Offset(unsigned channel)
{
    switch(channel) {
    case 1: return (unsigned)offsetof(TIM_TypeDef,CCR1);
    case 2: return (unsigned)offsetof(TIM_TypeDef,CCR2);
    case 3: return (unsigned)offsetof(TIM_TypeDef,CCR3);
    default: return (unsigned)offsetof(TIM_TypeDef,CCR4);
    }
}
unsigned FaultTest_Width(void) { return (unsigned)sizeof(fault_test_tim2->CCR3); }
''', encoding='utf-8')
        library = directory / 'fatal_handlers.so'
        subprocess.run(['gcc', '-std=c99', '-O2', '-shared', '-fPIC', '-Wall', '-Wextra',
                        '-Wdouble-promotion', '-ffp-contract=off', '-Werror',
                        '-DUSE_STDPERIPH_DRIVER', '-DSTM32F10X_HD',
                        '-Istart', '-Ilibrary', '-Iuser', '-Ihardware', '-Isystem',
                        str(harness), '-o', str(library)], cwd=ROOT, check=True)
        cls.fw = C.CDLL(str(library))
        cls.fw.FaultTest_Bind.argtypes = [C.c_void_p]
        cls.fw.FaultTest_Size.restype = C.c_uint
        cls.fw.FaultTest_Offset.argtypes = [C.c_uint]
        cls.fw.FaultTest_Offset.restype = C.c_uint
        cls.fw.FaultTest_Width.restype = C.c_uint
        for handler in ('HardFault_Handler', 'MemManage_Handler', 'BusFault_Handler', 'UsageFault_Handler'):
            getattr(cls.fw, handler).argtypes = []
            getattr(cls.fw, handler).restype = None

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def run_handler(self, name):
        size, width = self.fw.FaultTest_Size(), self.fw.FaultTest_Width()
        offsets = {channel: self.fw.FaultTest_Offset(channel) for channel in (1, 2, 3, 4)}
        memory = mmap.mmap(-1, size, flags=mmap.MAP_SHARED,
                           prot=mmap.PROT_READ | mmap.PROT_WRITE)
        memory[:] = bytes([0xA5]) * size
        for channel, value in ((1, 17), (2, 23), (3, 30), (4, 22)):
            memory[offsets[channel]:offsets[channel]+width] = value.to_bytes(width, 'little')
        expected = bytearray(memory[:])
        for channel in (3, 4):
            expected[offsets[channel]:offsets[channel]+width] = bytes(width)
        buffer = (C.c_uint8 * size).from_buffer(memory)
        self.fw.FaultTest_Bind(C.addressof(buffer))
        child = os.fork()
        if child == 0:
            getattr(self.fw, name)()
            os._exit(93)  # A fatal exception handler must never return.
        reaped = False
        try:
            deadline = time.monotonic() + 1.0
            while memory[:] != expected and time.monotonic() < deadline:
                done, status = os.waitpid(child, os.WNOHANG)
                if done:
                    reaped = True
                    self.fail(f'{name} exited instead of remaining latched: wait status {status}')
                time.sleep(0.001)
            self.assertEqual(memory[:], bytes(expected),
                             f'{name} must clear CCR3/CCR4 only before latching')
            # This comparison also checks CCR1/CCR2, reserved words, and every
            # other TIM register remain byte-for-byte unchanged.
            done, status = os.waitpid(child, os.WNOHANG)
            if done:
                reaped = True
                self.fail(f'{name} returned after stopping PWM: wait status {status}')
        finally:
            if not reaped:
                try:
                    os.kill(child, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                os.waitpid(child, 0)
            del buffer
            memory.close()

    def test_hardfault_withdraws_only_motor_pwm_and_stays_latched(self):
        self.run_handler('HardFault_Handler')

    def test_memmanage_withdraws_only_motor_pwm_and_stays_latched(self):
        self.run_handler('MemManage_Handler')

    def test_busfault_withdraws_only_motor_pwm_and_stays_latched(self):
        self.run_handler('BusFault_Handler')

    def test_usagefault_withdraws_only_motor_pwm_and_stays_latched(self):
        self.run_handler('UsageFault_Handler')


if __name__ == '__main__':
    unittest.main(verbosity=2)
