"""Windows 回退：ARMCC 编译真实 C harness，Unicorn 执行 Cortex-M3 指令。

GPIO/编码器/显示仍由 harness 提供替身，不能代替烧录和实车验证。
只依赖 ARMCC 5、Python 和 Unicorn；ELF 读取使用 Python 标准库。
"""
from pathlib import Path
import ctypes as C
import os
import struct
import subprocess


def build(root):
    toolchain = Path(os.environ.get('ARMCC_BIN', r'C:\Keil_v5\ARM\ARMCC\bin'))
    obj = root / 'tests/native/control_arm.o'
    elf = root / 'tests/native/control_arm.axf'
    subprocess.run([str(toolchain / 'armcc.exe'), '--cpu', 'Cortex-M3', '--c99',
                    '-O1', '--debug', '-DUSE_STDPERIPH_DRIVER', '-DSTM32F10X_HD',
                    '-Itests/native', '-Istart', '-Ilibrary', '-Iuser',
                    '-Ihardware', '-Isystem', '-c', 'tests/native/harness.c',
                    '-o', str(obj)], cwd=root, check=True)
    subprocess.run([str(toolchain / 'armlink.exe'), '--cpu', 'Cortex-M3',
                    '--entry', 'Native_Reset', '--no_remove',
                    '--ro-base', '0x08000000', '--rw-base', '0x20000000',
                    '-o', str(elf), str(obj)], cwd=root, check=True)
    return elf


def read_elf(path):
    data = path.read_bytes()
    if data[:7] != b'\x7fELF\x01\x01\x01':
        raise ValueError('需要 32 位小端 ARM ELF')
    header = struct.unpack_from('<16sHHIIIIIHHHHHH', data)
    phoff, shoff = header[5:7]
    phsize, phnum, shsize, shnum = header[9:13]
    segments = []
    for index in range(phnum):
        p = struct.unpack_from('<IIIIIIII', data, phoff + index * phsize)
        if p[0] == 1:
            segments.append((p[2], data[p[1]:p[1] + p[4]]))
    sections = [struct.unpack_from('<IIIIIIIIII', data, shoff + i * shsize)
                for i in range(shnum)]
    symbols = {}
    for section in sections:
        if section[1] != 2:
            continue
        names = sections[section[6]]
        strings = data[names[4]:names[4] + names[5]]
        for offset in range(section[4], section[4] + section[5], section[9]):
            name, value, _, info, _, _ = struct.unpack_from('<IIIBBH', data, offset)
            if name and value and info & 15 == 2:
                name = strings[name:strings.index(0, name)].decode('ascii')
                symbols[name] = value
    return segments, symbols


class ArmFunction:
    def __init__(self, api, name):
        self.api, self.name = api, name
        self.restype = C.c_int
        self.argtypes = None

    def __call__(self, *args):
        result = self.api.call(self.name, args)
        if self.restype is C.c_float:
            return struct.unpack('<f', struct.pack('<I', result))[0]
        if self.restype is C.c_char_p:
            value = bytearray()
            while True:
                byte = self.api.cpu.mem_read(result + len(value), 1)[0]
                if not byte:
                    return bytes(value)
                value.append(byte)
        if self.restype is None:
            return None
        return self.restype(result).value


class ArmApi:
    def __init__(self, root):
        from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
        from unicorn.arm_const import (UC_ARM_REG_R0, UC_ARM_REG_R1,
            UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR,
            UC_ARM_REG_XPSR, UC_ARM_REG_PC)
        self.registers = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3]
        self.sp, self.lr = UC_ARM_REG_SP, UC_ARM_REG_LR
        self.pc = UC_ARM_REG_PC
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.cpu.mem_map(0x08000000, 0x100000)
        self.cpu.mem_map(0x20000000, 0x20000)
        self.cpu.reg_write(UC_ARM_REG_XPSR, 0x01000000)
        override = os.environ.get('CONTROL_ARM_ELF')
        segments, self.symbols = read_elf(Path(override) if override else build(root))
        for address, data in segments:
            self.cpu.mem_write(address, data)
        self.functions = {}

    def __getattr__(self, name):
        if name not in self.symbols:
            raise AttributeError(name)
        if name not in self.functions:
            self.functions[name] = ArmFunction(self, name)
        return self.functions[name]

    def call(self, name, args):
        if len(args) > 4:
            raise ValueError('测试接口只支持最多四个参数')
        pointers = []
        for index, arg in enumerate(args):
            if hasattr(arg, '_obj'):
                address = 0x2001D000 + index * 16
                self.cpu.mem_write(address, C.string_at(arg, C.sizeof(arg._obj)))
                pointers.append((arg, address))
                value = address
            else:
                value = int(getattr(arg, 'value', arg)) & 0xFFFFFFFF
            self.cpu.reg_write(self.registers[index], value)
        self.cpu.reg_write(self.sp, 0x2001F000)
        self.cpu.reg_write(self.lr, 0x080FF001)
        self.cpu.emu_start(self.symbols[name] | 1, 0x080FF000, count=200000)
        if self.cpu.reg_read(self.pc) != 0x080FF000:
            raise RuntimeError(f'{name} 超过 Cortex-M3 测试指令预算')
        for arg, address in pointers:
            C.memmove(arg, bytes(self.cpu.mem_read(address, C.sizeof(arg._obj))), C.sizeof(arg._obj))
        return self.cpu.reg_read(self.registers[0])
