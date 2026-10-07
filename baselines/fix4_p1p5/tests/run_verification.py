"""可重跑的验证入口：控制回归、几何模型、ARMCC 固件构建与源码摘要。"""
from datetime import datetime, timezone, timedelta
import hashlib
from pathlib import Path
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
LOG = Path(__file__).with_name('verification.txt')


def validate_hex(path):
    memory={}; base=0; ended=False; records=0
    for line in path.read_text(encoding='ascii').splitlines():
        if not line.startswith(':'):
            raise ValueError('HEX 记录缺少冒号')
        record=bytes.fromhex(line[1:]); records+=1
        if len(record)!=record[0]+5 or sum(record)&255:
            raise ValueError('HEX 长度或校验和不正确')
        count,high,low,kind=record[:4]; address=(high<<8)|low; data=record[4:4+count]
        if kind==0:
            for offset,byte in enumerate(data): memory[base+address+offset]=byte
        elif kind==1: ended=True
        elif kind==2: base=int.from_bytes(data,'big')<<4
        elif kind==4: base=int.from_bytes(data,'big')<<16
    if not ended or not memory or not (0x08000000<=min(memory)<=max(memory)<0x08080000):
        raise ValueError('HEX 结束记录或 STM32 Flash 地址不正确')
    vectors=bytes(memory[0x08000000+i] for i in range(8))
    stack=int.from_bytes(vectors[:4],'little'); reset=int.from_bytes(vectors[4:],'little')
    if not 0x20000000<stack<=0x20010000 or not reset&1 or (reset&~1) not in memory:
        raise ValueError('HEX 初始栈或 Thumb 复位入口不正确')
    return f'HEX PASS: {records} records, Flash 0x{min(memory):08X}..0x{max(memory):08X}, SP=0x{stack:08X}, Reset=0x{reset:08X}'


def main():
    log = [
        '本次验证运行交付目录中的实际 C 源码；记录生成后修改源码需要重跑。',
        'Windows 使用 ARMCC 5 编译硬件替身 harness，Unicorn 执行 Cortex-M3 指令。',
        'GPIO/编码器为硬件替身，几何和电机参数为假设；通过不等于实车验证。',
        '几何采用44/133/175mm、251/265.35计数及30/40%空转两点；摩擦、起转及低速响应为假设。',
        '28/40mm宽线标准双向矩形必须出现轮轴推进/直角旋转；20mm或偏置/角度/尺寸误差允许连续对线完成。',
        '所有矩形仍须完成至少1.1圈、1圈转向、4角、不停车，RUN前探头最大误差小于35mm。',
        '几何日志分别给出RUN/SEARCH/全程误差及SEARCH累计时间、最长全白；不把SEARCH误差排除后称全程稳定。',
        'pressure_results.json 的人为高摩擦压力不纳入标准通过结论，失败需作为局限报告。',
        '验证时间（UTC+8）: ' + datetime.now(timezone(timedelta(hours=8))).isoformat(),
        '', '源码 SHA256:']
    paths = [ROOT/'hardware/Track.c', ROOT/'hardware/CarConfig.h',
             ROOT/'tests/native/harness.c', ROOT/'tests/arm_api.py',
             ROOT/'tests/native_api.py', ROOT/'tests/test_control.py',
             ROOT/'tests/test_turn_regression.py', ROOT/'tests/test_calibration.py', ROOT/'tests/test_geometry.py',
             ROOT/'tests/track_model.py', ROOT/'build.ps1',
             ROOT/'tests/run_verification.py',
             ROOT/'tracking_square_continuous.uvprojx']
    for path in paths:
        log.append(hashlib.sha256(path.read_bytes()).hexdigest() + '  ' + str(path.relative_to(ROOT)))
    commands = [[sys.executable, 'tests/test_control.py'],
                [sys.executable, 'tests/test_turn_regression.py'],
                [sys.executable, 'tests/test_calibration.py'],
                [sys.executable, 'tests/test_geometry.py', '--stress']]
    if os.name == 'nt':
        shell = shutil.which('pwsh.exe') or shutil.which('powershell.exe')
        commands.append([shell, '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', 'build.ps1'])
    env = os.environ.copy()
    env['PYTHONUTF8'] = '1'
    env.pop('CONTROL_ARM_ELF', None)
    statuses = []
    for command in commands:
        print('RUN ' + ' '.join(command), flush=True)
        result = subprocess.run(command, cwd=ROOT, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        output = result.stdout.decode('utf-8', errors='replace').replace('\r\n', '\n')
        print(output, flush=True)
        log.extend(['', '$ ' + ' '.join(command), output, f'exit={result.returncode}'])
        statuses.append(result.returncode)
        LOG.write_text('\n'.join(log), encoding='utf-8')
    for path in (ROOT/'Objects/linefollow_calibrated_fix4.hex', ROOT/'Objects/linefollow_calibrated_fix4.axf'):
        if path.exists():
            log.append(hashlib.sha256(path.read_bytes()).hexdigest() + '  ' + str(path.relative_to(ROOT)))
    if os.name=='nt':
        try:
            hex_result=validate_hex(ROOT/'Objects/linefollow_calibrated_fix4.hex')
            print(hex_result,flush=True); log.extend(['',hex_result])
        except (ValueError,KeyError,OSError) as error:
            log.extend(['','HEX FAIL: '+str(error)]); statuses.append(1)
    # 校验运行途中控制源码没有变化，防止并行编辑后误把旧验证当成最终验证。
    original = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    logged = {line.split('  ', 1)[1]: line.split('  ', 1)[0] for line in log
              if '  ' in line and len(line.split('  ', 1)[0]) == 64}
    changed = [str(p.relative_to(ROOT)) for p in paths
               if original[str(p)] != logged[str(p.relative_to(ROOT))]]
    if changed:
        log.extend(['', '验证运行期间文件被改动，需要重跑: ' + ', '.join(changed)])
        statuses.append(1)
    log.extend(['', 'VERIFICATION PASS' if not any(statuses) else 'VERIFICATION FAIL'])
    LOG.write_text('\n'.join(log), encoding='utf-8')
    return int(any(statuses))


if __name__ == '__main__':
    raise SystemExit(main())
