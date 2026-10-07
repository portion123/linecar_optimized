"""重跑实际C源码的host验证，分别记录单元/标准几何/压力失败，不使用历史HEX。"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT/'tests/verification.txt'


def main():
    # 覆盖所有当前固件C/头文件、测试和工程文件，不对运行产生的数据文件做摘要。
    paths = sorted({p for folder in ('hardware','system','user','start','library','tests','scripts')
                    for p in (ROOT/folder).rglob('*')
                    if p.suffix in ('.c','.h','.py') and 'variants' not in p.parts})
    paths += [ROOT/'tracking_square_continuous.uvprojx',ROOT/'build.ps1']
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    log = ['验证时间 UTC: '+datetime.now(timezone.utc).isoformat(),
           '执行交付源码的真实C逻辑；GPIO/计时/电机动力学由host替身或假设模型提供。',
           'ARMCC及arm-none-eabi-gcc不可用：没有生成本次修改的STM32固件。',
           '历史HEX/AXF仅存放在docs/original_evidence/firmware；不作为本次验证证据。',
           '单元测试和标准几何必须通过；压力失败单独列出，不能称全部测试通过。',
           'GCC语法检查与六种宏配置仅证明host可编译，不证明ARM链接或实车时序。',
           '几何保持原35mm矩形RUN/25mm圆弧全程误差门槛，并记录全部状态误差。',
           '', '源码 SHA256:']
    log += [digest+'  '+name for name,digest in hashes.items()]
    env = os.environ.copy(); env['PYTHONUTF8']='1'; env.pop('CONTROL_ARM_ELF',None)
    commands = [
        ('units',[sys.executable,'-m','unittest','discover','-s','tests','-p','test_*.py','-v']),
        ('build',[sys.executable,'scripts/check_build.py']),
        ('geometry',[sys.executable,'tests/test_geometry.py','--stress']),
    ]
    statuses={}; unit_summary={}
    for label,command in commands:
        print('RUN '+' '.join(command),flush=True)
        completed=subprocess.run(command,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        output=completed.stdout.decode('utf-8',errors='replace').replace('\r\n','\n')
        print(output,flush=True)
        log += ['', '$ '+' '.join(command),output,f'exit={completed.returncode}']
        statuses[label]=completed.returncode
        if label=='units':
            match=re.search(r'Ran (\d+) tests?',output)
            unit_summary={'original_adapted':53,'new':41,'total':int(match[1]) if match else 0,
                          'passed':0,'failed':0,'skipped':0}
            for field,pattern in [('failed',r'failures=(\d+)'),('errors',r'errors=(\d+)'),('skipped',r'skipped=(\d+)')]:
                item=re.search(pattern,output);unit_summary[field]=int(item[1]) if item else 0
            unit_summary['passed']=unit_summary['total']-unit_summary['failed']-unit_summary['errors']-unit_summary['skipped']
        LOG.write_text('\n'.join(log),encoding='utf-8')
    changed=[name for name,digest in hashes.items() if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest]
    if changed:
        statuses['source_changed']=1;log += ['', '验证途中源码改变，必须重跑: '+', '.join(changed)]
    geometry=json.loads((ROOT/'tests/geometry_results.json').read_text())
    pressure=json.loads((ROOT/'tests/pressure_results.json').read_text())['results']
    def totals(items):
        failures=[{'case':r['case'],'state':r['state'],'stop_reason':r['stop_reason'],
                   'laps':r['laps'],'max_run_front_error_mm':r['max_run_front_error_mm'],
                   'max_all_front_error_mm':r['max_all_front_error_mm']} for r in items if not r['passed']]
        return {'total':len(items),'passed':len(items)-len(failures),'failed':len(failures),'failures':failures}
    standard=totals(geometry); stress=totals(pressure)
    required=not any(statuses.values()) and not standard['failed']
    result={'time_utc':datetime.now(timezone.utc).isoformat(),'units':unit_summary,
            'geometry_standard':standard,'geometry_stress':stress,'exit_codes':statuses,
            'required_checks_passed':required,'all_tests_passed':required and not stress['failed'],
            'arm_firmware_build':'NOT RUN','physical_car_test':'NOT RUN',
            'source_hashes':hashes,'source_changed_during_verification':changed}
    (ROOT/'tests/verification_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    log += ['',f'REQUIRED CHECKS: {"PASS" if required else "FAIL"}',
            f'STRESS: {stress["passed"]}/{stress["total"]} passed, {stress["failed"]} failed',
            'ALL TESTS: '+('PASS' if result['all_tests_passed'] else 'FAIL (see recorded failures)'),
            'STM32 ARM firmware / physical test: NOT RUN']
    LOG.write_text('\n'.join(log),encoding='utf-8')
    print('\n'.join(log[-5:]),flush=True)
    return int(not required)


if __name__=='__main__':raise SystemExit(main())
