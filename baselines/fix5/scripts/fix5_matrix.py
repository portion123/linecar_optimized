"""Compile FIX5 all-off/stable-on/all-features/single-switch branches.
No ARMCC or STM32 link is implied. This is independent of sanitizers.
"""
import hashlib
import json
from pathlib import Path
import subprocess
from fix5_golden import ROOT, SWITCHES, make_header

def main():
    directory=ROOT/'tests/native/fix5_matrix';directory.mkdir(parents=True,exist_ok=True)
    off={key:0 for key in SWITCHES}
    stable={key:int(key in ('TRACK_TRACE_ENABLE','TRACK_ERROR_TREND_ENABLE',
                          'TRACK_ALIGN_TREND_ENABLE')) for key in SWITCHES}
    all_features={key:int(key!='TRACK_ERROR_PREDICT_ENABLE') for key in SWITCHES}
    branches={'all_off':off,'stable_on':stable,'all_features_on':all_features}
    branches.update({key.lower():dict(off,**{key:1}) for key in SWITCHES})
    flags=['-std=c99','-O2','-g','-Wall','-Wextra','-Wdouble-promotion','-ffp-contract=off','-Werror',
        '-DUSE_STDPERIPH_DRIVER','-DSTM32F10X_HD','-Itests/native','-Istart','-Ilibrary','-Iuser','-Ihardware','-Isystem']
    results=[];headers={}
    for label,overrides in branches.items():
        branch=directory/label;header=make_header(branch,overrides)
        headers[str(header.relative_to(ROOT))]=hashlib.sha256(header.read_bytes()).hexdigest()
        commands=[['gcc',*flags,'-shared','-fPIC','-include',str(header),'tests/native/harness.c','-o',str(branch/'controller.so')],
            ['gcc',*flags,'-include',str(header),'-fsyntax-only','user/main.c']]
        for command in commands:
            outcome=subprocess.run(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            results.append({'variant':label,'unit':'harness' if '-shared' in command else 'main',
                'passed':outcome.returncode==0,'exit':outcome.returncode,'output':outcome.stdout})
        print(label+': '+('PASS' if all(r['passed'] for r in results[-2:]) else 'FAIL'),flush=True)
    report={'compiler':subprocess.check_output(['gcc','--version'],text=True).splitlines()[0],
        'flags':flags,'variants':len(branches),'checks':len(results),'passed':sum(r['passed'] for r in results),
        'switches':branches,'config_headers_sha256':headers,
        'forbidden_flags_absent':['-ffast-math','-march=native'],
        'stm32_armcc':'NOT RUN','results':results}
    (ROOT/'docs/fix5/compile_matrix.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return int(not all(r['passed'] for r in results))

if __name__=='__main__':raise SystemExit(main())
