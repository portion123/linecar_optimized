"""Repeat enabled model checks and compare each case to immutable FIX4 baseline.

Thresholds were declared with the baseline: success and safe bounded outcomes
are categorical; marked lateral degradation means >max(2mm,25%) additional
TRACK error. Timing >1.30x is an explanation flag, never a failure by itself.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from fix5_golden import BASE, ROOT, SWITCHES, make_header

def source_identity():
    """Git is optional in the delivered source archive; hashes remain useful."""
    commit=None
    try:
        result=subprocess.run(['git','rev-parse','--show-toplevel','HEAD'],cwd=ROOT,text=True,
                              stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
        fields=result.stdout.splitlines()
        if result.returncode==0 and len(fields)==2 and Path(fields[0]).resolve()==ROOT:
            commit=fields[1]
    except OSError:
        pass # Git is not a runtime requirement for the source archive.
    return {'source_commit':commit,
            'source_sha256':{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                for name in ('hardware/CarConfig.h','hardware/Track.c',
                             'hardware/Track.h','user/stm32f10x_it.c')}}

def compare_cases(baseline,actual):
    previous={case['case']:case for case in baseline}
    regressions=[];observations=[]
    for case in actual:
        old=previous.get(case['case'])
        if old is None:
            regressions.append({'case':case['case'],'reason':'no baseline'});continue
        reasons=[]
        if old['passed'] and not case['passed']:reasons.append('baseline success now fails')
        if not old.get('uncontrolled',False) and case.get('uncontrolled',False):reasons.append('new uncontrolled timeout')
        if old['passed'] and case.get('safety_stop',False):reasons.append('normal baseline success now protection STOP')
        for metric in ('max_run_front_error_mm',):
            allowance=max(2.0,.25*old[metric])
            if old['passed'] and case[metric]>old[metric]+allowance:
                reasons.append(metric+' marked deterioration')
        if reasons:regressions.append({'case':case['case'],'reasons':reasons,'baseline':old,'actual':case})
        observations.append({'case':case['case'],'baseline_passed':old['passed'],'passed':case['passed'],
            'baseline_stop_reason':old['stop_reason'],'stop_reason':case['stop_reason'],
            'track_error_delta_mm':round(case['max_run_front_error_mm']-old['max_run_front_error_mm'],2),
            'time_ratio':round(case['time_s']/old['time_s'],4) if old['time_s'] else None,
            'time_explanation_required':case['time_s']>1.30*old['time_s'],
            'straight_sign_flips_delta':case.get('straight_correction_sign_flips',0)-old.get('straight_correction_sign_flips',0)})
    missing=set(previous)-{case['case'] for case in actual}
    regressions.extend({'case':name,'reason':'missing current case'} for name in sorted(missing))
    return {'total':len(actual),'passed':sum(case['passed'] for case in actual),
        'baseline_passed':sum(case['passed'] for case in baseline),
        'safety_stops':sum(case.get('safety_stop',False) for case in actual),
        'uncontrolled':sum(case.get('uncontrolled',False) for case in actual),
        'regressions':regressions,'observations':observations}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('stage')
    parser.add_argument('--overrides',help='JSON object for enabled stage header')
    parser.add_argument('--skip-golden',action='store_true',help='only when exact golden already passed for unchanged source')
    args=parser.parse_args();directory=ROOT/'docs/fix5'/args.stage;directory.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env['PYTHONUTF8']='1';env.pop('CONTROL_ARM_ELF',None)
    env.pop('CONTROL_CAPTURE',None);env['CONTROL_BUILD_TAG']='fix5_'+args.stage
    if args.overrides:env['CONTROL_CONFIG_HEADER']=str(make_header(directory,json.loads(args.overrides)))
    statuses={}
    commands={'geometry':[sys.executable,'tests/test_geometry.py','--stress'],
        'random':[sys.executable,'tests/fix5_random.py','--output',str(directory/'random_results.json')]}
    for label,command in commands.items():
        outcome=subprocess.run(command,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (directory/(label+'.log')).write_text(outcome.stdout,encoding='utf-8');statuses[label]=outcome.returncode
        print(label+': exit='+str(outcome.returncode),flush=True)
        if label=='geometry':
            for name in ('geometry_results.json','pressure_results.json'):
                shutil.copyfile(ROOT/'tests'/name,directory/name)
    current=json.loads((directory/'geometry_results.json').read_text());baseline=json.loads((BASE/'geometry_results.json').read_text())
    standard=compare_cases(baseline,current)
    pressure=compare_cases(json.loads((BASE/'pressure_results.json').read_text())['results'],json.loads((directory/'pressure_results.json').read_text())['results'])
    random_cases=compare_cases(json.loads((BASE/'random_results.json').read_text())['results'],json.loads((directory/'random_results.json').read_text())['results'])
    if not args.skip_golden:
        golden_env=env.copy();golden_env.pop('CONTROL_CONFIG_HEADER',None)
        result=subprocess.run([sys.executable,'scripts/fix5_golden.py','compare','--output',str(directory/'all_off')],
            cwd=ROOT,env=golden_env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (directory/'golden.log').write_text(result.stdout,encoding='utf-8');statuses['golden']=result.returncode
        print('golden: exit='+str(result.returncode),flush=True)
    passed=not any(statuses.values()) and not any(group['regressions'] for group in (standard,pressure,random_cases))
    report={'stage':args.stage,**source_identity(),
        'enabled_overrides':json.loads(args.overrides) if args.overrides else 'current defaults',
        'pass':passed,'thresholds':{'success':'no previously successful case may fail',
            'uncontrolled':'no new timeout with motors enabled','normal_protection_stop':'none new on baseline success',
            'marked_track_error_degradation':'additional >max(2mm,baseline*0.25)',
            'time_ratio_explanation':1.30,'time_is_failure':False},
        'exit_codes':statuses,'standard':standard,'pressure':pressure,'random':random_cases,
        'claim_scope':'current model assumptions; not physical car stability or ARM firmware compilation'}
    (directory/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'pass':passed,'standard':f"{standard['passed']}/{standard['total']}",
        'pressure':f"{pressure['passed']}/{pressure['total']}",'random':f"{random_cases['passed']}/{random_cases['total']}",
        'new_regressions':sum(len(group['regressions']) for group in (standard,pressure,random_cases))},ensure_ascii=False))
    return int(not passed)

if __name__=='__main__':raise SystemExit(main())
