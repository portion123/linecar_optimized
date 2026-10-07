"""Immutable baseline capture / exact all-off comparison of control outputs.

All fields are control semantics. No trace index, log sequence, scheduler wall
clock or newly added observations participate. Float hex strings compare bit
exactly: tolerance is zero, same GCC and -O2/-ffp-contract=off flags.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'docs/fix5/baseline'
SWITCHES=('TRACK_TRACE_ENABLE','TRACK_SENSOR_FILTER_ENABLE','TRACK_NONLINEAR_FOLLOW_ENABLE',
    'TRACK_ERROR_TREND_ENABLE','TRACK_ADAPTIVE_SPEED_ENABLE','TRACK_ERROR_PREDICT_ENABLE',
    'TRACK_LOW_SPEED_ZONE_ENABLE','TRACK_TURN_CONTINUITY_ENABLE','TRACK_ALIGN_TREND_ENABLE')

def make_header(directory,overrides):
    directory.mkdir(parents=True,exist_ok=True)
    content=(ROOT/'hardware/CarConfig.h').read_text(encoding='utf-8-sig')
    for key,value in overrides.items():
        pattern=r'(#define\s+'+key+r'\s+)\S+'
        if re.search(pattern,content):content=re.sub(pattern,lambda m:m[1]+str(value),content,count=1)
        else:
            prefix,suffix=content.rsplit('#endif',1)
            content=prefix+f'#define {key} {value}\n#endif'+suffix
    path=directory/'CarConfig_override.h';path.write_text(content,encoding='utf-8');return path

def run(command,env,log):
    result=subprocess.run(command,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    log.write_text(result.stdout,encoding='utf-8')
    print(log.name+': exit='+str(result.returncode),flush=True)
    return result

def compare(left,right):
    count=0
    with gzip.open(left,'rt') as expected,gzip.open(right,'rt') as actual:
        for count,(a,b) in enumerate(__import__('itertools').zip_longest(expected,actual),1):
            if a!=b:
                old=json.loads(a) if a else None;new=json.loads(b) if b else None
                keys=[key for key in (old or {}).get('control',{}) if (old or {})['control'][key]!=(new or {}).get('control',{}).get(key)]
                return {'passed':False,'first_differing_record':count,'fields':keys,'expected':old,'actual':new}
    return {'passed':True,'records':count,'float_tolerance':0}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=('capture','compare'))
    parser.add_argument('--output',type=Path);args=parser.parse_args()
    directory=args.output or (BASE if args.mode=='capture' else ROOT/'tests/native/fix5_all_off')
    if args.mode=='capture' and (BASE/'metadata.json').exists():
        raise SystemExit('Baseline already captured. Refusing to replace immutable golden.')
    if args.mode=='compare':
        metadata=json.loads((BASE/'metadata.json').read_text(encoding='utf-8'))
        for name,digest in metadata['golden_sha256'].items():
            if hashlib.sha256((BASE/name).read_bytes()).hexdigest()!=digest:
                raise SystemExit('Immutable baseline SHA256 mismatch: '+name)
        compiler=subprocess.check_output(['gcc','--version'],text=True).splitlines()[0]
        if compiler!=metadata['gcc']:
            raise SystemExit('Exact golden requires baseline compiler '+metadata['gcc']+
                             '; current compiler '+compiler+'. Baseline must not be rewritten.')
    directory.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env['PYTHONUTF8']='1';env.pop('CONTROL_ARM_ELF',None)
    env['CONTROL_BUILD_TAG']='fix5_'+args.mode
    if args.mode=='compare':env['CONTROL_CONFIG_HEADER']=str(make_header(directory,{key:0 for key in SWITCHES}))
    manifest=directory/'unit_manifest.json' if args.mode=='capture' else BASE/'unit_manifest.json'
    commands={
        'units':[sys.executable,'tests/fix5_units.py','--write-manifest' if args.mode=='capture' else '--manifest',str(manifest)],
        'geometry':[sys.executable,'tests/test_geometry.py','--stress'],
        'random':[sys.executable,'tests/fix5_random.py','--output',str(directory/'random_results.json')],
    }
    statuses={}; comparisons={}
    for label,command in commands.items():
        env['CONTROL_CAPTURE']=str(directory/(label+'_golden.jsonl.gz'))
        outcome=run(command,env,directory/(label+'.log'));statuses[label]=outcome.returncode
        if args.mode=='compare':
            comparisons[label]=compare(BASE/(label+'_golden.jsonl.gz'),directory/(label+'_golden.jsonl.gz'))
            print(label+' golden '+json.dumps({k:v for k,v in comparisons[label].items() if k not in ('expected','actual')}),flush=True)
        elif label=='geometry':
            for name in ('geometry_results.json','pressure_results.json'):
                shutil.copyfile(ROOT/'tests'/name,directory/name)
    env.pop('CONTROL_CAPTURE',None)
    if args.mode=='capture':
        statuses['build']=run([sys.executable,'scripts/check_build.py'],env,directory/'build.log').returncode
        shutil.copyfile(ROOT/'tests/build_checks.json',directory/'build_checks.json')
        metadata={'baseline_git_commit':subprocess.check_output(['git','rev-parse','fix4_baseline'],cwd=ROOT,text=True).strip(),
            'gcc':subprocess.check_output(['gcc','--version'],text=True).splitlines()[0],
            'build_flags':['-std=c99','-O2','-g','-Wall','-Wextra','-Wdouble-promotion','-ffp-contract=off','-Werror'],
            'forbidden_flags_absent':['-ffast-math','-march=native'],'float_comparison_tolerance':0,
            'integer_enum_comparison':'exact','switches_for_all_off':list(SWITCHES),
            'exit_codes':statuses,'golden_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in directory.glob('*golden.jsonl.gz')}}
        (directory/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
        print(json.dumps(metadata,indent=2))
    else:
        (directory/'comparison.json').write_text(json.dumps({'exit_codes':statuses,'comparisons':comparisons},indent=2),encoding='utf-8')
    return int(any(statuses.values()) or any(not c['passed'] for c in comparisons.values()))

if __name__=='__main__':raise SystemExit(main())
