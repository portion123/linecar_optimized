"""Build temporary mutants and require the corresponding real tests to fail.

No production file is patched.  All variants use the same GCC flags and the
same explicit FIX5 switches.  A compiler error or an already-failing reference
is a failed mutation check, never a killed mutant.  Results record source
hashes and the workspace status before/after so cleanup is inspectable.
"""
import argparse
import ctypes as C
import hashlib
import importlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
FEATURES=('TRACK_TRACE_ENABLE','TRACK_SENSOR_FILTER_ENABLE',
          'TRACK_NONLINEAR_FOLLOW_ENABLE','TRACK_ERROR_TREND_ENABLE',
          'TRACK_ADAPTIVE_SPEED_ENABLE','TRACK_LOW_SPEED_ZONE_ENABLE',
          'TRACK_TURN_CONTINUITY_ENABLE','TRACK_ALIGN_TREND_ENABLE')


def once(source,old,new):
    if source.count(old)!=1:
        raise RuntimeError('mutation anchor must occur once: '+old)
    return source.replace(old,new,1)


def alter_function(source,name,change):
    match=re.search(r'^static\s+[^\n;]+\b'+re.escape(name)+r'\([^;]*?\)\s*\{',
                    source,re.MULTILINE)
    if not match:
        raise RuntimeError('production helper not found: '+name)
    start=match.end()-1; depth=0
    for index in range(start,len(source)):
        if source[index]=='{': depth+=1
        elif source[index]=='}':
            depth-=1
            if not depth:
                body=source[start+1:index]
                changed=change(body,match.group(0))
                return source[:start+1]+changed+source[index:]
    raise RuntimeError('unclosed function: '+name)


def make_variants(source,header):
    yield 'bypass_filter',once(source,
        'sensor_filtered=MajorityMask(sensor_history[0],sensor_history[1],sensor_history[2]);',
        'sensor_filtered=sensor_raw;'),header,('FilterFollowTests','test_02_single_frame_noise_does_not_change_line_error')
    yield 'reverse_error_rate',alter_function(source,'TrendPush',
        lambda body,_: body+'\n    error_rate=-error_rate;\n'),header,('FilterFollowTests','test_06_same_error_outward_trend_slows_earlier')
    yield 'one_side_target_bias',alter_function(source,'FollowLine',
        lambda body,_: once(body,'left_target=base+correction;',
                            'left_target=base+correction+0.25f;')),header,('FilterFollowTests','test_07_mirrored_logic_swaps_targets_and_negates_correction')
    yield 'old_low_speed_dead_zone',alter_function(source,'SpeedOutput',
        lambda body,_: once(body,'target=Abs(target);',
                            'target=Abs(target); if(target<10.0f) target=0.0f;')),header,('SpeedContinuityTests','test_08_nonzero_5_8_12_18rpm_never_becomes_zero')
    yield 'remove_first_cause',alter_function(source,'EnterStop',
        lambda body,_: once(body,'if(state==CAR_STOP) return;',
                            '/* intentionally removed first-cause guard */')),header,('TraceStopTests','test_18b_stop_is_idempotent_first_cause_and_frozen')
    yield 'user_reason_aliases_fault',source,once(header,
        '#define STOP_REASON_USER_KEY TRACK_STOP_USER',
        '#define STOP_REASON_USER_KEY TRACK_STOP_ENCODER'),('TraceStopTests','test_18c_user_key_has_distinct_reason_and_restart_rearms')
    yield 'restart_keeps_trace_frozen',once(source,
        'stop_reason=STOP_REASON_NONE; TraceRestart();',
        'stop_reason=STOP_REASON_NONE; if(state==CAR_STOP) TraceRestart();'),header,('TraceStopTests','test_18c_user_key_has_distinct_reason_and_restart_rearms')
    def poison_forward_base(body,signature):
        # Read parameter spelling from the real helper; do not duplicate its
        # mode handling or infer state from feedback.
        params=signature[signature.index('(')+1:signature.rfind(')')].split(',')
        left=params[1].strip().split()[-1]; right=params[2].strip().split()[-1]
        return '\n    adaptive_base_rpm=('+left+'+'+right+')*0.5f;\n'+body
    yield 'pivot_pollutes_adaptive_base',alter_function(source,'AdaptiveSync',poison_forward_base),header,('TurnAlignHandoffTests','test_19b_spin_and_pivot_do_not_pollute_forward_base')


def build(directory,name,source,header):
    directory=directory/name; directory.mkdir()
    track=directory/'Track.c'; track.write_text(source)
    (directory/'Track.h').write_text(header)
    harness_source=(ROOT/'tests/native/harness.c').read_text()
    def absolute_include(match):
        path=(ROOT/'tests/native'/match.group(1)).resolve()
        if path==ROOT/'hardware/Track.c': path=track
        return '#include "'+str(path)+'"'
    harness_source=re.sub(r'^#include "([^\"]+\.c)"',absolute_include,
                          harness_source,flags=re.MULTILINE)
    harness_source=harness_source.replace('#include "fix5_observers.inc"',
        '#include "'+str(ROOT/'tests/native/fix5_observers.inc')+'"')
    harness=directory/'harness.c'; harness.write_text(harness_source)
    library=directory/'controller.so'
    command=['gcc','-std=c99','-O2','-g','-shared','-fPIC','-Wall','-Wextra',
             '-Wdouble-promotion','-ffp-contract=off','-Werror',
             '-DUSE_STDPERIPH_DRIVER','-DSTM32F10X_HD']
    command += ['-D'+feature+'=1' for feature in FEATURES]
    command += ['-DTRACK_ERROR_PREDICT_ENABLE=0']
    # KEY.c includes Track.h before the copied Track.c. Preinclude the variant
    # header so its guard/stop aliases are used by every real C unit.
    command += ['-include',str(directory/'Track.h')]
    command += ['-I'+str(ROOT/path) for path in
                ('tests/native','start','library','user','hardware','system')]
    command += [str(harness),'-o',str(library)]
    subprocess.run(command,cwd=ROOT,check=True,capture_output=True,text=True)
    return C.CDLL(str(library))


def configure_like(library,original):
    # ctypes signatures already declared by native_api/test_fix5 are copied;
    # algorithms, state and masks remain inside the independent real C build.
    for name in dir(original):
        if name.startswith(('Native_','Fix5_','Track_','Gray_','Encoder_')):
            if hasattr(library,name):
                before=getattr(original,name); after=getattr(library,name)
                after.argtypes=before.argtypes; after.restype=before.restype


def run_test(module,native,library,class_name,method):
    original=module.fw; native_original=native.fw
    configure_like(library,original)
    module.configure(library)
    module.fw=native.fw=library
    try:
        test=getattr(module,class_name)(method)
        result=unittest.TestResult(); test.run(result)
        return {'passed':not(result.failures or result.errors or result.skipped),
                'assertion_failures':len(result.failures),'errors':len(result.errors),
                'skipped':len(result.skipped),
                'detail':(result.failures+result.errors+result.skipped)}
    finally:
        module.fw=original; native.fw=native_original


def serial_result(result):
    return {key:([str(item[1]) for item in value] if key=='detail' else value)
            for key,value in result.items()}


def hashes():
    files=sorted((ROOT/'hardware').glob('*'))+sorted((ROOT/'system').glob('*'))
    return {str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files if path.is_file()}


def status():
    # A source ZIP intentionally has no .git; do not accidentally inspect an
    # unrelated parent repository. Source hashes still prove no patch leaked.
    try:
        result=subprocess.run(['git','rev-parse','--show-toplevel'],cwd=ROOT,
                              text=True,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
    except OSError:
        return None
    if result.returncode or Path(result.stdout.strip()).resolve()!=ROOT:
        return None
    return subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path)
    arguments=parser.parse_args()
    # Avoid environment-specific preincluded configuration affecting the
    # initial ctypes interface. Every tested variant explicitly enables the
    # same functions; prediction stays off.
    os.environ.pop('CONTROL_CONFIG_HEADER',None)
    os.environ.pop('CONTROL_SANITIZE',None)
    module=importlib.import_module('test_fix5')
    native=importlib.import_module('native_api')
    before_hashes=hashes(); before_status=status()
    source=(ROOT/'hardware/Track.c').read_text()
    header=(ROOT/'hardware/Track.h').read_text()
    rows=[]
    with tempfile.TemporaryDirectory(prefix='linecar_fix5_mutation_') as temporary:
        directory=Path(temporary)
        reference=build(directory,'reference',source,header)
        for name,mutated,mutated_header,target in make_variants(source,header):
            baseline=run_test(module,native,reference,*target)
            row={'mutation':name,'test':'.'.join(target),'reference':serial_result(baseline)}
            if not baseline['passed']:
                row.update(killed=False,problem='reference assertion does not pass')
            else:
                try:
                    library=build(directory,name,mutated,mutated_header)
                    result=run_test(module,native,library,*target)
                    row.update(mutant=serial_result(result),killed=result['assertion_failures']>0 and not result['errors'] and not result['skipped'])
                except subprocess.CalledProcessError as error:
                    row.update(killed=False,problem='mutant compile failed',compiler=error.stderr)
            rows.append(row)
    after_hashes=hashes(); after_status=status()
    report={'flags':'GCC -O2 -Wall -Wextra -Wdouble-promotion -ffp-contract=off -Werror',
            'features':{feature:1 for feature in FEATURES}|{'TRACK_ERROR_PREDICT_ENABLE':0},
            'mutations':rows,'killed':sum(row['killed'] for row in rows),'total':len(rows),
            'source_unchanged':before_hashes==after_hashes,
            'workspace_status_unchanged':before_status==after_status,
            'git_workspace_available':before_status is not None,
            'workspace_was_clean':not before_status if before_status is not None else None,
            'source_sha256':after_hashes}
    if arguments.output:
        arguments.output.parent.mkdir(parents=True,exist_ok=True)
        arguments.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:value for key,value in report.items() if key!='source_sha256'},indent=2))
    return 0 if report['killed']==report['total'] and report['source_unchanged'] and report['workspace_status_unchanged'] else 1


if __name__=='__main__': sys.exit(main())
