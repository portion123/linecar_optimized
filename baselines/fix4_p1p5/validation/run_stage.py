import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

root = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else Path(__file__).resolve().parents[1]
stage = sys.argv[1]
out = Path(__file__).parent / stage
out.mkdir(exist_ok=True)
commands = [('fault_matrix', ['python3', 'tests/test_fault_matrix.py']),
            ('control', ['python3', 'tests/test_control.py']),
            ('turn_regression', ['python3', 'tests/test_turn_regression.py']),
            ('calibration', ['python3', 'tests/test_calibration.py']),
            ('geometry', ['python3', 'tests/test_geometry.py'])]
if stage == 'final':
    commands[-1][1].append('--stress')
    commands.extend([('fault_random', ['python3', 'tests/test_fault_random.py']),
                     ('fault_details', ['python3', 'tests/test_fault_details.py'])])
summary = {'stage': stage, 'commands': []}
for name, command in commands:
    started = time.monotonic()
    result = subprocess.run(command, cwd=root, text=True, capture_output=True)
    (out / (name + '.stdout.txt')).write_text(result.stdout, encoding='utf-8')
    (out / (name + '.stderr.txt')).write_text(result.stderr, encoding='utf-8')
    record = {'name': name, 'command': ' '.join(command), 'returncode': result.returncode,
              'elapsed_seconds': round(time.monotonic() - started, 3)}
    if name == 'fault_matrix':
        record.update(enc_stops=result.stdout.count('结束于 ENC'),
                      lost_stops=result.stdout.count('结束于 LOST'))
    if name == 'geometry':
        for file in ['geometry_results.json', 'pressure_results.json', 'geometry_traces.json']:
            source = root / 'tests' / file
            if source.exists() and (file != 'pressure_results.json' or '--stress' in command):
                shutil.copy2(source, out / file)
        data = json.loads((out / 'geometry_results.json').read_text())
        record.update(passed=sum(r['passed'] for r in data), total=len(data),
                      max_run_front_error_mm=max(r['max_run_front_error_mm'] for r in data))
    if name == 'fault_random':
        shutil.copy2(root / 'tests/fault_random_results.json', out / 'fault_random_results.json')
    summary['commands'].append(record)
    (out / (name + '.execution.json')).write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(json.dumps(record), flush=True)
    print('\n'.join((result.stdout + result.stderr).strip().splitlines()[-4:]), flush=True)
(out / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
# Stage failures are retained verbatim; in particular calibration_04 is expected until P3.
