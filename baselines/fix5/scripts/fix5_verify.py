"""Run every required FIX5 host check sequentially against one source snapshot.

All-off records must match the immutable FIX4 golden exactly. Enabled models
may retain baseline stress/random failures; any new regression fails this
command. Skipped assertions are counted separately. This does not build ARM
firmware or test a physical car, and sanitizers provide no control metrics.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys

from fix5_golden import BASE, ROOT, SWITCHES, make_header


FINAL = ROOT / 'docs/fix5/final'
PROFILES = ROOT / 'tests/native/fix5_profiles'
STABLE = {key: int(key in ('TRACK_TRACE_ENABLE', 'TRACK_ERROR_TREND_ENABLE',
                          'TRACK_ALIGN_TREND_ENABLE')) for key in SWITCHES}
ALL_OFF = {key: 0 for key in SWITCHES}
ALL_FEATURES = {key: int(key != 'TRACK_ERROR_PREDICT_ENABLE') for key in SWITCHES}
EXPECTED_GOLDEN_RECORDS = 142327
# Original tests retain their FIX4 fault-state assertions and tolerances.
# Only these exact diagnostics are expected; an arbitrary failure in one of
# these testcases is still a regression and must fail the verification.
LEGACY_STOP_DIAGNOSTICS = {
    'test_control.ControlTests.test_11_encoder_fault_stops_not_infinite_boost': '8 != 9',
    'test_control.ControlTests.test_14_forward_mode_motor_polarity_and_no_line_dependency': '8 != 18',
    'test_control.ControlTests.test_18_millisecond_wrap_and_large_gap': '8 != 18',
    'test_safety_bounds.SafetyBoundsTests.test_fault_stop_stays_latched_until_explicit_restart': '8 != 9',
}
GENERATED_SOURCE_DIRECTORIES = {'variants', 'fix5_matrix', 'fix5_profiles', 'fix5_all_off'}


def relative(path):
    return str(Path(path).relative_to(ROOT))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():
    """Hash applied sources/tests/project files, excluding generated profiles/data."""
    paths = set()
    for folder in ('hardware', 'system', 'user', 'start', 'library', 'tests', 'scripts'):
        for path in (ROOT / folder).rglob('*'):
            name = path.relative_to(ROOT)
            generated = (name.parts[:2] == ('tests', 'native') and
                         any(part in GENERATED_SOURCE_DIRECTORIES for part in name.parts[2:-1]))
            if path.is_file() and path.suffix.lower() in ('.c', '.h', '.py', '.inc') and not generated:
                paths.add(path)
    paths.update(ROOT.glob('*.uvprojx'))
    paths.update(ROOT.glob('*.ps1'))
    return {relative(path): sha256(path) for path in sorted(paths)}


def changed_sources(before, after):
    return sorted(name for name in set(before) | set(after) if before.get(name) != after.get(name))


def baseline_hashes():
    # Baseline JSON files are immutable comparison inputs, separate from
    # generated JSON results. Golden files are also checked against the
    # capture's recorded SHA256 values before they authorize --skip-golden.
    names = ('metadata.json', 'unit_manifest.json', 'geometry_results.json',
             'pressure_results.json', 'random_results.json',
             'units_golden.jsonl.gz', 'geometry_golden.jsonl.gz', 'random_golden.jsonl.gz')
    return {relative(BASE / name): sha256(BASE / name) for name in names if (BASE / name).is_file()}


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def unit_counts(output):
    """Read actual unittest counts, including the original manifest runner."""
    manifest = re.search(r'^FIX5_UNITS_SUMMARY (\{[^\n]+\})$', output, re.MULTILINE)
    if manifest:
        result = json.loads(manifest[1])
    else:
        runs = re.findall(r'^Ran (\d+) tests?\b', output, re.MULTILINE)
        endings = re.findall(r'^(?:OK|FAILED)(?: \([^\n]*\))?$', output, re.MULTILINE)
        ending = endings[-1] if endings else ''
        result = {'total': int(runs[-1]) if runs else 0}
        for field, label in (('failed', 'failures'), ('errors', 'errors'), ('skipped', 'skipped')):
            match = re.search(label + r'=(\d+)', ending)
            result[field] = int(match[1]) if match else 0
        result['passed'] = result['total'] - sum(result[key] for key in ('failed', 'errors', 'skipped'))
    result['parsed'] = (result['total'] > 0 and result['passed'] >= 0 and
                        result['total'] == sum(result[key] for key in ('passed', 'failed', 'errors', 'skipped')))
    return result


def unit_diagnostics(output):
    result = []
    pattern = r'^(FAIL|ERROR): ([^\n]+)\n(.*?)(?=^={5,}|\Z)'
    for match in re.finditer(pattern, output, re.MULTILINE | re.DOTALL):
        identifier = re.search(r'\(([^)]+)\)$', match[2])
        result.append({'kind': match[1], 'test': identifier[1] if identifier else match[2],
                       'display': match[2],
                       'assertions': re.findall(r'^AssertionError: (.*)$', match[3], re.MULTILINE)})
    return result


def model_totals(group):
    total, passed = group.get('total', 0), group.get('passed', 0)
    return {'total': total, 'passed': passed, 'failed': total - passed,
            'baseline_passed': group.get('baseline_passed', 0),
            'safety_stops': group.get('safety_stops', 0), 'uncontrolled': group.get('uncontrolled', 0),
            'new_regressions': len(group.get('regressions', [])),
            'regressions': group.get('regressions', []),
            'failures': [case for case in group.get('observations', []) if not case['passed']]}


def main():
    FINAL.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['PYTHONUTF8'] = '1'
    for name in ('CONTROL_ARM_ELF', 'CONTROL_CAPTURE', 'CONTROL_CONFIG_HEADER',
                 'CONTROL_SANITIZE', 'CONTROL_BUILD_TAG'):
        env.pop(name, None)
    before = source_hashes()
    baseline_before = baseline_hashes()
    observed_changes = set()
    baseline_changes = set()
    generated_headers = {}
    commands, checks, artifact_errors = {}, {}, []
    started = datetime.now(timezone.utc).isoformat()
    print('FIX5 verification: current C source, GCC host/models only; ARM/physical NOT RUN', flush=True)

    def load_json(path):
        try:
            return json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError) as error:
            artifact_errors.append({'artifact': relative(path), 'error': str(error)})
            return {}

    def run(label, arguments, environment=None, artifacts=()):
        for path in artifacts:
            path.unlink(missing_ok=True)
        command = [sys.executable, *arguments]
        print('RUN ' + label + ': ' + shlex.join(['python3', *arguments]), flush=True)
        start = datetime.now(timezone.utc).isoformat()
        try:
            process = subprocess.run(command, cwd=ROOT, env=environment or env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, encoding='utf-8', errors='replace')
            output, code = process.stdout, process.returncode
        except OSError as error:
            output, code = str(error) + '\n', 127
        log = FINAL / (label + '.log')
        log.write_text(output, encoding='utf-8')
        commands[label] = {'command': ['python3', *arguments], 'start_utc': start,
                           'end_utc': datetime.now(timezone.utc).isoformat(),
                           'exit_code': code, 'log': relative(log)}
        observed_changes.update(changed_sources(before, source_hashes()))
        baseline_changes.update(changed_sources(baseline_before, baseline_hashes()))
        print(label + ': exit=' + str(code) + ' (' + relative(log) + ')', flush=True)
        return code, output

    build_code, _ = run('build', ['scripts/check_build.py'], artifacts=(ROOT / 'tests/build_checks.json',))
    build = load_json(ROOT / 'tests/build_checks.json')
    generated_headers.update({relative(path): sha256(path) for path in (ROOT / 'tests/native/variants').glob('*.h')})
    checks['build'] = build_code == 0 and bool(build.get('calibration_and_mapping_unchanged'))
    matrix_path = ROOT / 'docs/fix5/compile_matrix.json'
    matrix_code, _ = run('compile_matrix', ['scripts/fix5_matrix.py'], artifacts=(matrix_path,))
    matrix = load_json(matrix_path)
    generated_headers.update(matrix.get('config_headers_sha256', {}))
    checks['compile_matrix'] = (matrix_code == 0 and matrix.get('variants') == 12 and
                                matrix.get('checks') == matrix.get('passed') == 24 and
                                matrix.get('switches', {}).get('stable_on') == STABLE and
                                matrix.get('switches', {}).get('all_features_on') == ALL_FEATURES)

    golden_directory = FINAL / 'all_off'
    golden_code, _ = run('golden', ['scripts/fix5_golden.py', 'compare', '--output',
                                   relative(golden_directory)],
                         artifacts=(golden_directory / 'comparison.json',))
    golden = load_json(golden_directory / 'comparison.json')
    golden_header = golden_directory / 'CarConfig_override.h'
    if golden_header.exists():
        generated_headers[relative(golden_header)] = sha256(golden_header)
    baseline_metadata = load_json(BASE / 'metadata.json')
    checks['baseline_capture_integrity'] = (len(baseline_before) == 8 and
        len(baseline_metadata.get('golden_sha256', {})) == 3 and
        all(baseline_before.get(relative(BASE / name)) == digest
            for name, digest in baseline_metadata.get('golden_sha256', {}).items()))
    golden_units = unit_counts((golden_directory / 'units.log').read_text(encoding='utf-8')
                               if (golden_directory / 'units.log').exists() else '')
    golden_comparisons = golden.get('comparisons', {})
    records = sum(row.get('records', 0) for row in golden_comparisons.values())
    checks['golden_all_off'] = (golden_code == 0 and set(golden_comparisons) == {'units', 'geometry', 'random'} and
                                all(row.get('passed') is True for row in golden_comparisons.values()) and
                                records == EXPECTED_GOLDEN_RECORDS and golden_units['parsed'] and
                                golden_units['total'] == golden_units['passed'] == 94 and
                                not any(golden_units[key] for key in ('failed', 'errors', 'skipped')))
    golden.update(total_records=records, expected_records=EXPECTED_GOLDEN_RECORDS,
                  units=golden_units, artifact=relative(golden_directory / 'comparison.json'),
                  source_sha256=before)

    stage_path = FINAL / 'comparison.json'
    # Reusing an earlier invocation's golden result is forbidden. The exact
    # comparison above belongs to this invocation and this source snapshot.
    golden_reusable = (checks['golden_all_off'] and checks['baseline_capture_integrity'] and
                       not observed_changes and not baseline_changes and
                       not changed_sources(before, source_hashes()))
    if golden_reusable:
        stage_code, _ = run('stage', ['scripts/fix5_stage.py', 'final', '--skip-golden'], artifacts=(stage_path,))
        stage = load_json(stage_path)
    else:
        stage_path.unlink(missing_ok=True)
        stage_code, stage = None, {}
        (FINAL / 'stage.log').write_text('NOT RUN: exact all-off golden prerequisite failed or source changed.\n', encoding='utf-8')
        commands['stage'] = {'exit_code': None, 'status': 'not_run', 'log': relative(FINAL / 'stage.log')}
    models = {name: model_totals(stage.get(name, {})) for name in ('standard', 'pressure', 'random')}
    checks['enabled_models'] = (stage_code == 0 and stage.get('pass') is True and
                                all(models[name]['total'] == count for name, count in
                                    (('standard', 34), ('pressure', 10), ('random', 30))) and
                                models['standard']['failed'] == 0 and
                                all(group['new_regressions'] == 0 for group in models.values()))

    headers = {'all_off': make_header(PROFILES / 'all_off', ALL_OFF),
               'all_features': make_header(PROFILES / 'all_features', ALL_FEATURES)}
    header_hashes = {name: sha256(path) for name, path in headers.items()}
    generated_headers.update({relative(path): header_hashes[name] for name, path in headers.items()})
    config = (ROOT / 'hardware/CarConfig.h').read_text(encoding='utf-8-sig')
    defaults = {}
    for name in SWITCHES:
        match = re.search(r'^#define\s+' + name + r'\s+(\d+)\b', config, re.MULTILINE)
        defaults[name] = int(match[1]) if match else None
    checks['default_switches'] = defaults == STABLE

    suites = {}
    for label, profile in (('logic_default', None), ('logic_all_on', 'all_features')):
        unit_env = env.copy()
        unit_env['CONTROL_BUILD_TAG'] = 'fix5_verify_' + label
        if profile:
            unit_env['CONTROL_CONFIG_HEADER'] = relative(headers[profile])
        code, output = run(label, ['tests/test_fix5.py'], unit_env)
        counts = unit_counts(output)
        diagnostics = unit_diagnostics(output)
        suites[label] = dict(counts, failures=[row['display'] for row in diagnostics if row['kind'] == 'FAIL'],
                             error_details=[row['display'] for row in diagnostics if row['kind'] == 'ERROR'],
                             log=commands[label]['log'])
        expected = (17, 14) if profile is None else (31, 0)
        checks[label] = (code == 0 and counts['parsed'] and counts['total'] == 31 and
                         (counts['passed'], counts['skipped']) == expected and
                         counts['failed'] == counts['errors'] == 0)

    legacy_env = env.copy()
    legacy_env['CONTROL_BUILD_TAG'] = 'fix5_verify_legacy_default'
    legacy_code, output = run('legacy_default', ['tests/fix5_units.py', '--manifest',
                                               relative(BASE / 'unit_manifest.json')], legacy_env)
    legacy = unit_counts(output)
    legacy_diagnostics = unit_diagnostics(output)
    permitted = all(row['kind'] == 'FAIL' and row['test'] in LEGACY_STOP_DIAGNOSTICS and
                    row['assertions'] == [LEGACY_STOP_DIAGNOSTICS[row['test']]] for row in legacy_diagnostics)
    checks['legacy_default_contract'] = (legacy_code in (0, 1) and legacy['parsed'] and
                                         legacy['total'] == 94 and legacy['errors'] == legacy['skipped'] == 0 and
                                         legacy['passed'] == 90 and
                                         legacy['failed'] == len(legacy_diagnostics) == 4 and permitted)
    legacy.update(failures=[row['display'] for row in legacy_diagnostics if row['kind'] == 'FAIL'],
                  error_details=[row['display'] for row in legacy_diagnostics if row['kind'] == 'ERROR'],
                  diagnostics=legacy_diagnostics, log=commands['legacy_default']['log'],
                  expected_difference='FIX5 unified CAR_STOP=8; original tests assert FIX4 fault states 9/18',
                  contract_verified=checks['legacy_default_contract'])
    suites['legacy_default'] = legacy

    fault_code, fault_output = run('fault_stop', ['tests/test_fault_stop.py'])
    faults = unit_counts(fault_output)
    faults['log'] = commands['fault_stop']['log']
    checks['fatal_handlers'] = (fault_code == 0 and faults['parsed'] and
                                faults['total'] == faults['passed'] == 4 and
                                faults['failed'] == faults['errors'] == faults['skipped'] == 0)

    mutation_path = FINAL / 'mutations.json'
    mutation_code, _ = run('mutations', ['tests/run_fix5_mutations.py', '--output', relative(mutation_path)],
                           artifacts=(mutation_path,))
    mutations = load_json(mutation_path)
    checks['mutations'] = (mutation_code == 0 and mutations.get('total') == mutations.get('killed') == 8 and
                           mutations.get('source_unchanged') is True and
                           mutations.get('workspace_status_unchanged') is True)

    sanitizers = {}
    for profile, switches in (('default', STABLE), ('all_off', ALL_OFF), ('all_features', ALL_FEATURES)):
        path = FINAL / ('sanitizer_' + profile + '.json')
        args = ['tests/run_fix5_sanitizer.py', '--output', relative(path)]
        header = headers.get(profile, ROOT / 'hardware/CarConfig.h')
        if profile != 'default':
            args += ['--config-header', relative(header)]
        sanitizer_before = source_hashes()
        digest = sha256(header)
        code, _ = run('sanitizer_' + profile, args, artifacts=(path,))
        sanitizer_after = source_hashes()
        report = load_json(path)
        stable_source = sanitizer_before == sanitizer_after and sha256(header) == digest
        report.update(variant=profile, requested_switches=switches,
                      source_sha256=sanitizer_before, source_sha256_after=sanitizer_after,
                      source_unchanged_during_run=stable_source, config_header=relative(header),
                      config_header_sha256=digest, reproduce_command=['python3', *args])
        report['build_command'] = [relative(value) if isinstance(value, str) and value.startswith(str(ROOT) + '/')
                                   else value for value in report.get('build_command', [])]
        write_json(path, report)
        sanitizers[profile] = {'status': report.get('status', 'failed'), 'exercise': report.get('exercise', {}),
                               'source_unchanged_during_run': stable_source,
                               'config_header_sha256': digest, 'artifact': relative(path),
                               'purpose': 'memory bounds and undefined behaviour only; no control metrics'}
        checks['sanitizer_' + profile] = (code == 0 and report.get('status') == 'passed' and stable_source and
                                          report.get('exercise', {}).get('checked_periods') == 40470)

    after = source_hashes()
    baseline_after = baseline_hashes()
    observed_changes.update(changed_sources(before, after))
    baseline_changes.update(changed_sources(baseline_before, baseline_after))
    checks['source_unchanged'] = not observed_changes
    checks['baseline_unchanged'] = not baseline_changes
    checks['profile_headers_unchanged'] = all((ROOT / name).is_file() and sha256(ROOT / name) == digest
                                             for name, digest in generated_headers.items())
    checks['artifacts_readable'] = not artifact_errors
    required = all(checks.values())
    logic_results = dict(suites, recovery_wrapper_nested_tests=24, offset_modified_builds=4,
                         offset_direction_comparisons=16, source_sha256=before)
    write_json(FINAL / 'logic_results.json', logic_results)
    result = {
        'start_utc': started, 'end_utc': datetime.now(timezone.utc).isoformat(),
        'reproduce_command': 'python3 tests/run_verification.py',
        'required_checks_passed': required,
        'all_tests_passed': (required and not legacy['failed'] and
                             all(group['failed'] == 0 for group in models.values()) and
                             all(suite['skipped'] == 0 for suite in suites.values())),
        'required_checks': checks, 'commands': commands,
        'exit_codes': {label: command['exit_code'] for label, command in commands.items()},
        'build': build, 'compile_matrix': {'variants': matrix.get('variants', 0),
                                          'checks': matrix.get('checks', 0), 'passed': matrix.get('passed', 0),
                                          'artifact': relative(matrix_path)},
        'golden_all_off': golden, 'units': suites, 'fatal_handlers': faults,
        'geometry_standard': models['standard'], 'geometry_stress': models['pressure'],
        'random_models': models['random'], 'enabled_model_comparison': relative(stage_path),
        'golden_reused_for_enabled_models': golden_reusable,
        'mutations': {'total': mutations.get('total', 0), 'killed': mutations.get('killed', 0),
                       'artifact': relative(mutation_path)}, 'sanitizers': sanitizers,
        'default_switches': defaults,
        'source_sha256': before, 'source_sha256_after': after,
        'source_changed_during_verification': sorted(observed_changes),
        'baseline_inputs_sha256': baseline_before, 'baseline_inputs_sha256_after': baseline_after,
        'baseline_changed_during_verification': sorted(baseline_changes),
        'profile_header_sha256': header_hashes, 'generated_config_headers_sha256': generated_headers,
        'artifact_errors': artifact_errors,
        'arm_firmware_build': 'NOT RUN', 'physical_car_test': 'NOT RUN',
        'scope': 'GCC host software/model verification; simulator assumptions do not establish real-car timing/stability',
    }
    write_json(FINAL / 'summary.json', result)
    write_json(ROOT / 'tests/verification_summary.json', result)
    final_lines = [
        'REQUIRED CHECKS: ' + ('PASS' if required else 'FAIL'),
        'ALL TESTS: ' + ('PASS' if result['all_tests_passed'] else 'FAIL / SKIPPED (see exact counts)'),
        'ALL-OFF GOLDEN: ' + ('PASS' if checks['golden_all_off'] else 'FAIL') +
        '; records=' + str(records) + '/' + str(EXPECTED_GOLDEN_RECORDS) + '; original units ' +
        str(golden_units['passed']) + '/' + str(golden_units['total']),
        'DEFAULT ORIGINAL UNITS: ' + str(legacy['passed']) + '/' + str(legacy['total']) +
        '; failed=' + str(legacy['failed']) + '; skipped=' + str(legacy['skipped']),
        'DEFAULT FIX5 UNITS: ' + str(suites['logic_default']['passed']) + '/' +
        str(suites['logic_default']['total']) + '; skipped=' + str(suites['logic_default']['skipped']),
        'PRESSURE: ' + str(models['pressure']['passed']) + '/' + str(models['pressure']['total']) +
        '; RANDOM: ' + str(models['random']['passed']) + '/' + str(models['random']['total']),
        'STM32 ARM firmware / physical car: NOT RUN',
        'Summary: ' + relative(FINAL / 'summary.json'),
    ]
    (FINAL / 'verification.log').write_text('\n'.join(final_lines) + '\n', encoding='utf-8')
    (ROOT / 'tests/verification.txt').write_text('\n'.join(final_lines) + '\n', encoding='utf-8')
    print('\n'.join(final_lines), flush=True)
    return int(not required)


if __name__ == '__main__':
    raise SystemExit(main())
