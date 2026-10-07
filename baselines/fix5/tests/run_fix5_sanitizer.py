"""Build and run a standalone ASan/UBSan executable (never a metrics build).

Reproduce with: python3 tests/run_fix5_sanitizer.py
Optional: --config-header <header> to check another FIX5 switch configuration.
Build failures, sanitizer reports and exercise assertions all fail the runner.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler', default='gcc')
    parser.add_argument('--config-header', type=Path)
    parser.add_argument('--output', type=Path, help='optional JSON result artifact')
    args = parser.parse_args()
    version = subprocess.run([args.compiler, '--version'], cwd=ROOT, check=True,
                             text=True, capture_output=True).stdout.splitlines()[0]
    command = [args.compiler, '-std=c99', '-O1', '-g', '-Wall', '-Wextra',
               '-Wdouble-promotion', '-Werror', '-ffp-contract=off',
               '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
               '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
               '-DUSE_STDPERIPH_DRIVER', '-DSTM32F10X_HD',
               '-Itests/native', '-Istart', '-Ilibrary', '-Iuser', '-Ihardware', '-Isystem']
    if args.config_header:
        command += ['-include', str(args.config_header.resolve())]
    command += ['tests/fix5_sanitizer.c', '-lm']
    # An executable avoids preloading ASan into the Python/ctypes baseline
    # process. Non-PIE makes the shadow mapping deterministic on Linux hosts.
    environment = os.environ.copy()
    environment['ASAN_OPTIONS'] = 'detect_leaks=1:halt_on_error=1:abort_on_error=1'
    environment['UBSAN_OPTIONS'] = 'halt_on_error=1:print_stacktrace=1'
    with tempfile.TemporaryDirectory(prefix='linecar-fix5-sanitizer-') as directory:
        executable = Path(directory) / 'fix5_sanitizer'
        build = subprocess.run(command + ['-o', str(executable)], cwd=ROOT,
                               text=True, capture_output=True)
        result = {'compiler': version, 'build_command': command + ['-o', '<temporary>/fix5_sanitizer'],
                  'build_exit_code': build.returncode, 'build_stdout': build.stdout,
                  'build_stderr': build.stderr,
                  'purpose': 'memory bounds and undefined behaviour only; no physics or timing metrics'}
        if build.returncode == 0:
            run = subprocess.run([str(executable)], cwd=ROOT, env=environment,
                                 text=True, capture_output=True)
            result.update(run_exit_code=run.returncode, run_stdout=run.stdout,
                          run_stderr=run.stderr, sanitizer_environment={
                              key: environment[key] for key in ('ASAN_OPTIONS', 'UBSAN_OPTIONS')})
            if run.returncode == 0:
                result['exercise'] = json.loads(run.stdout)
        success = result['build_exit_code'] == 0 and result.get('run_exit_code') == 0
        result['status'] = 'passed' if success else 'failed'
        text = json.dumps(result, indent=2, ensure_ascii=False) + '\n'
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(text, encoding='utf-8')
        print(text, end='')
        return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
