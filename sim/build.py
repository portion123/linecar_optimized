"""Build one control-server binary per firmware variant.

Every binary compiles the variant's REAL firmware sources with
-Wall -Wextra -Wdouble-promotion -Werror (strict) and optionally ASan/UBSan.
Config overrides are passed as -D macros (flags guarded by #ifndef in
CarConfig.h); nothing in the variant tree is modified.
"""
import hashlib, os, shutil, subprocess, sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / 'sim'
BIN = SIM / 'bin'
STRICT = ['-std=c99', '-O2', '-g', '-Wall', '-Wextra', '-Wdouble-promotion', '-Werror',
          '-ffp-contract=off', '-fno-strict-aliasing']
SAN = ['-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']

# name -> (source root, probe file, strict warnings?)
VARIANTS = {
    'fix4': ('baselines/fix4', 'probe_fix4.inc', False),
    'fix4_p1p5': ('baselines/fix4_p1p5', 'probe_fix4.inc', False),
    'opt': ('baselines/opt', 'probe_opt.inc', False),
    'fix5': ('baselines/fix5', 'probe_opt.inc', False),
    'work': ('work/firmware', 'work/firmware/tests/sim_probe.inc', True),
}

def sources_digest(root):
    h = hashlib.sha256()
    for sub in ('hardware', 'system', 'user'):
        for p in sorted((REPO / root / sub).rglob('*')):
            if p.is_file() and p.suffix in ('.c', '.h'):
                h.update(str(p.relative_to(REPO / root)).encode()); h.update(p.read_bytes())
    return h.hexdigest()

def build(variant, name=None, defines=(), sanitize=False, strict=None):
    root, probe, strict_default = VARIANTS[variant]
    strict = strict_default if strict is None else strict
    name = name or variant
    if sanitize: name += '_san'
    out_dir = BIN / name
    out_dir.mkdir(parents=True, exist_ok=True)
    probe_path = (SIM / 'server' / probe) if not probe.startswith('work/') else REPO / probe
    shutil.copyfile(probe_path, out_dir / 'probe.inc')
    flags = (STRICT if strict else ['-std=c99', '-O2', '-g', '-ffp-contract=off', '-w']) + (SAN if sanitize else [])
    src = REPO / root
    cmd = ['gcc'] + flags + ['-DUSE_STDPERIPH_DRIVER', '-DSTM32F10X_HD'] + [f'-D{d}' for d in defines] + [
        '-I' + str(out_dir), '-I' + str(SIM / 'server' / 'stub'),
        '-I' + str(src / 'hardware'), '-I' + str(src / 'system'), '-I' + str(src / 'user'),
        '-I' + str(src / 'start'), '-I' + str(src / 'library'),
        str(SIM / 'server' / 'server.c'), '-o', str(out_dir / 'server'), '-lm']
    key = hashlib.sha256((' '.join(cmd) + sources_digest(root) +
                          (SIM / 'server' / 'server.c').read_text() + (out_dir / 'probe.inc').read_text()).encode()).hexdigest()
    stamp = out_dir / 'stamp'
    if not (stamp.exists() and stamp.read_text() == key and (out_dir / 'server').exists()):
        result = subprocess.run(cmd, capture_output=True, text=True)
        (out_dir / 'build.log').write_text(' '.join(cmd) + '\n' + result.stdout + result.stderr)
        if result.returncode:
            sys.stderr.write(result.stderr)
            raise SystemExit(f'build failed: {name}')
        stamp.write_text(key)
    return out_dir / 'server'

if __name__ == '__main__':
    for v in sys.argv[1:] or ['fix4', 'opt', 'fix5']:
        print(build(v))
