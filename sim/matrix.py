"""Run the full scenario matrix for one or more variants in parallel.

Variant spec: 'fix4' | 'opt' | 'fix5' | 'fix4_p1p5' | 'work' | 'work:FLAG=1,OTHER=0'
Writes results/<tag>/<variant>.json : {'legacy': {...}, 'families': [...]}
Every result keeps the SHA256 of the per-frame controller records.
"""
import argparse, json, os, sys, time
from multiprocessing import Pool
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import build, families as F, legacy, runner
from ctl import Controller

SIM = Path(__file__).resolve().parent
LEGACY_SUITES = ('GEO', 'PRS', 'FMX', 'FRD', 'F5R')

def parse(spec):
    if ':' not in spec: return spec, []
    base, flags = spec.split(':', 1)
    return base, [f for f in flags.split(',') if f]

def bin_name(spec, sanitize=False):
    base, defines = parse(spec)
    name = base if not defines else base + '__' + '_'.join(d.replace('=', '') for d in defines)
    return name + ('_san' if sanitize else '')

def ensure_built(spec, sanitize=False):
    base, defines = parse(spec)
    name = base if not defines else base + '__' + '_'.join(d.replace('=', '') for d in defines)
    return build.build(base, name=name, defines=defines, sanitize=sanitize)

_ctl = {}
def _controller(binary):
    if binary not in _ctl:
        env = dict(os.environ, ASAN_OPTIONS='detect_leaks=1:abort_on_error=1', UBSAN_OPTIONS='print_stacktrace=1:halt_on_error=1')
        _ctl[binary] = Controller(binary, env=env)
    return _ctl[binary]

def _job(args):
    binary, kind, a, b, c = args
    ctl = _controller(binary)
    if kind == 'family':
        return ('family', runner.episode(ctl, a, b, c))
    return ('legacy', (a, legacy.run_suite(ctl, a)))

def _close():
    for c in _ctl.values(): c.close()

def run_variant(spec, families=F.FAMILIES, seeds=F.SEEDS, legacy_suites=LEGACY_SUITES, sanitize=False, procs=4):
    binary = str(ensure_built(spec, sanitize))
    jobs = [(binary, 'legacy', s, None, None) for s in legacy_suites]
    jobs += [(binary, 'family', f, i, side) for f in families for i in range(seeds) for side in (1, -1)]
    out = {'variant': spec, 'binary': binary, 'sanitize': sanitize, 'legacy': {}, 'families': []}
    with Pool(procs) as pool:
        for kind, payload in pool.imap_unordered(_job, jobs, chunksize=4):
            if kind == 'legacy': out['legacy'][payload[0]] = payload[1]
            else: out['families'].append(payload)
        pool.map(_shutdown_worker, range(procs * 4))
    out['families'].sort(key=lambda r: (F.FAMILIES.index(r['family']), r['seed'], -r['side']))
    return out

def _shutdown_worker(_):
    _close(); return 0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('variants', nargs='+'); ap.add_argument('--tag', default='latest')
    ap.add_argument('--families', default=','.join(F.FAMILIES)); ap.add_argument('--seeds', type=int, default=F.SEEDS)
    ap.add_argument('--legacy', default=','.join(LEGACY_SUITES)); ap.add_argument('--sanitize', action='store_true')
    ap.add_argument('--procs', type=int, default=4)
    a = ap.parse_args()
    fams = tuple(x for x in a.families.split(',') if x); leg = tuple(x for x in a.legacy.split(',') if x)
    outdir = SIM / 'results' / a.tag; outdir.mkdir(parents=True, exist_ok=True)
    for spec in a.variants:
        t = time.time()
        res = run_variant(spec, fams, a.seeds, leg, a.sanitize, a.procs)
        res['elapsed_s'] = round(time.time() - t, 1)
        (outdir / (bin_name(spec, a.sanitize) + '.json')).write_text(json.dumps(res, indent=0))
        fam = {}
        for r in res['families']:
            k = f"{r['family']}{'+' if r['side'] > 0 else '-'}"; fam.setdefault(k, [0, 0]); fam[k][0] += r['passed']; fam[k][1] += 1
        print(spec, f"{res['elapsed_s']}s", {s: f"{sum(x['passed'] for x in v)}/{len(v)}" for s, v in res['legacy'].items()},
              {k: f'{p}/{n}' for k, (p, n) in fam.items()}, flush=True)

if __name__ == '__main__':
    main()
