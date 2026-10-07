"""Acceptance gate for one candidate result.

  python3 gate.py results/<tag>/<cand>.json [--ref results/<tag>/<prev_default>.json] [--off]

Checks
  * vs golden/baseline_fix4.json : every FIX4-passed run must still pass (BLOCKING floor)
  * vs golden/fix4_safe.json     : --off => full per-frame hash must be identical for every run
                                   otherwise reports how many runs are core-identical
  * vs --ref (previous default)  : lost passes are BLOCKING; gained passes listed
  * metrics per family: pass, stops, SEARCH, turns, overshoot, withdraw delay, TRACK error, mirror +/-
"""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import golden, summarize

ROOT = Path(__file__).resolve().parents[1]

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('cand'); ap.add_argument('--ref'); ap.add_argument('--off', action='store_true')
    ap.add_argument('--json'); a = ap.parse_args()
    cand = golden.load(a.cand)
    fix4 = golden.load(ROOT / 'golden/baseline_fix4.json'); safe = golden.load(ROOT / 'golden/fix4_safe.json')
    out = {'candidate': a.cand}
    lost_fix4 = [k for k in fix4 if fix4[k]['passed'] and not cand[k]['passed']]
    out['fix4_pass_total'] = sum(v['passed'] for v in fix4.values())
    out['cand_pass_total'] = sum(v['passed'] for v in cand.values())
    out['lost_vs_fix4'] = lost_fix4
    out['gained_vs_fix4'] = [k for k in fix4 if not fix4[k]['passed'] and cand[k]['passed']]
    out['identical_full_vs_fix4_safe'] = sum(safe[k]['sha256'] == cand[k]['sha256'] for k in safe)
    out['identical_core_vs_fix4_safe'] = sum(safe[k].get('core256') == cand[k].get('core256') for k in safe)
    out['runs'] = len(safe)
    if a.ref:
        ref = golden.load(a.ref)
        out['lost_vs_ref'] = [k for k in ref if ref[k]['passed'] and not cand[k]['passed']]
        out['gained_vs_ref'] = [k for k in ref if not ref[k]['passed'] and cand[k]['passed']]
    print(f"runs {out['runs']}  pass {out['cand_pass_total']} (FIX4 {out['fix4_pass_total']})")
    print(f"vs FIX4 floor: lost {len(lost_fix4)}  gained {len(out['gained_vs_fix4'])}")
    for k in lost_fix4: print('   LOST(FIX4)', k, 'stop', cand[k].get('stop_reason'))
    print(f"vs FIX4_SAFE golden: full-hash identical {out['identical_full_vs_fix4_safe']}/{out['runs']}, core identical {out['identical_core_vs_fix4_safe']}/{out['runs']}")
    if a.off and out['identical_full_vs_fix4_safe'] != out['runs']:
        print('   OFF-EQUIVALENCE FAILED'); out['off_equivalence'] = False
    elif a.off: out['off_equivalence'] = True; print('   OFF-EQUIVALENCE OK')
    if a.ref:
        print(f"vs ref {a.ref}: lost {len(out['lost_vs_ref'])}  gained {len(out['gained_vs_ref'])}")
        for k in out['lost_vs_ref']: print('   LOST(ref)', k, 'stop', cand[k].get('stop_reason'))
    leg, fam = summarize.summary(a.cand)
    out['legacy'] = leg; out['families'] = fam
    print('legacy', leg)
    print(f"{'fam':7s} pass  stop srch wsrch turns  ov50 ov95  wd50 wd95 trkMaxMax rms50")
    for k, v in fam.items():
        print(f"{k:7s} {v['pass']:2d}/{v['n']} {v['stops']:4d} {v['search']:4d} {v['white_search']:5d} {v['turns']:5d} "
              f"{v['ov_p50']:5} {v['ov_p95']:4} {v['wd_p50']!s:5} {v['wd_p95']!s:5} {v['trk_max_max']:9} {v['rms_p50']}")
    tot = {s: sum(v[s] for v in fam.values()) for s in ('pass', 'stops', 'search', 'white_search', 'turns')}
    plus = sum(v['pass'] for k, v in fam.items() if k.endswith('+')); minus = sum(v['pass'] for k, v in fam.items() if k.endswith('-'))
    print(f"families total: pass {tot['pass']}  stops {tot['stops']}  SEARCH {tot['search']} (after white {tot['white_search']})  mirror +{plus} / -{minus}")
    out['family_totals'] = tot; out['mirror'] = {'+': plus, '-': minus}
    if a.json: Path(a.json).write_text(json.dumps(out, indent=1))
    blocking = bool(lost_fix4) or (a.ref and bool(out['lost_vs_ref'])) or (a.off and not out['off_equivalence'])
    return 1 if blocking else 0

if __name__ == '__main__':
    raise SystemExit(main())
