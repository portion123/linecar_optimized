"""Golden records and regression comparison.

golden/<name>.json keeps, per run: original pass flag, per-frame record SHA256,
frame count, end state/stop reason and key metrics.  compare() reports
  * BLOCKING: runs passed by the reference but failed by the candidate
    (checked per run, so equal totals with a changed pass set are caught)
  * gained passes, frame-hash equality, SEARCH / stop / overshoot deltas.
"""
import json, sys
from pathlib import Path

KEEP = ('passed', 'sha256', 'frames', 'end_state', 'stop_reason', 'search_entries', 'search_time_s',
        'turn_entries', 'approach_entries', 'max_track_err_mm', 'rms_track_err_mm', 'overshoot_deg',
        'withdraw_ms', 'white_search', 'search_max_deg', 'time_s', 'laps', 'max_pwm', 'state')

def runs(result):
    out = {}
    for suite, items in result['legacy'].items():
        for r in items: out[f"{suite}|{r['case']}"] = {k: r[k] for k in KEEP if k in r}
    for r in result['families']:
        key = f"{r['family']}|{r['seed']}|{'+' if r['side'] > 0 else '-'}"
        out[key] = {k: r[k] for k in KEEP if k in r}
    return out

def save(result_path, golden_path, note=''):
    res = json.loads(Path(result_path).read_text())
    g = {'variant': res['variant'], 'note': note, 'runs': runs(res)}
    Path(golden_path).parent.mkdir(parents=True, exist_ok=True)
    Path(golden_path).write_text(json.dumps(g, indent=0, sort_keys=True))
    return g

def load(path):
    d = json.loads(Path(path).read_text())
    return d['runs'] if 'runs' in d else runs(d)

def group(key): return key.split('|')[0] + (key.split('|')[2] if key.count('|') == 2 else '')

def compare(ref, cand):
    keys = sorted(set(ref) | set(cand))
    missing = [k for k in keys if k not in cand or k not in ref]
    lost = [k for k in keys if k in ref and k in cand and ref[k]['passed'] and not cand[k]['passed']]
    gained = [k for k in keys if k in ref and k in cand and not ref[k]['passed'] and cand[k]['passed']]
    same_hash = sum(1 for k in keys if k in ref and k in cand and ref[k].get('sha256') == cand[k].get('sha256'))
    groups = {}
    for k in keys:
        if k not in ref or k not in cand: continue
        g = groups.setdefault(group(k), {'n': 0, 'ref_pass': 0, 'cand_pass': 0, 'ref_search': 0, 'cand_search': 0,
                                          'ref_stop': 0, 'cand_stop': 0})
        g['n'] += 1; g['ref_pass'] += ref[k]['passed']; g['cand_pass'] += cand[k]['passed']
        g['ref_search'] += ref[k].get('search_entries', 0); g['cand_search'] += cand[k].get('search_entries', 0)
        g['ref_stop'] += ref[k].get('stop_reason', 0) not in (0, None); g['cand_stop'] += cand[k].get('stop_reason', 0) not in (0, None)
    return {'total': len(keys), 'missing': missing, 'lost_passes_BLOCKING': lost, 'gained_passes': gained,
            'identical_frame_hash': same_hash, 'groups': groups}

def report(ref_path, cand_path, show=20):
    c = compare(load(ref_path), load(cand_path))
    print(f"runs {c['total']}  identical-frame-hash {c['identical_frame_hash']}  missing {len(c['missing'])}")
    print(f"LOST passes (BLOCKING): {len(c['lost_passes_BLOCKING'])}  gained: {len(c['gained_passes'])}")
    for k in c['lost_passes_BLOCKING'][:show]: print('  LOST', k)
    for k in c['gained_passes'][:show]: print('  GAIN', k)
    for g, v in c['groups'].items():
        if v['ref_pass'] != v['cand_pass'] or v['ref_search'] != v['cand_search'] or v['ref_stop'] != v['cand_stop']:
            print(f"  {g:12s} pass {v['ref_pass']}->{v['cand_pass']}/{v['n']}  SEARCH {v['ref_search']}->{v['cand_search']}  stops {v['ref_stop']}->{v['cand_stop']}")
    return c

if __name__ == '__main__':
    if sys.argv[1] == 'save': save(sys.argv[2], sys.argv[3], ' '.join(sys.argv[4:]))
    else: report(sys.argv[2], sys.argv[3])
