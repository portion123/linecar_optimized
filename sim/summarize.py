"""Per-family summary of a matrix result (pass, stops, SEARCH, overshoot, withdraw delay, TRACK error)."""
import json, statistics, sys
from collections import defaultdict

def pct(v, q):
    if not v: return float('nan')
    v = sorted(v); return v[min(len(v) - 1, int(q * (len(v) - 1) + .5))]

def summary(path):
    d = json.load(open(path)); out = {}
    g = defaultdict(list)
    for r in d['families']: g[r['family'] + ('+' if r['side'] > 0 else '-')].append(r)
    for k, rs in g.items():
        ov = [x for r in rs for x in r['overshoot_deg']]; wd = [x for r in rs for x in r['withdraw_ms']]
        out[k] = {'pass': sum(r['passed'] for r in rs), 'n': len(rs), 'stops': sum(r['stop_reason'] not in (0,) for r in rs),
                  'search': sum(r['search_entries'] for r in rs), 'white_search': sum(r['white_search'] for r in rs),
                  'search_s': round(sum(r['search_time_s'] for r in rs), 1), 'turns': sum(r['turn_entries'] for r in rs),
                  'approach': sum(r['approach_entries'] for r in rs),
                  'ov_p50': round(pct(ov, .5), 1), 'ov_p95': round(pct(ov, .95), 1), 'ov_max': round(max(ov), 1) if ov else None,
                  'wd_p50': pct(wd, .5), 'wd_p95': pct(wd, .95),
                  'trk_max_p50': round(pct([r['max_track_err_mm'] for r in rs], .5), 1),
                  'trk_max_max': round(max(r['max_track_err_mm'] for r in rs), 1),
                  'rms_p50': round(pct([r['rms_track_err_mm'] for r in rs], .5), 2),
                  'straight_rms_p50': round(pct([r.get('straight_rms_mm', float('nan')) for r in rs], .5), 2),
                  'straight_flips': sum(r.get('straight_flips', 0) for r in rs),
                  'straight_peak_max': round(max(r.get('straight_peak_mm', 0) for r in rs), 1),
                  'stop_frames': sum(r.get('stop_frames', 0) for r in rs),
                  'pwm_step_max': max(r.get('max_pwm_step', 0) for r in rs),
                  'corner_s_p50': round(pct([x for r in rs for x in r.get('corner_time_s', [])], .5), 2)}
    leg = {s: f"{sum(x['passed'] for x in v)}/{len(v)}" for s, v in d['legacy'].items()}
    return leg, out

if __name__ == '__main__':
    for p in sys.argv[1:]:
        leg, out = summary(p); print('==', p, leg)
        print(f"{'fam':7s} pass  stop srch wsrch srch_s turns appr  ov50 ov95 ovmax  wd50 wd95  trkMax50 trkMaxMax rms50")
        for k, v in out.items():
            print(f"{k:7s} {v['pass']:2d}/{v['n']} {v['stops']:4d} {v['search']:4d} {v['white_search']:5d} {v['search_s']:6.1f} {v['turns']:5d} {v['approach']:4d} "
                  f"{v['ov_p50']:5} {v['ov_p95']:4} {v['ov_max']!s:5} {v['wd_p50']!s:5} {v['wd_p95']!s:5} {v['trk_max_p50']:8} {v['trk_max_max']:9} {v['rms_p50']}")
