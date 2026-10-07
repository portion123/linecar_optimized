"""Frame-level equivalence: for each run whose core hash differs between a
reference and a candidate, replay both, record every control record and
report the first divergent frame and the candidate's state there.
Usage: python3 equiv.py REF_SPEC CAND_SPEC results/<tag>/<ref>.json results/<tag>/<cand>.json
"""
import json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import families as F, golden, legacy, matrix, runner
from ctl import Controller, core_bytes, Record, _REC

class RecCtl(Controller):
    def __init__(self, binary):
        super().__init__(binary); self.log = []
    def _call(self, payload):
        r = super()._call(payload); self.log.append(self.raw); return r

def replay(ctl, key):
    ctl.log = []
    suite = key.split('|')[0]
    if suite in F.FAMILIES:
        _, seed, side = key.split('|')
        runner.episode(ctl, suite, int(seed) - F.BASE[suite], 1 if side == '+' else -1)
    else:
        case = key.split('|', 1)[1]
        if suite in ('GEO', 'PRS'):
            for l, mk, req in (legacy.geo_cases() if suite == 'GEO' else legacy.prs_cases()):
                if l == case: legacy.geometry_case(ctl, l, mk(), require_corner_states=req)
        elif suite in ('FMX', 'FRD'):
            for l, mk, d, s in (legacy.fmx_cases() if suite == 'FMX' else legacy.frd_cases()):
                if l == case: legacy.fault_case(ctl, l, mk(), d, s)
        elif suite == 'F5R':
            legacy.f5r_case(ctl, int(case.split('_')[-1]))
    return list(ctl.log)

def first_divergence(a, b):
    for i, (x, y) in enumerate(zip(a, b)):
        if core_bytes(x) != core_bytes(y): return i
    return None if len(a) == len(b) else min(len(a), len(b))

def check(ref_spec, cand_spec, ref_json, cand_json, allowed_reasons):
    ra = golden.runs(json.loads(Path(ref_json).read_text())); rb = golden.runs(json.loads(Path(cand_json).read_text()))
    diff = [k for k in ra if ra[k].get('core256') != rb[k].get('core256')]
    A = RecCtl(matrix.ensure_built(ref_spec)); B = RecCtl(matrix.ensure_built(cand_spec))
    out = []
    for k in diff:
        la, lb = replay(A, k), replay(B, k)
        i = first_divergence(la, lb)
        rec = Record._make(_REC.unpack(lb[i])) if i is not None and i < len(lb) else None
        ok = rec is not None and not rec.running and rec.stop_reason in allowed_reasons and i == len(lb) - 1
        out.append({'run': k, 'first_divergent_frame': i, 'cand_frames': len(lb), 'ref_frames': len(la),
                    'cand_state': rec.state if rec else None, 'cand_stop_reason': rec.stop_reason if rec else None,
                    'only_new_protection_stop': bool(ok)})
    A.close(); B.close()
    return out

if __name__ == '__main__':
    allowed = {6, 8, 9, 16, 17, 18, 19, 4, 5}   # gap, approach dist/yaw, recovery budgets, pending-reverse
    res = check(*sys.argv[1:5], allowed)
    bad = [r for r in res if not r['only_new_protection_stop']]
    print(f'{len(res)} differing runs; {len(res) - len(bad)} identical up to a new-protection STOP frame; {len(bad)} other')
    for r in res: print(r)
    Path(sys.argv[5] if len(sys.argv) > 5 else '/dev/null').write_text(json.dumps(res, indent=1))
