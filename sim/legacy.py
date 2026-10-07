"""Original test suites ported verbatim onto the control server.

Inputs, seeds, geometry, plant equations, time limits and pass criteria are
copied from the baselines (FIX4 tests/test_geometry.py, FIX4-P1P5
tests/test_fault_matrix.py / test_fault_random.py, FIX5 tests/fix5_random.py).
Only the transport changed (ctypes -> control server). Each case returns a
dict with the ORIGINAL pass flag plus the SHA256 of the per-frame records.
"""
import hashlib, math, random
from dataclasses import replace
from track_model import SquareConfig, SquarePlant

class CirclePlant(SquarePlant):
    def __init__(self, direction=1, radius=.60, config=None):
        self.radius = radius
        super().__init__(direction=direction, config=config)
        self.x = math.sqrt(radius**2 - self.config.front_offset_m**2)
        self.y = 0.0
        self.heading = direction * math.pi / 2
        self.initial_heading = self.heading
        self.initial_progress_m = self.previous_progress_m = self.nearest_track()[1]
        self.pose_history = [(0.0, self.x, self.y, self.heading)]
    @property
    def perimeter_m(self): return 2 * math.pi * self.radius
    @property
    def completed_corners(self): return 0
    def nearest_track(self, x=None, y=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        return abs(math.hypot(x, y) - self.radius), (math.atan2(y, x) % (2*math.pi))*self.radius, 0

from ctl import core_bytes

class Recorder:
    def __init__(self):
        self.h = hashlib.sha256(); self.core = hashlib.sha256(); self.frames = 0; self.records = []; self.keep = False
    def add(self, ctl, rec):
        self.h.update(ctl.raw); self.core.update(core_bytes(ctl.raw)); self.frames += 1
        if self.keep: self.records.append(rec)

def geometry_case(ctl, label, plant, min_laps=1.1, max_seconds=180, require_corner_states=True, rec=None):
    rec = rec or Recorder()
    r = ctl.start(plant.sensor_mask()); rec.add(ctl, r)
    assert r.running, label
    states = set(); max_front = max_all = 0.; edge_run = max_edge = outer_run = max_outer = 0.
    sq = 0.; n = 0; search_time = 0.
    lpwm, rpwm = r.left_pwm, r.right_pwm
    for tick in range(20, max_seconds*1000+1, 20):
        counts = plant.advance(lpwm, rpwm, dt=.02)
        mask = plant.sensor_mask(); r = ctl.step(tick, counts[0], counts[1], mask); rec.add(ctl, r)
        lpwm, rpwm = r.left_pwm, r.right_pwm
        state = r.state; states.add(state)
        if state == 1 and mask in (0x01,0x03,0x07,0x80,0xC0,0xE0):
            edge_run += .02; max_edge = max(max_edge, edge_run)
        else: edge_run = 0.
        if state == 1 and mask in (0x01,0x03,0x80,0xC0):
            outer_run += .02; max_outer = max(max_outer, outer_run)
        else: outer_run = 0.
        x = plant.x+plant.config.front_offset_m*math.cos(plant.heading)
        y = plant.y+plant.config.front_offset_m*math.sin(plant.heading)
        d = plant.nearest_track(x, y)[0]; max_all = max(max_all, d)
        if state == 1: max_front = max(max_front, d); sq += d*d; n += 1
        if state == 2: search_time += .02
        if not r.running: break
        if plant.laps >= min_laps and plant.yaw_turns >= 1 and state == 1 and mask: break
    passed = plant.laps >= min_laps and plant.yaw_turns >= 1 and state == 1
    if isinstance(plant, CirclePlant):
        passed = passed and max_all < .025 and 14 not in states and max_outer <= 1.05
    else:
        passed = passed and plant.completed_corners >= 4 and max_front < .035
        if require_corner_states: passed = passed and {11, 14}.issubset(states)
    return {'case': label, 'passed': bool(passed), 'time_s': round(plant.time_s, 2), 'laps': round(plant.laps, 3),
            'yaw_turns': round(plant.yaw_turns, 3), 'state': state, 'stop_reason': r.stop_reason,
            'states_seen': sorted(states), 'corners_passed': plant.completed_corners,
            'max_run_front_error_mm': round(max_front*1000, 2), 'max_all_front_error_mm': round(max_all*1000, 2),
            'rms_run_front_error_mm': round(math.sqrt(sq/max(1, n))*1000, 2),
            'search_total_time_s': round(search_time, 2), 'max_continuous_outer_run_ms': round(max_outer*1000),
            'frames': rec.frames, 'sha256': rec.h.hexdigest(), 'core256': rec.core.hexdigest()}

def geo_cases():
    out = []
    for direction in (1, -1):
        for offset, angle, width, front in [(0,0,.020,.175),(-.015,0,.020,.175),(.015,0,.020,.175),
                (0,-5,.020,.175),(0,5,.020,.175),(0,0,.014,.175),(0,0,.028,.175),(0,0,.040,.175),
                (0,0,.020,.155),(0,0,.020,.195)]:
            label = f'square_{direction:+d}_offset{offset*1000:g}_angle{angle}_width{width*1000:g}_front{front*1000:g}'
            req = offset == 0 and angle == 0 and width in (.028, .040) and front == .175
            out.append((label, (lambda d=direction, o=offset, a=angle, w=width, f=front:
                        SquarePlant(d, o, math.radians(a), SquareConfig(line_width_m=w, front_offset_m=f))), req))
        for radius in [.20, .25, .30, .50, .60]:
            out.append((f'circle_{direction:+d}_radius{radius*1000:g}', (lambda d=direction, R=radius: CirclePlant(d, R)), True))
    loaded = SquareConfig(left_gain_rpm_per_pwm=14, right_gain_rpm_per_pwm=13, left_dead_pwm=9, right_dead_pwm=11,
                          left_breakaway_pwm=17, right_breakaway_pwm=18, left_response_s=.18, right_response_s=.24)
    for direction in (1, -1):
        out.append((f'loaded_square_{direction:+d}', (lambda d=direction: SquarePlant(d, config=loaded)), False))
        out.append((f'loaded_circle_{direction:+d}_radius600', (lambda d=direction: CirclePlant(d, .60, loaded)), False))
    return out

def prs_cases():
    config = SquareConfig(left_gain_rpm_per_pwm=12, right_gain_rpm_per_pwm=11, left_dead_pwm=13, right_dead_pwm=15,
                          left_breakaway_pwm=20, right_breakaway_pwm=22, left_response_s=.22, right_response_s=.28)
    out = []
    for direction in (1, -1):
        for radius in (.20, .25, .30, .35):
            out.append((f'pressure_circle_{direction:+d}_radius{radius*1000:g}', (lambda d=direction, R=radius: CirclePlant(d, R, config)), True))
        out.append((f'pressure_square_{direction:+d}', (lambda d=direction: SquarePlant(d, config=config)), False))
    return out

# ---- FIX4 P1-P5 fault matrix / random (user PDF test) ----
def _cfg(break_l, break_r, gain_l=14, gain_r=13, dead_l=9, dead_r=11, resp_l=.18, resp_r=.24):
    return SquareConfig(left_gain_rpm_per_pwm=gain_l, right_gain_rpm_per_pwm=gain_r, left_dead_pwm=dead_l,
                        right_dead_pwm=dead_r, left_breakaway_pwm=break_l, right_breakaway_pwm=break_r,
                        left_response_s=resp_l, right_response_s=resp_r)
LOADS = {'free': SquareConfig(), 'L1': _cfg(17, 18), 'L2': _cfg(20, 22, 12, 11, 13, 15, .22, .28),
         'BA20/26': _cfg(20, 26), 'BA26/20': _cfg(26, 20), 'BA24': _cfg(24, 24),
         'heavyR': _cfg(18, 24, 14, 11, 9, 13, .18, .30)}

def fault_case(ctl, label, plant, dropout=0.0, seed=1, seconds=100, laps=1.1, rec=None):
    rec = rec or Recorder(); rnd = random.Random(seed)
    r = ctl.start(plant.sensor_mask()); rec.add(ctl, r)
    if not r.running: return {'case': label, 'passed': False, 'end': 'NOSTART', 'time_s': 0.0}
    lpwm, rpwm = r.left_pwm, r.right_pwm
    for tick in range(20, seconds*1000+1, 20):
        counts = plant.advance(lpwm, rpwm, dt=.02)
        mask = plant.sensor_mask()
        for bit in range(8):
            if dropout and mask & (1 << bit) and rnd.random() < dropout:
                mask &= ~(1 << bit)
        r = ctl.step(tick, counts[0], counts[1], mask); rec.add(ctl, r)
        lpwm, rpwm = r.left_pwm, r.right_pwm
        if not r.running: break
        if plant.laps >= laps and plant.yaw_turns >= 1 and r.state == 1 and mask: break
    ok = plant.laps >= laps and plant.yaw_turns >= 1 and r.state == 1
    return {'case': label, 'passed': bool(ok), 'end_state': r.state, 'stop_reason': r.stop_reason,
            'time_s': round(plant.time_s, 1), 'frames': rec.frames, 'sha256': rec.h.hexdigest(), 'core256': rec.core.hexdigest()}

def fmx_cases():
    out = []
    for name, config in LOADS.items():
        for direction in (1, -1):
            for dropout, seeds in ((0.0, (1,)), (0.04, (1, 2))):
                for seed in seeds:
                    for kind in ('square', 'circle_R300'):
                        mk = (lambda d=direction, c=config: SquarePlant(d, config=c)) if kind == 'square' else \
                             (lambda d=direction, c=config: CirclePlant(d, .30, c))
                        out.append((f'{kind} {name} dir{direction:+d} dropout{dropout} seed{seed}', mk, dropout, seed))
    return out

def frd_cases():
    out = []
    for name in ('L2', 'BA20/26', 'BA26/20', 'BA24', 'heavyR'):
        for direction in (1, -1):
            for dropout in (.04, .08):
                for seed in (11, 12, 13):
                    out.append((f'square {name} dir{direction:+d} dropout{dropout} seed{seed}',
                                (lambda d=direction, c=LOADS[name]: SquarePlant(d, config=c)), dropout, seed))
    return out

# ---- FIX5 fixed-seed random (seeds 10500..10529) ----
def f5r_case(ctl, seed, rec=None):
    DT = .02; MAX_SECONDS = 180; MIN_LAPS = 1.1
    rec = rec or Recorder(); rng = random.Random(seed)
    direction = 1 if seed % 2 == 0 else -1
    standard = SquareConfig()
    cfg = replace(standard,
        left_gain_rpm_per_pwm=standard.left_gain_rpm_per_pwm*rng.uniform(.95,1.05),
        right_gain_rpm_per_pwm=standard.right_gain_rpm_per_pwm*rng.uniform(.95,1.05),
        left_dead_pwm=standard.left_dead_pwm+rng.uniform(-.5,.5),
        right_dead_pwm=standard.right_dead_pwm+rng.uniform(-.5,.5),
        left_breakaway_pwm=standard.left_breakaway_pwm+rng.uniform(-1,1),
        right_breakaway_pwm=standard.right_breakaway_pwm+rng.uniform(-1,1),
        left_response_s=standard.left_response_s*rng.uniform(.94,1.06),
        right_response_s=standard.right_response_s*rng.uniform(.94,1.06))
    initial_cfg = vars(cfg).copy()
    offset = rng.uniform(-.004,.004); heading = rng.uniform(-math.radians(1), math.radians(1))
    plant = SquarePlant(direction, offset, heading, cfg)
    friction = [(rng.uniform(-.4,.4), rng.uniform(-.4,.4)) for _ in range(MAX_SECONDS//2+1)]
    noise = []; previous_noisy = False
    for _ in range(int(MAX_SECONDS/DT)):
        bit = rng.randrange(8)
        noisy = rng.random() < .005 and not previous_noisy
        noise.append((1 << bit) if noisy else 0); previous_noisy = noisy
    r = ctl.start(plant.sensor_mask()); rec.add(ctl, r)
    maximum_track = 0.; lpwm, rpwm = r.left_pwm, r.right_pwm; states = set()
    for index, tick in enumerate(range(20, MAX_SECONDS*1000+1, 20)):
        delta = friction[index//100]
        cfg.left_dead_pwm = initial_cfg['left_dead_pwm']+delta[0]
        cfg.right_dead_pwm = initial_cfg['right_dead_pwm']+delta[1]
        counts = plant.advance(lpwm, rpwm, dt=DT)
        mask = plant.sensor_mask() ^ noise[index]
        r = ctl.step(tick, counts[0], counts[1], mask); rec.add(ctl, r)
        lpwm, rpwm = r.left_pwm, r.right_pwm; state = r.state; states.add(state)
        x = plant.x+cfg.front_offset_m*math.cos(plant.heading); y = plant.y+cfg.front_offset_m*math.sin(plant.heading)
        if state == 1: maximum_track = max(maximum_track, plant.nearest_track(x, y)[0])
        if not r.running: break
        if plant.laps >= MIN_LAPS and plant.yaw_turns >= 1 and state == 1 and mask: break
    passed = plant.laps >= MIN_LAPS and plant.yaw_turns >= 1 and state == 1 and plant.completed_corners >= 4 and maximum_track < .035
    return {'case': f'random_seed_{seed}', 'seed': seed, 'passed': bool(passed), 'state': state,
            'stop_reason': r.stop_reason, 'time_s': round(plant.time_s, 2), 'laps': round(plant.laps, 3),
            'states_seen': sorted(states), 'max_run_front_error_mm': round(maximum_track*1000, 2),
            'frames': rec.frames, 'sha256': rec.h.hexdigest(), 'core256': rec.core.hexdigest()}

def run_suite(ctl, suite):
    """Return list of result dicts for one original suite."""
    if suite == 'GEO':
        return [geometry_case(ctl, l, mk(), require_corner_states=req) for l, mk, req in geo_cases()]
    if suite == 'PRS':
        return [geometry_case(ctl, l, mk(), require_corner_states=req) for l, mk, req in prs_cases()]
    if suite == 'FMX':
        return [fault_case(ctl, l, mk(), d, s) for l, mk, d, s in fmx_cases()]
    if suite == 'FRD':
        return [fault_case(ctl, l, mk(), d, s) for l, mk, d, s in frd_cases()]
    if suite == 'F5R':
        return [f5r_case(ctl, s) for s in range(10500, 10530)]
    raise KeyError(suite)
