"""New scenario families. Seeds, distributions and pass criteria are FIXED
here before any candidate firmware is evaluated (see docs/assumptions.md).

Every family has 30 seeds x 2 mirror sides. Side '+' is the geometry as
built (left turns for CCW polygons), side '-' reflects the track y -> -y.
The car itself is NOT mirrored: left motor/encoder (CPR 251) stays left and
right (CPR 265.35) stays right, so +/- is the DYNAMIC mirror test.
"""
import math, random
from track_model import SquareConfig, SquarePlant
from world import MotorParams, WorldPlant, polygon_track

FAMILIES = ('SQ2', 'DYN', 'LOW8', 'TAIL', 'BEND', 'GAP', 'ANG', 'MIX')
SEEDS = 30
BASE = {f: 30000 + 1000 * i for i, f in enumerate(FAMILIES)}
LAPS = 2.1                 # two full laps plus 0.1 lap so the 8th corner is complete
SQ2_MIN_YAW_TURNS = 1.95   # 8 corners; 18 deg tolerance for the final heading wobble
MAX_S = 200
TRACK_ERROR_LIMIT_M = 0.035
SQUARE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]

def _orig_motors(rng):
    s = SquareConfig()
    return (MotorParams(gain=s.left_gain_rpm_per_pwm * rng.uniform(.95, 1.05), dead=s.left_dead_pwm + rng.uniform(-.5, .5),
                        breakaway=s.left_breakaway_pwm + rng.uniform(-1, 1), tau=s.left_response_s * rng.uniform(.94, 1.06)),
            MotorParams(gain=s.right_gain_rpm_per_pwm * rng.uniform(.95, 1.05), dead=s.right_dead_pwm + rng.uniform(-.5, .5),
                        breakaway=s.right_breakaway_pwm + rng.uniform(-1, 1), tau=s.right_response_s * rng.uniform(.94, 1.06)))

def _dyn_motors(rng, stribeck=0.0):
    s = SquareConfig(); battery = rng.uniform(.90, 1.05); out = []
    for gain, dead in ((s.left_gain_rpm_per_pwm, s.left_dead_pwm), (s.right_gain_rpm_per_pwm, s.right_dead_pwm)):
        load = rng.uniform(3.0, 6.0); d = dead + rng.uniform(-.5, .5)
        out.append(MotorParams(gain=gain * battery * rng.uniform(.95, 1.05), dead=d, load=load,
                               breakaway=d + load + rng.uniform(3.0, 6.0), tau=rng.uniform(.10, .22),
                               spin_load=rng.uniform(1.0, 3.0), stribeck=stribeck, stribeck_rpm=8.0, coast=True,
                               slip_spin=rng.uniform(.02, .07), slip_fwd=rng.uniform(0.0, .01)))
    return tuple(out)

def _sensor(rng, noise=.06, dropout=.002):
    return dict(thresholds=[rng.uniform(.35, .65) for _ in range(8)], sensor_noise=noise, dropout=dropout)

def make(family, seed_index, side):
    """Return (plant, meta). side: +1 as built, -1 mirrored."""
    seed = BASE[family] + seed_index
    rng = random.Random(seed)
    meta = {'family': family, 'seed': seed, 'side': side}
    if family == 'SQ2':
        left, right = _orig_motors(rng)
        cfg = SquareConfig(left_gain_rpm_per_pwm=left.gain, right_gain_rpm_per_pwm=right.gain,
                           left_dead_pwm=left.dead, right_dead_pwm=right.dead,
                           left_breakaway_pwm=left.breakaway, right_breakaway_pwm=right.breakaway,
                           left_response_s=left.tau, right_response_s=right.tau)
        offset = rng.uniform(-.004, .004); heading = rng.uniform(-math.radians(1), math.radians(1))
        plant = SquarePlant(side, offset * side, heading * side, cfg)
        friction = [(rng.uniform(-.4, .4), rng.uniform(-.4, .4)) for _ in range(MAX_S // 2 + 1)]
        noise = []; prev = False
        for _ in range(MAX_S * 50):
            bit = rng.randrange(8); noisy = rng.random() < .005 and not prev
            noise.append((1 << bit) if noisy else 0); prev = noisy
        if side < 0: noise = [int(f'{n:08b}'[::-1], 2) for n in noise]
        meta.update(noise=noise, friction=friction, base_dead=(cfg.left_dead_pwm, cfg.right_dead_pwm))
        return plant, meta
    sensor_rng = random.Random(seed * 7 + 1)
    vertices = SQUARE; stubs = None; gaps = None; arcs = None; dynamic = False
    if family in ('DYN', 'LOW8', 'MIX'):
        left, right = _dyn_motors(rng, stribeck=3.0 if family == 'LOW8' else 0.0); dynamic = True
    else:
        left, right = _orig_motors(rng)
    offset = rng.uniform(-.004, .004); heading = rng.uniform(-math.radians(1), math.radians(1))
    track_w_true = .133 + (rng.uniform(-.003, .004) if dynamic else 0.0)
    sensor = _sensor(rng, dropout=.002)
    if family == 'TAIL':
        stubs = [rng.uniform(.005, .025) for _ in range(4)]
    elif family == 'BEND':
        vertices = [(0.0, 0.0), (1.2, 0.0), (1.2, 0.9), (0.0, 0.9)]
        arcs = {1: rng.uniform(.15, .30), 3: rng.uniform(.15, .30)}
    elif family == 'GAP':
        gaps = [(i, rng.uniform(.03, .15), rng.uniform(.004, .012)) for i in range(4)]
        sensor['dropout'] = .01
    elif family == 'ANG':
        vertices = [(0.0, 0.0), (1.0, 0.0), (1.0 + rng.uniform(-.12, .12), 1.0 + rng.uniform(-.12, .12)),
                    (rng.uniform(-.12, .12), 1.0 + rng.uniform(-.12, .12))]
    elif family == 'MIX':
        stubs = [rng.uniform(.005, .02) for _ in range(4)]
        arcs = {2: rng.uniform(.20, .30)}
        gaps = [(i, rng.uniform(.05, .3), rng.uniform(.004, .010)) for i in range(4)]
        sensor['dropout'] = .005
    track = polygon_track(vertices, stubs=stubs, gaps=gaps, arcs=arcs)
    if side < 0:
        track = track.mirrored(); sensor['thresholds'] = sensor['thresholds'][::-1]
        offset, heading = -offset, -heading
    plant = WorldPlant(track, left, right, dynamic=dynamic, offset=offset, heading=heading,
                       track_w_true=track_w_true, rng=sensor_rng, **sensor)
    meta['corners_per_lap'] = track.corners
    return plant, meta
