"""Per-frame trace of one run for causal analysis (raw mask -> state -> target -> command -> PWM -> encoder)."""
import argparse, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import legacy, matrix, runner
from ctl import Controller

def legacy_trace(ctl, suite, case):
    cases = {'GEO': legacy.geo_cases, 'PRS': legacy.prs_cases}[suite]()
    for label, mk, req in cases:
        if label == case:
            plant = mk(); rec = legacy.Recorder(); rec.keep = True
            frames = []
            r = ctl.start(plant.sensor_mask()); lpwm, rpwm = r.left_pwm, r.right_pwm
            for tick in range(20, 180001, 20):
                counts = plant.advance(lpwm, rpwm, dt=.02); mask = plant.sensor_mask()
                r = ctl.step(tick, counts[0], counts[1], mask); lpwm, rpwm = r.left_pwm, r.right_pwm
                frames.append((tick, plant.x, plant.y, plant.heading, plant.rpm[0], plant.rpm[1], counts[0], counts[1], mask, r))
                if not r.running: break
                if plant.laps >= 1.1 and plant.yaw_turns >= 1 and r.state == 1 and mask: break
            return frames
    raise KeyError(case)

def show(frames, t0=None, t1=None, every=1):
    print(' tick  st  mask     x      y    hdg   wL    wR  cL cR |  tgtL  tgtR  cmdL  cmdR  pwmL pwmR   err   angle  appr  dir sr att')
    for i, (tick, x, y, h, wl, wr, cl, cr, mask, r) in enumerate(frames):
        if t0 is not None and tick < t0: continue
        if t1 is not None and tick > t1: break
        if i % every: continue
        print(f'{tick:6d} {r.state:3d} {mask:08b} {x:6.3f} {y:6.3f} {math.degrees(h):6.1f} {wl:5.1f} {wr:5.1f} {cl:3d} {cr:2d} |'
              f' {r.left_target:5.1f} {r.right_target:5.1f} {r.left_command:5.1f} {r.right_command:5.1f} {r.left_pwm:4d} {r.right_pwm:4d}'
              f' {r.error:5.1f} {math.degrees(r.angle):6.1f} {r.approach:5.1f} {r.dir:3d} {r.stop_reason:2d} {r.attempts}')

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('variant'); ap.add_argument('run')
    ap.add_argument('--t0', type=int); ap.add_argument('--t1', type=int); ap.add_argument('--every', type=int, default=1)
    ap.add_argument('--states', default='')
    a = ap.parse_args()
    ctl = Controller(matrix.ensure_built(a.variant))
    parts = a.run.split('|')
    if parts[0] in ('GEO', 'PRS'): frames = legacy_trace(ctl, parts[0], parts[1])
    else:
        _, frames = runner.episode(ctl, parts[0], int(parts[1]) - __import__('families').BASE[parts[0]], 1 if parts[2] == '+' else -1, keep=True)
    if a.states:
        want = {int(s) for s in a.states.split(',')}
        frames = [f for f in frames if f[-1].state in want]
    show(frames, a.t0, a.t1, a.every)
    ctl.close()

def worst_track(frames, plant_maker):
    """Return tick of max front error while state==1 (uses a fresh plant only for geometry)."""
    plant, _ = plant_maker()
    best = (0, None)
    for f in frames:
        tick, x, y, h = f[0], f[1], f[2], f[3]
        if f[-1].state != 1: continue
        fx, fy = x + 0.175 * math.cos(h), y + 0.175 * math.sin(h)
        d = plant.track.metric_distance(fx, fy) if hasattr(plant, 'track') else plant.nearest_track(fx, fy)[0]
        if d > best[0]: best = (d, tick)
    return best
