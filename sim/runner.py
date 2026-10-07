"""Closed-loop episode runner + metrics shared by every variant."""
import hashlib, math
import families as F
from track_model import SquarePlant

TURN_STATES = (11, 16)
STRONG_SPIN_RPM = 12.0   # requested half-difference above this with counter-rotation = strong turn

def _contiguous(m):
    if not m: return False
    while not m & 1: m >>= 1
    return (m & (m + 1)) == 0

def edge_index_cw(edge):
    """SquarePlant direction -1 traverses edges 2,1,0,3 (y=s start, heading +x)."""
    return {2: 0, 1: 1, 0: 2, 3: 3}[edge]

def episode(ctl, family, seed_index, side, keep=False):
    plant, meta = F.make(family, seed_index, side)
    square = isinstance(plant, SquarePlant)
    h = hashlib.sha256(); frames = []
    r = ctl.start(plant.sensor_mask() if square else plant.sensor_mask(clean=True)); h.update(ctl.raw)
    lpwm, rpwm = r.left_pwm, r.right_pwm
    m = dict(search_entries=0, search_time_s=0.0, search_max_deg=0.0, turn_entries=0, approach_entries=0,
             exit_entries=0, align_entries=0, edge_entries=0, max_track_err_mm=0.0, sq_err=0.0, n_err=0,
             withdraw_ms=[], overshoot_deg=[], max_pwm=0, white_search=0)
    lpwm, rpwm = r.left_pwm, r.right_pwm
    prev_state = r.state; credible_t = None; turn_dir = 0
    corner = None   # [outgoing heading, turn sign, max overshoot deg] from APPROACH/TURN entry until TRACK
    for k in range(1, F.MAX_S * 50 + 1):
        tick = k * 20
        if square:
            delta = meta['friction'][(k - 1) // 100]
            plant.config.left_dead_pwm = meta['base_dead'][0] + delta[0]
            plant.config.right_dead_pwm = meta['base_dead'][1] + delta[1]
        counts = plant.advance(lpwm, rpwm, .02)
        mask = plant.sensor_mask() ^ (meta['noise'][k - 1] if square else 0)
        r = ctl.step(tick, counts[0], counts[1], mask); h.update(ctl.raw)
        lpwm, rpwm = r.left_pwm, r.right_pwm; st = r.state
        m['max_pwm'] = max(m['max_pwm'], abs(lpwm), abs(rpwm))
        if keep:
            frames.append((tick, plant.x, plant.y, plant.heading, plant.wheels[0].rpm if not square else plant.rpm[0],
                           plant.wheels[1].rpm if not square else plant.rpm[1], counts[0], counts[1], mask, r))
        if st != prev_state:
            if st == 2:
                m['search_entries'] += 1
                if prev_state in (15, 17, 1): m['white_search'] += 1
            elif st in TURN_STATES and prev_state not in TURN_STATES: m['turn_entries'] += 1; credible_t = None
            elif st == 14: m['approach_entries'] += 1
            elif st == 15: m['exit_entries'] += 1
            elif st == 17: m['align_entries'] += 1
            if st in (14,) + TURN_STATES and corner is None:
                if square:
                    edge = plant.nearest_track()[2]; sgn = plant.direction
                    h_out = (edge + 1) * math.pi / 2 * sgn if sgn > 0 else -(edge_index_cw(edge) + 1) * math.pi / 2
                else:
                    _, _, idx, h_in = plant.track.path.nearest(plant.x, plant.y)
                    nxt = plant.track.path.pieces[(idx + 1) % len(plant.track.path.pieces)]
                    h_out = math.atan2(nxt[4] - nxt[2], nxt[3] - nxt[1]) if nxt[0] == 'L' else h_in
                    sgn = 1 if ((h_out - h_in + math.pi) % (2 * math.pi) - math.pi) > 0 else -1
                corner = [h_out, sgn, -180.0]
            if st == 1 and corner is not None:
                m['overshoot_deg'].append(round(corner[2], 1)); corner = None
        if corner is not None and st in TURN_STATES + (15, 16, 17, 2):
            rel = (plant.heading - corner[0] + math.pi) % (2 * math.pi) - math.pi
            corner[2] = max(corner[2], math.degrees(rel) * corner[1])
        if square:
            fx = plant.x + plant.config.front_offset_m * math.cos(plant.heading)
            fy = plant.y + plant.config.front_offset_m * math.sin(plant.heading)
            err = plant.nearest_track(fx, fy)[0]
        else:
            err = plant.front_error()[0]
        if st == 1:
            m['max_track_err_mm'] = max(m['max_track_err_mm'], err * 1000); m['sq_err'] += err * err; m['n_err'] += 1
        if st == 2:
            m['search_time_s'] += .02; m['search_max_deg'] = max(m['search_max_deg'], abs(math.degrees(r.angle)))
        if st in TURN_STATES:
            spin = (r.left_target - r.right_target) * 0.5 * (1 if r.dir > 0 else -1)
            counter = r.left_target * r.right_target < 0
            if credible_t is None and mask and _contiguous(mask) and bin(mask).count('1') <= 4 \
                    and abs(math.degrees(r.angle)) >= 45:
                credible_t = tick
            if credible_t is not None and credible_t >= 0 and not (counter and spin > STRONG_SPIN_RPM):
                m['withdraw_ms'].append(tick - credible_t); credible_t = -1
        elif credible_t is not None and credible_t >= 0:
            m['withdraw_ms'].append(tick - credible_t); credible_t = -1
        prev_state = st
        if not r.running: break
        if plant.laps >= F.LAPS and st == 1 and mask: break
    laps = plant.laps
    passed = laps >= F.LAPS and st == 1 and r.running and m['max_track_err_mm'] < F.TRACK_ERROR_LIMIT_M * 1000
    if square: passed = passed and plant.completed_corners >= 8 and plant.yaw_turns >= F.SQ2_MIN_YAW_TURNS
    res = {'family': family, 'seed': meta['seed'], 'side': side, 'passed': bool(passed),
           'end_state': st, 'stop_reason': r.stop_reason, 'running': bool(r.running),
           'time_s': round(k * .02, 2), 'laps': round(laps, 3),
           'rms_track_err_mm': round(math.sqrt(m.pop('sq_err') / max(1, m.pop('n_err'))) * 1000, 2),
           'frames': k + 1, 'sha256': h.hexdigest()}
    m['max_track_err_mm'] = round(m['max_track_err_mm'], 2); m['search_time_s'] = round(m['search_time_s'], 2)
    m['search_max_deg'] = round(m['search_max_deg'], 1)
    res.update(m)
    return (res, frames) if keep else res
