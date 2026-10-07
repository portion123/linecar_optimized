"""FIX5 fixed-seed stress observer; original plant equations are unchanged.

Thirty seeds were declared before evaluating FIX5. Parameter perturbations are
model assumptions, not measurements. Safety STOP is a bounded outcome and is
reported separately from timeout with motors still enabled (uncontrolled).
"""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import random
from native_api import fw, pwm, step, set_scenario
from track_model import SquareConfig, SquarePlant

SEEDS = tuple(range(10500, 10530))
DT = .02
MAX_SECONDS = 180
MIN_LAPS = 1.1

def run_seed(seed):
    rng = random.Random(seed)
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
    initial_cfg=vars(cfg).copy()
    offset=rng.uniform(-.004,.004); heading=rng.uniform(-math.radians(1),math.radians(1))
    plant=SquarePlant(direction,offset,heading,cfg)
    # Generate all disturbances in advance: early STOP cannot change the stream
    # of random draws or bias subsequent parameter / noise scheduling.
    friction=[(rng.uniform(-.4,.4),rng.uniform(-.4,.4)) for _ in range(MAX_SECONDS//2+1)]
    noise=[]; previous_noisy=False
    for _ in range(int(MAX_SECONDS/DT)):
        bit=rng.randrange(8)
        noisy=rng.random()<.005 and not previous_noisy
        noise.append((1<<bit) if noisy else 0); previous_noisy=noisy
    set_scenario('random_seed_'+str(seed))
    fw.Native_Start(plant.sensor_mask())
    states=set(); maximum=maximum_track=0.; sign_prev=flips=0; noise_frames=0
    for index,tick in enumerate(range(20,MAX_SECONDS*1000+1,20)):
        # Slowly varying friction within the predeclared +/-0.4 PWM band;
        # controller cannot read the plant's configuration or these samples.
        delta=friction[index//100]
        cfg.left_dead_pwm=initial_cfg['left_dead_pwm']+delta[0]
        cfg.right_dead_pwm=initial_cfg['right_dead_pwm']+delta[1]
        counts=plant.advance(*pwm(),dt=DT)
        mask=plant.sensor_mask() ^ noise[index]; noise_frames+=bool(noise[index])
        step(tick,mask,counts); state=fw.Native_State(); states.add(state)
        x=plant.x+cfg.front_offset_m*math.cos(plant.heading)
        y=plant.y+cfg.front_offset_m*math.sin(plant.heading)
        deviation=plant.nearest_track(x,y)[0]
        maximum=max(maximum,deviation)
        if state==1: maximum_track=max(maximum_track,deviation)
        along=plant.nearest_track()[1]%cfg.side_m
        straight=state==1 and .25<=along<=.65
        correction=.5*(fw.Native_LeftTarget()-fw.Native_RightTarget())
        sign=1 if correction>1e-6 else -1 if correction < -1e-6 else 0
        if straight and sign:
            flips+=bool(sign_prev and sign!=sign_prev); sign_prev=sign
        elif not straight: sign_prev=0
        if not fw.Track_IsRunning(): break
        if plant.laps>=MIN_LAPS and plant.yaw_turns>=1 and state==1 and mask: break
    passed=plant.laps>=MIN_LAPS and plant.yaw_turns>=1 and state==1 and plant.completed_corners>=4 and maximum_track<.035
    running=bool(fw.Track_IsRunning())
    uncontrolled=running and plant.time_s>=MAX_SECONDS-.001
    return {'case':'random_seed_'+str(seed),'seed':seed,'direction':direction,
        'passed':passed,'safety_stop':not running and fw.Native_StopReason()!=0,
        'uncontrolled':uncontrolled,'state':state,'stop_reason':fw.Native_StopReason(),
        'time_s':round(plant.time_s,2),'laps':round(plant.laps,3),
        'max_run_front_error_mm':round(maximum_track*1000,2),
        'max_all_front_error_mm':round(maximum*1000,2),
        'straight_correction_sign_flips':flips,'noise_frames_applied':noise_frames,
        'states_seen':sorted(states),'initial_offset_mm':offset*1000,
        'initial_heading_deg':math.degrees(heading),'initial_motor_parameters':initial_cfg}

def summary(results):
    values=lambda name:sorted(r[name] for r in results)
    def distribution(name):
        data=values(name)
        return {'min':data[0],'p50':.5*(data[14]+data[15]),'p90':data[26],'max':data[-1]}
    return {'total':len(results),'successes':sum(r['passed'] for r in results),
        'safety_stops':sum(r['safety_stop'] for r in results),
        'uncontrolled':sum(r['uncontrolled'] for r in results),
        'max_run_front_error_mm':distribution('max_run_front_error_mm'),
        'max_all_front_error_mm':distribution('max_all_front_error_mm'),
        'time_s':distribution('time_s'),
        'straight_correction_sign_flips':distribution('straight_correction_sign_flips')}

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,
        default=Path(__file__).with_name('fix5_random_results.json'))
    args=parser.parse_args(); results=[run_seed(seed) for seed in SEEDS]
    report={'seed_policy':'fixed before FIX5; no tuning to seed success',
        'seeds':list(SEEDS),'model':'unmodified tests/track_model.py SquarePlant equations',
        'perturbations':{'motor_gain_percent':5,'dead_pwm':.5,'breakaway_pwm':1,
            'response_percent':6,'initial_offset_mm':4,'initial_heading_deg':1,
            'time_varying_friction_dead_pwm':.4,'single_frame_noise_probability':.005},
        'pass_standard':'original square: >=1.1 laps, >=1 yaw turn, TRACK, >=4 corners, TRACK front error <35mm',
        'uncontrolled_definition':'still running at original 180s geometry timeout',
        'summary':summary(results),'results':results}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report['summary'],ensure_ascii=False,indent=2))
    return 0

if __name__=='__main__':raise SystemExit(main())
