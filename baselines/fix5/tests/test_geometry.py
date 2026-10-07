"""独立几何模型生成八路输入与编码器脉冲，驱动本次修改的真实 C 控制代码。
轮径/轮距/前置距/左右计数来自实测；线宽、低速电机响应和摩擦仍为假设。
"""
import argparse
import json
import math
from pathlib import Path
from native_api import fw, pwm, step, set_scenario
from track_model import SquareConfig, SquarePlant, self_check

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

def run_case(label, plant, trace=False, min_laps=1.1, max_seconds=180, require_corner_states=True):
    set_scenario(label)
    fw.Native_Start(plant.sensor_mask())
    assert fw.Track_IsRunning(), '起步探头没有看到线'
    states=set(); max_front_distance=max_all_front_distance=max_search_front_distance=0.; old_state=None
    edge_run_s=max_edge_run_s=outer_run_s=max_outer_run_s=0.; squared_front_error=0.; front_samples=0
    search_time_s=search_white_s=max_search_white_s=0.
    transitions=[]
    straight_sign=straight_flips=0
    for tick in range(20, max_seconds*1000+1, 20):
        counts=plant.advance(*pwm(), dt=.02)
        mask=plant.sensor_mask(); step(tick, mask, counts)
        state=fw.Native_State(); states.add(state)
        # Observation only: the square's axle is well inside a straight side,
        # outside the front-array corner approach. Zero correction is ignored.
        progress=plant.nearest_track()[1] % plant.config.side_m
        straight=not isinstance(plant,CirclePlant) and state==1 and .25<=progress<=.65
        correction=(fw.Native_LeftTarget()-fw.Native_RightTarget())*.5
        sign=1 if correction>1e-6 else -1 if correction < -1e-6 else 0
        if straight and sign:
            straight_flips += bool(straight_sign and sign!=straight_sign)
            straight_sign=sign
        elif not straight: straight_sign=0
        if state==1 and mask in (0x01,0x03,0x07,0x80,0xC0,0xE0):
            edge_run_s+=.02; max_edge_run_s=max(max_edge_run_s,edge_run_s)
        else: edge_run_s=0.
        # 1/2路最外侧才达到软件的持续偏差>=6接管门槛；3路边线允许连续过弯。
        if state==1 and mask in (0x01,0x03,0x80,0xC0):
            outer_run_s+=.02; max_outer_run_s=max(max_outer_run_s,outer_run_s)
        else: outer_run_s=0.
        if state != old_state:
            transition={'time_s':round(plant.time_s,2),'state':state,'mask':hex(mask),
                        'x':round(plant.x,4),'y':round(plant.y,4),
                        'heading_deg':round(math.degrees(plant.heading),1),
                        'goal_mm':round(fw.Native_Goal(),2),'angle_deg':round(math.degrees(fw.Native_Angle()),1)}
            transitions.append(transition)
            if trace: print(label,transition,flush=True)
        old_state=state
        x=plant.x+plant.config.front_offset_m*math.cos(plant.heading)
        y=plant.y+plant.config.front_offset_m*math.sin(plant.heading)
        front_distance=plant.nearest_track(x,y)[0]
        max_all_front_distance=max(max_all_front_distance,front_distance)
        if state==1:
            max_front_distance=max(max_front_distance,front_distance)
            squared_front_error+=front_distance**2; front_samples+=1
        if state==2:
            search_time_s+=.02; max_search_front_distance=max(max_search_front_distance,front_distance)
            if not mask:
                search_white_s+=.02; max_search_white_s=max(max_search_white_s,search_white_s)
            else: search_white_s=0.
        else: search_white_s=0.
        if not fw.Track_IsRunning(): break
        if plant.laps>=min_laps and plant.yaw_turns>=1 and state==1 and mask: break
    result={'case':label,'time_s':round(plant.time_s,2),'laps':round(plant.laps,3),
            'yaw_turns':round(plant.yaw_turns,3),'state':state,'mask':hex(mask),
            'stop_reason':fw.Native_StopReason(),
            'states_seen':sorted(states),'corners_passed':plant.completed_corners,
            'max_run_front_error_mm':round(max_front_distance*1000,2),
            'max_all_front_error_mm':round(max_all_front_distance*1000,2),
            'rms_run_front_error_mm':round(math.sqrt(squared_front_error/max(1,front_samples))*1000,2),
            'max_search_front_error_mm':round(max_search_front_distance*1000,2),
            'search_total_time_s':round(search_time_s,2),
            'longest_search_all_white_ms':round(max_search_white_s*1000),
            'max_continuous_edge_run_ms':round(max_edge_run_s*1000),
            'max_continuous_outer_run_ms':round(max_outer_run_s*1000),
            'longest_all_white_ms':round(plant.longest_zero_sensor_s*1000),
            'required_corner_states':require_corner_states and not isinstance(plant,CirclePlant)}
    result['straight_correction_sign_flips']=straight_flips
    result['safety_stop']=not bool(fw.Track_IsRunning()) and fw.Native_StopReason()!=0
    result['uncontrolled']=bool(fw.Track_IsRunning()) and plant.time_s>=max_seconds-.001
    passed=plant.laps>=min_laps and plant.yaw_turns>=1 and state==1
    if isinstance(plant,CirclePlant):
        # 保持25mm轨迹边界；持续外侧允许600ms接管加400ms柔和接回，不能无限贴边。
        passed=passed and max_all_front_distance<.025 and 14 not in states and max_outer_run_s<=1.05
    else:
        passed=passed and plant.completed_corners>=4 and max_front_distance<.035
        if require_corner_states: passed=passed and {11,14}.issubset(states)
    result['passed']=passed
    print(('PASS ' if passed else 'FAIL ')+json.dumps(result,ensure_ascii=False),flush=True)
    return result,plant,transitions

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--quick',action='store_true')
    parser.add_argument('--trace',action='store_true')
    parser.add_argument('--stress',action='store_true',help='另记录人为高摩擦压力，不纳入标准通过结论')
    args=parser.parse_args(); self_check()
    results=[]; histories=[]
    for direction in (1,-1):
        cases=[(0,0,.020,.175)] if args.quick else [
            (0,0,.020,.175),(-.015,0,.020,.175),(.015,0,.020,.175),
            (0,-5,.020,.175),(0,5,.020,.175),(0,0,.014,.175),
            (0,0,.028,.175),(0,0,.040,.175),
            (0,0,.020,.155),(0,0,.020,.195)]
        for offset,angle,width,front in cases:
            label=f'square_{direction:+d}_offset{offset*1000:g}_angle{angle}_width{width*1000:g}_front{front*1000:g}'
            plant=SquarePlant(direction,offset,math.radians(angle),SquareConfig(line_width_m=width,front_offset_m=front))
            require_corner_states=offset==0 and angle==0 and width in (.028,.040) and front==.175
            result,p,transitions=run_case(label,plant,args.trace,require_corner_states=require_corner_states)
            results.append(result); histories.append({'case':label,'poses':p.pose_history,'transitions':transitions})
        for radius in ([.60] if args.quick else [.20,.25,.30,.50,.60]):
            label=f'circle_{direction:+d}_radius{radius*1000:g}'
            result,p,transitions=run_case(label,CirclePlant(direction,radius),args.trace)
            results.append(result); histories.append({'case':label,'poses':p.pose_history,'transitions':transitions})
    if not args.quick:
        # 低速落地负载尚未实测：额外验证更低增益/更高起转门槛的独立假设。
        loaded=SquareConfig(left_gain_rpm_per_pwm=14,right_gain_rpm_per_pwm=13,
                            left_dead_pwm=9,right_dead_pwm=11,
                            left_breakaway_pwm=17,right_breakaway_pwm=18,
                            left_response_s=.18,right_response_s=.24)
        for direction in (1,-1):
            for label,plant in [(f'loaded_square_{direction:+d}',SquarePlant(direction,config=loaded)),
                                (f'loaded_circle_{direction:+d}_radius600',CirclePlant(direction,.60,loaded))]:
                result,p,transitions=run_case(label,plant,args.trace,require_corner_states=False)
                results.append(result); histories.append({'case':label,'poses':p.pose_history,'transitions':transitions})
    Path(__file__).with_name('geometry_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    Path(__file__).with_name('geometry_traces.json').write_text(json.dumps(histories,ensure_ascii=False),encoding='utf-8')
    if args.stress:
        config=SquareConfig(left_gain_rpm_per_pwm=12,right_gain_rpm_per_pwm=11,
                            left_dead_pwm=13,right_dead_pwm=15,
                            left_breakaway_pwm=20,right_breakaway_pwm=22,
                            left_response_s=.22,right_response_s=.28)
        pressure=[]
        for direction in (1,-1):
            for radius in (.20,.25,.30,.35):
                result,_,_=run_case(f'pressure_circle_{direction:+d}_radius{radius*1000:g}',
                                    CirclePlant(direction,radius,config),args.trace)
                pressure.append(result)
            result,_,_=run_case(f'pressure_square_{direction:+d}',SquarePlant(direction,config=config),
                                args.trace,require_corner_states=False)
            pressure.append(result)
        Path(__file__).with_name('pressure_results.json').write_text(
            json.dumps({'required':False,'note':'人为高摩擦假设，非实车参数，失败须作为局限报告',
                        'parameters':vars(config),'results':pressure},ensure_ascii=False,indent=2),encoding='utf-8')
    assert all(r['passed'] for r in results), '至少一个几何场景未通过，见结果与轨迹'
    print(f'PASS: {len(results)} required independent geometry cases; NOT a physical-car test')
if __name__=='__main__': main()
