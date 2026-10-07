"""FIX4 实车故障复现矩阵：起转门槛/左右不对称负载
× 探头漏读 × 方形(双向直角)/圆弯。
用法（在工程根目录）：python3 tests/test_fault_matrix.py
只驱动真实 Track.c（tests/native_api.py 用 gcc 原生编译），电机和几何用
tests/track_model.py。
通过 = 跑满 1.1 圈、转向满 1 圈、结束时仍是 RUN；任何 ENC 停车 / LOST 停车都算失败。
注意：起转门槛、响应时间等是模型假设，不是实车测量；通过不等于实车通过。
"""
import math, random, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from native_api import fw, pwm, step
from track_model import SquareConfig, SquarePlant
from test_geometry import CirclePlant


STATE = {0: 'READY', 1: 'RUN', 2: 'SEARCH', 4: 'NO_LINE', 8: 'LOST', 9: 'ENC',
         10: 'FWD', 11: 'CORNER', 12: 'WIDE', 14: 'APPR', 15: 'EXIT'}


def cfg(break_l, break_r, gain_l=14, gain_r=13, dead_l=9, dead_r=11, resp_l=.18,
        resp_r=.24):
    return SquareConfig(left_gain_rpm_per_pwm=gain_l, right_gain_rpm_per_pwm=gain_r,
                        left_dead_pwm=dead_l, right_dead_pwm=dead_r,
                        left_breakaway_pwm=break_l, right_breakaway_pwm=break_r,
                        left_response_s=resp_l, right_response_s=resp_r)


LOADS = {
    'free': SquareConfig(),                         # 原模型：空转拟合
    'L1': cfg(17, 18),                              # 原测试里的 loaded
    'L2': cfg(20, 22, 12, 11, 13, 15, .22, .28),      # 原压力场景的高摩擦
    'BA20/26': cfg(20, 26),                          # 右轮起转门槛明显更高
    'BA26/20': cfg(26, 20),                          # 左轮起转门槛明显更高
    'BA24': cfg(24, 24),                             # 两轮都要 24% 才起转
    'heavyR': cfg(18, 24, 14, 11, 9, 13, .18, .30),   # 右轮又重又慢
}


def run(label, plant, dropout=0.0, seed=1, seconds=100, laps=1.1):
    rnd = random.Random(seed)
    fw.Native_Start(plant.sensor_mask())
    if not fw.Track_IsRunning():
        return dict(case=label, ok=False, end='NOSTART', t=0.0)
    for tick in range(20, seconds * 1000 + 1, 20):
        counts = plant.advance(*pwm(), dt=.02)
        mask = plant.sensor_mask()
        for bit in range(8):  # 每个亮着的探头每帧以 dropout 概率漏读
            if dropout and mask & (1 << bit) and rnd.random() < dropout:
                mask &= ~(1 << bit)
        step(tick, mask, counts)
        if not fw.Track_IsRunning():
            break
        if plant.laps >= laps and plant.yaw_turns >= 1 and fw.Native_State() == 1 and mask:
            break
    ok = plant.laps >= laps and plant.yaw_turns >= 1 and fw.Native_State() == 1
    return dict(case=label, ok=ok, end=STATE.get(fw.Native_State(), fw.Native_State()),
                t=round(plant.time_s, 1))


def main():
    total = passed = 0
    failures = []
    for name, config in LOADS.items():
        for direction in (1, -1):
            for dropout, seeds in ((0.0, (1,)), (0.04, (1, 2))):
                for seed in seeds:
                    for kind in ('square', 'circle_R300'):
                        plant = SquarePlant(direction, config=config) if kind == 'square' \
                            else CirclePlant(direction, .30, config)
                        result = run(f'{kind} {name} dir{direction:+d} dropout{dropout} seed{seed}',
                                     plant, dropout, seed)
                        total += 1; passed += result['ok']
                        if not result['ok']:
                            failures.append(result)
    for r in failures:
        print(f"FAIL {r['case']}: 结束于 {r['end']}，t={r['t']}s")
    print(f'{"PASS" if passed == total else "FAIL"}: {passed}/{total} 故障矩阵场景通过')
    assert passed == total, '存在失败场景'


if __name__ == '__main__':
    main()
