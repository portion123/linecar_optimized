"""扩展随机故障验证：五种较重负载 × 方形双向 × 漏读4%/8% × 种子11～13。

沿用附件矩阵的负载和通过条件；所有电机响应与漏读仍是模型假设。
用法（工程根目录）：python3 tests/test_fault_random.py
"""
import json
from pathlib import Path

from test_fault_matrix import LOADS, SquarePlant, run


LOAD_NAMES = ('L2', 'BA20/26', 'BA26/20', 'BA24', 'heavyR')


def main():
    results = []
    for name in LOAD_NAMES:
        for direction in (1, -1):
            for dropout in (.04, .08):
                for seed in (11, 12, 13):
                    label = f'square {name} dir{direction:+d} dropout{dropout} seed{seed}'
                    result = run(label, SquarePlant(direction, config=LOADS[name]), dropout, seed)
                    results.append(result)
                    print(('PASS ' if result['ok'] else 'FAIL ') +
                          json.dumps(result, ensure_ascii=False), flush=True)
    Path(__file__).with_name('fault_random_results.json').write_text(
        json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    passed = sum(result['ok'] for result in results)
    print(f'{"PASS" if passed == len(results) else "FAIL"}: {passed}/{len(results)} 扩展随机场景通过')
    assert len(results) == 60, '扩展随机场景数量不符'
    assert all(result['ok'] for result in results), '存在失败场景'


if __name__ == '__main__':
    main()
