# 附件故障矩阵提取说明

输入附件：`2-test_fault_matrix.pdf`，PDF 标题为 `test_fault_matrix.py`，共 5 页。

SHA-256：`078b5d7cb38a70bb0235063cadd59ce598f7d0a28a311985e8f4e47d62c92f18`。

用 `pdftotext -layout` 与 `pdftotext -raw` 两种输出交叉核对，再保存到
`test_fault_matrix.py`。PDF 纸面宽度将 import、函数声明、条件表达式、字符串、
注释和 `return SquareConfig(...)` 强制折行，且 `return` 与返回表达式跨页。
仅恢复这些强制断行与 Python 缩进，并去除 PDF 分页字符；未改模型参数、随机
序列、循环顺序、通过条件或断言。

保留的矩阵为 7 种负载 × 2 个方向 × (无漏读种子 1 / 4% 漏读种子 1、2)
× 方形、R300 圆，共 84 例。每例最多 100 s，通过条件仍为完成至少 1.1 圈、
至少 1 圈航向转动、最终处于 RUN；ENC / LOST 停车均无法满足通过条件。

提取后先运行 `python3 -m py_compile tests/test_fault_matrix.py
tests/test_fault_random.py`，语法检查通过。此步骤不会导入 `native_api` 或运行矩阵。

扩展随机测试 `test_fault_random.py` 复用矩阵的 `LOADS` 与 `run`。将用户要求
的“五种负载”解释为 L2、BA20/26、BA26/20、BA24、heavyR，即 7 种负载中除去
空转拟合 free 和较轻 loaded L1。组合为方形 × 5 负载 × 2 个方向 × 4% / 8%
漏读 × 种子 11～13，共 60 例；不改变矩阵通过标准。

运行命令（工程根目录）：

```sh
python3 tests/test_fault_matrix.py
python3 tests/test_fault_random.py
```

起转门槛、左右响应不对称、响应时间和独立逐探头随机漏读仍属于模型假设。
GCC 原生仿真通过不能视为 ARMCC 5 编译或实车通过。
