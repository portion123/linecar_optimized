# 01 FIX4_SAFE：FIX4 控制路径 + 只停车的安全层

## 1. 声明

**FIX4_SAFE = `work/firmware` 在本提交时的源码**（所有新增功能 flag 不存在/为 0）。
它的逐帧记录哈希冻结在 `golden/fix4_safe.json`，以后每个 work 提交在全部新 flag=0 时必须逐帧复现。

FIX4_SAFE 相对 FIX4（baselines/fix4，实车可跑圈）只增加硬约束要求、但 FIX4 缺少的保护；
这些保护**只增加停车条件，不改变任何控制输出**：

| 项 | FIX4 | FIX4_SAFE | 代码 |
|---|---|---|---|
| 首个停车原因 | 无（只有 LOST/ENC 状态） | `TrackStopReason` 19 种，首因锁存到下次 KEY1/KEY5；运行中按键=USER(1)，已故障停车后再按键不覆盖 | Track.h、Track.c `Halt/Track_Stop/ResetRuntime` |
| 调度间隔 >40 ms | 清 PID/宽线后继续 | 锁定 STOP（新状态 18 TIME GAP，原因 6） | Track.c Control |
| 编码器反向 250 ms | 一帧零计数即清零（低速间歇反向脉冲永远累计不到 250 ms） | 只在确有反向脉冲的帧累计；同向脉冲清零；**零计数=待定，不加不清**（零计数不证明持续反转） | Track.c EncoderFault |
| APPROACH | 6 s | 6 s + 实际编码器距离 >200 mm 停（原因 8）+ 累计偏航 >15° 停（原因 9） | Track.c Control |
| Recovery 会话 | 无 | 首个全白帧/推进/扫描开始；跨假恢复累计：12 s、4 次尝试、300°、700 mm；仅正常 RUN 窄线稳定 600 ms **且** 前进 30 mm 才清除 | Track.c RecoveryBegin/Step/Attempt |
| SEARCH/CORNER/边缘对线 | 10 s/60°/两侧；12 s/105°；35°/2.5 s | 不变，只补停车原因 10–15 | Track.c Scan |
| 自动 PWM 上限 | 30%（助推 18/22%） | 不变 | — |
| OLED | — | 停车时右上角显示 `Rnn` 首因 | Track.c Display |

源文件保持原 BOM 与逐行 CRLF/LF（tools/srcedit.py），diff 仅 +126/−23 行。

## 2. 与 FIX4 的逐帧比较（698 run 全矩阵）

比较字段：时间、状态、PWM、mask、方向、运行标志、扫描侧、左右目标/命令 RPM、误差、角度、滤波转速、推进距离/目标
（排除只有观测意义的停车原因/尝试数等字段）。

* 661/698 run **逐帧完全相同**（场景构造修正后）；
* 其余 37 run：**每一个都与 FIX4 逐帧相同，直到 FIX4_SAFE 因新保护停车的那一帧**（`sim/equiv.py`，证据
  `golden/fix4_safe_vs_fix4_equivalence.json`）：
  * APPROACH 偏航 >15°（原因 9）：28 run，其中 13 个是 FIX4 原本“通过”的
    （GEO loaded_square ±1、FMX L1 方形 5 个、FRD L2 1 个、DYN 31009+、LOW8 32005−/32007±/32024+）；
  * Recovery 累计转角 300°（原因 18）：9 run，全部是 FIX4 原本就失败的 MIX。

## 3. 这 13 个“FIX4 通过但超出 15° 推进偏航预算”的 run

trace（`python3 sim/trace.py work 'GEO|loaded_square_+1' --states 12,14`）：负载模型下左轮响应慢、
起转门槛高，FIX4 推进阶段左右都只请求 50 RPM、没有偏航修正；左轮持续 ~38 RPM、右轮 ~52 RPM，
165 mm 推进中偏航累计 >15°（FIX4 原版继续推进，随后 55–105° 的转角扫描范围把这个偏差“吃掉”）。
硬约束规定推进偏航 15° 不可放宽，因此 FIX4_SAFE 在这里停车是正确行为；
这 13 个 run 作为后续“推进偏航保持”改进必须找回的阻断项（不得以放宽 15° 的方式找回）。
