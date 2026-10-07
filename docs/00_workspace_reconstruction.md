# 00 工作区重建说明（云端会话）

## 1. 为什么重建

请求中引用的本地工作目录 `D:\xht\05_个人\codex\2026-10-07\files-mentioned-by-the-user-fix4`
（含 docs/09_final_report.md、02_root_cause.md、10_default_rollback.md、test_results.md、
regression_audit.md、assumptions.md、keil_final.md，以及 work/firmware、sim、golden、sim/run_safety.py、
“L/M/B–E/H”场景和44项安全测试）**不在本云端容器中**。本仓库开始时只有 README.md；
可用输入仅为：

| 输入 | 来源 | SHA256（原包） |
|---|---|---|
| FIX4 实测编码器修复版 | 用户附件 365e0e0b-…zip | 见 baselines/fix4/MANIFEST.json |
| FIX4 P1～P5 | 用户附件 c00568ae-…zip | 见 baselines/fix4_p1p5/MANIFEST.json |
| FIX5 最终版 | 本仓库 fix5_delivery 分支 deliverables/linecar_FIX5.zip | c6fd8ba5…a80cd1fb |
| opt（FIX5 的输入基线 89bafa2） | FIX5 包内 docs/fix5/checkpoints/fix4_baseline.tar.gz | 见 baselines/opt/MANIFEST.json |

因此本地记录的数字（L 30/30、M 24/30、B–E 0/30、SEARCH 23、174 个场景、44 项安全测试等）
**无法在此复核，也不与本文件的数字对应**。本次在云端重新建立了等价结构，所有场景族名称
刻意不沿用 L/M/B–E/H，避免把不同定义的结果混为一谈。若本地工作区可上传，可再与之合并。

## 2. 用户追加信息带来的基线决定

工作中用户明确：“fix5是个失败的版本”。因此：

* `work/firmware` 起点 = **FIX4（baselines/fix4，实车可跑圈）**，不是 FIX5/opt。
* FIX5、opt、FIX4-P1P5 只作只读对照基线。
* FIX4_SAFE（见 docs/assumptions.md）= FIX4 控制路径 + 硬约束要求、但 FIX4 缺失的“只会停车”的保护层。

## 3. 目录

```
baselines/{fix4,fix4_p1p5,opt,fix5}  只读原包解压（文件只读权限；MANIFEST.json 列出原包每个文件的 SHA256 及省略的大文件）
work/firmware                       开发源码（Keil 工程结构不变）
sim/server/server.c                 独立 C server：链接各版本真实 Track.c/PID.c/Motor.c/Encoder.c…，寄存器桩
sim/{ctl,legacy,world,families,runner,matrix,golden}.py  主机仿真与矩阵
golden/                             冻结的逐 run 结果与逐帧记录哈希
docs/                               中文文档
outputs/                            最终交付
tools/extract_baselines.py          从原包可重复地生成 baselines/
```

## 4. 原测试移植的一致性证明

C server 只换了传输方式（ctypes → 管道），在本机 GCC 13.3 下复现了原包记录的结果：

| 套件 | 版本 | 本次 | 原记录 |
|---|---|---|---|
| 必需几何 34 | FIX4 / FIX5 | 34/34，所有误差/时间/状态字段逐项相同 | 34/34 |
| 压力 10 | FIX4 / FIX5 | 6/10 / 9/10 | 6/10 / 9/10 |
| P1～P5 故障矩阵 84 | FIX4 | 33/84 | 33/84 |
| P1～P5 扩展随机 60 | FIX4 | 3/60 | 3/60 |
| FIX5 固定种子 30 | FIX5 | 27/30（失败 10508/10514/10522） | 27/30 |
