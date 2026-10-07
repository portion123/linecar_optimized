# STM32F103ZE linecar_optimized FIX5

先读 [FIX5 完整报告](docs/fix5/FIX5报告.md)，包含真实源码核实、逐步回归、未保留为默认的候选、参数和 A～G 实车验证顺序。[最终源码审查](docs/fix5/final_source_review.md) 和 [FIX4 基线核实](docs/fix5/source_audit_baseline.md) 提供文件/行号依据。原 FIX4 报告与历史固件保留作对照，不代表当前验证结果。

默认开启 RAM Trace、统一 STOP、误差趋势观测和 ALIGN 趋势规则；正反四方向 offset/feedforward 可独立标定，致命异常先撤 PWM 再锁定。滤波、非线性位置增益、自适应速度、低速 PI 缩放、转弯转速曲线及预测默认关闭：逻辑测试通过，启用候选在完整模型中出现退化，详细失败记录没有删除。没有提高速度上限或放宽安全预算。

当前默认标准几何 34/34，压力 9/10，固定 30 种子 27 成功 / 3 安全停车 / 0 失控，与基线相同；保留 `pressure_square_-1` 及 3 个随机 ALIGN 停车失败。所有 FIX5 开关关闭时，原测试 94/94，142327 条规范输出与不可改写的 golden 零容差一致。默认统一 STOP 后，原 94 项中 4 项仍断言旧故障状态枚举（9/18），故按原门槛如实记录为 90/94，新增测试验证统一 CAR_STOP=8 的停车、首因、重启与归零语义。完整结果见 [最终验证汇总](docs/fix5/final/summary.json)。

Linux + Python 3 + 基线编译器 `gcc (Debian 14.2.0-19) 14.2.0` 在工程根目录重跑完整 FIX5 验证（精确 golden 检查会拒绝更换编译器或篡改基线）：

```text
python3 tests/run_verification.py
```

入口运行严格编译、开关矩阵、全关 golden、原测试、新逻辑、标准/压力/随机回归、异常撤 PWM、Mutation Check 和独立 ASan/UBSan。`required_checks_passed` 表示防回归门槛通过；原压力/随机失败及默认旧枚举断言仍使 `all_tests_passed=false`。Sanitizer 不参与性能或控制行为基线。源码包无需 `.git`；没有 Git 时记录版本/工作区状态不可用，并使用源码 SHA256 检查修改未泄漏，不能将不可用状态称为“工作区干净”。

打开 `tracking_square_continuous.uvprojx` 在本机 ARMCC5/Keil 重新编译；Windows 可用原 `build.ps1`。本次没有 ARMCC 编译链接、HEX、烧录或实车验证结果。历史 HEX/AXF/map 位于 `docs/original_evidence/firmware/`，不属于 FIX5 固件。几何 44/133/175 mm、编码器 251/265.35 count/rev、PF0～7、MOTOR_SIGN 和 ENCODER_SIGN 未改。

参数集中在 `hardware/CarConfig.h`。Trace 固定 200 条、约 4 秒，显式存储 6407 字节；RAM API 在 `hardware/Track.h`，不在控制期打印或写 Flash。KEY1 启动/运行中停车，KEY0/KEY2 停车，人工停车原因独立为 USER_KEY；重新启动清除首因并解冻 Trace。KEY5 保留原有限时间开环检查，闭环 bench 默认关闭，先按报告架空核对方向与最低持续转速。

`docs/fix5/checkpoints/` 保存输入基线和每步控制源码快照，Git 标签 `fix4_baseline`、`fix5_stage0`、`fix5_step1`～`fix5_step8`、`fix5_final` 保存工作区历史。源码包保留不可改写的三份基线 golden 和全部候选结果；重复的各阶段全关 replay 压缩日志可由工具再生成，打包时省略这些重复数据及 host 临时二进制；原 `history/` 大体积补丁/bundle仅在完整输入基线归档中保留，避免重复。
