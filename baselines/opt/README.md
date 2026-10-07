# STM32F103ZE 循迹优化源码

已基于用户FIX4工程修改；先读 [完整报告](docs/最终报告.md)，包含代码原因、修改清单、状态流程、参数表和A～F实车步骤。

- [阶段0：调用关系与A～I核实](docs/阶段0_扫描与核实.md)
- [阶段4：修改后安全审查](docs/阶段4_源码审查.md)
- [完整测试记录](tests/verification.txt) / [结果汇总](tests/verification_summary.json)

当前单元94/94、标准几何34/34、高摩擦压力9/10；保留1项右转压力失败，达到有限修正上限后停车。
**没有本次修改的ARMCC编译/链接结果或新HEX；所有实车行为仍需验证。**

原 `.uvprojx/.uvoptx` 与底层Motor/PWM/PID/GraySensor保留；251/265.35 count、44/133/175mm、PF0～7及方向宏不变。
打开 `tracking_square_continuous.uvprojx` 重新编译。Windows可使用原 `build.ps1`，需要本机ARMCC5工具链。
`Objects/` 和 `Listings/` 留空供重新构建；原HEX/AXF/map只在 `docs/original_evidence/firmware/` 作历史证据，不要把它当作优化版本。

KEY1启动/运行中停车；KEY0和KEY2立即停车。KEY5保留30%开环检查，现有15s上限和编码器保护；低速实车验证请先按报告使用默认关闭的3s闭环bench入口，完成后恢复 `TRACK_BENCH_TEST=0`。

Python3+GCC在工程根目录重跑：

```text
python3 tests/run_verification.py
```

脚本记录压力失败，`all_tests_passed=false`；必需单元/标准几何/编译检查通过不表示STM32实机验证通过。GCC库会现场编译，压缩包不带旧host二进制。

全部可调参数在 `hardware/CarConfig.h`；默认 `TRACK_DEBUG=0`、`TRACK_BENCH_TEST=0`。OLED显示状态/方向/IR/E、目标/实际RPM、PWM、角度/距离/尝试，按报告解读。
