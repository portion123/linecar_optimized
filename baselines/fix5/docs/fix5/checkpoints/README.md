# 可移植基线与逐步快照

`fix4_baseline.tar.gz` 是本次输入 linecar_optimized 的完整 Git 基线归档（89bafa2），包含原源码、测试、工程与原历史文档；不是更早的 `docs/original_source/`。压缩包中的历史固件只作原证据。

`fix5_stage0_controller.tar.gz` 和 `fix5_step1_controller.tar.gz`～`fix5_step8_controller.tar.gz` 保存四个控制/异常源码文件，可结合基线与当前测试核对每步修改。对应完整测试结果在各 step 目录，提交及源码 SHA256 见 manifest.json。工作区 Git 保存完整提交与同名标签；最终收尾修复以工程顶层现行文件和 final 验证为准。

原三份不可改写的逐周期 golden 位于 ../baseline/，不在每个快照重复复制；同 GCC/O2/严格浮点选项可用 scripts/fix5_golden.py compare 再生成全关输出。
