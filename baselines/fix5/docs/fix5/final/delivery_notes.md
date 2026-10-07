# 交付内容与复现口径

工程顶层是最终 FIX5 源码；四个生产源码文件实际修改，Keil 引用和真实几何/符号不变。不可改写的 FIX4 三份 golden、各候选成功/失败结果、最终完整验证、逐步控制快照和完整输入基线均随包提供。

源码包省略54份各阶段重复的 `all_off/*_golden.jsonl.gz` replay，保留这些阶段的对比JSON/日志，以及 baseline 和 final 两组三份完整数据。工具可以重新生成逐周期 replay；没有删除任何候选失败指标。省略顶层旧 `history/` 大体积补丁/bundle的重复副本：完整输入备份 `../checkpoints/fix4_baseline.tar.gz` 内仍保存它们。host共享库、临时profile与缓存不打包，不提供新的HEX。

Linux + Python 3 + `gcc (Debian 14.2.0-19) 14.2.0` 执行 `python3 tests/run_verification.py`。精确golden要求原编译器和原浮点选项；更换编译器会被明确拒绝，不得重写golden。没有 `.git` 也可复验：版本/工作区状态标记为不可用，源SHA256仍验证没有修改泄漏。

`required_checks_passed=true` 表示默认防回归门槛、逻辑、编译/内存和Mutation检查通过；`all_tests_passed=false` 如实保留4项原状态枚举断言失败、原压力/随机失败以及关闭实验功能的跳过。不能将host结果称为ARM编译/烧录或实车稳定。

已实际完成无`.git`独立导出树全入口复验：18项必需检查通过，117份源码hash与主验证一致，版本状态记录不可用；见 export_replay.json / export_replay.log。另在提交后的干净Git工作区Mutation重跑8/8，无修改残留，见 mutations_clean.json。
