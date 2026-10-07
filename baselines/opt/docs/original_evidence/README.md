# 用户上传工程的历史证据

本目录保存用户原包的说明、编译日志、SHA256、几何结果与旧HEX/AXF/map。
`baseline_geometry_gcc.txt` 是修改前源码在本次GCC环境下重跑的几何基线。
其余原文中的BUILD PASS、旧日期或“可直接烧录”等只属于原FIX4文件，不能证明本次优化源码完成ARM编译。

当前验证请读工程根目录 tests/verification.txt、verification_summary.json 和 docs/最终报告.md。
历史HEX放在 firmware/Objects；它没有本次修改，不能用于验证新状态机。
