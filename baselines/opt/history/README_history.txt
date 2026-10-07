Git历史恢复：
git clone history/control_history.bundle restored_project

原始基线：b3faca7；当前最终源码：2a25b4e。
五个实现步骤均有独立提交，现版本有完整测试和中文报告。
changes_from_original.patch 是相对用户原包基线的完整Git补丁，原工程本身已修改，无需再次应用。
历史HEX/AXF只属于旧FIX4；新源码需重新Keil编译。
