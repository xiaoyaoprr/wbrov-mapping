# 候选4：采集与持久化积压检查

本目录只包含源码、构建日志、测试回执及容量估算，不包含整段录像。当前8号未切换。

- `round6_candidate4/result.json`：Jetson构建结果与输入/二进制哈希。
- `round6_candidate4/camera_tests.log`：16项CTest，其中新增50×24MiB元数据积压、1500个微型帧序列检查；不分配或写入GB级图像。
- `round6_candidate4_negative/`：同一新增测试使用旧512MiB/32段头文件，按预期在积压断言失败；`dependencies.txt`核对实际头文件来源。负对照不代表候选失败。
- `package_round6_candidate4_result.json`：32项Python测试、入口check-only、独立候选包哈希。
- `round6_backlog_model.json`：以实机v2提交耗时和段大小估算容量，不是实机验收或缺失图像重建。
- `recompute_backlog_model.py`、`closed_segment_sizes.json`：配合上一轮公开逐帧台账和提交耗时可复算上述235段容量估计。
- `sha256.json`：从Jetson取回的20个原始文件哈希。其他本地脚本与说明由仓库总MANIFEST覆盖。

构建启动器第一次误用现场回执的phase枚举，被断言挡住、未启动；修正为真实的closed和exit0后才启动一次。详情见`startup_harness_failure.json`。未复写此前失败现场结果。

实机v3失败已收录`field_v3/`：7533帧全部保存核验，但5次采集空档、最大10.483秒；待同步峰值636MB低于2GiB，扩大文件额度不足。实际RGB、点云和三轴显示通过，未收到本轮移动完成回复。建图各子进程exit0，外层因退出身份观察返回1，不能称完整正常退出。该版本未部署8号；后续候选5改变压缩/磁盘线程边界，需独立验证。
