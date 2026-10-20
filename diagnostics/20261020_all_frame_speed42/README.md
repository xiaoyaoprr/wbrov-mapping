# 候选42离线证据

这份材料对应保留Tag/无Tag恢复路径的全帧处理候选42。当前是源码及离线验证更新，现场验收和8号切换待完成。报告见 `docs/ALL_FRAME_SPEED42_20261020.md`。

- `parallel20_*`、`parallel21_plain_result.json`：投影搜索双线程构建、固定随机对照、普通随机复核。
- `voxel40_*`：紧凑点云索引的边界、回退、内存检查和关闭地图组件测试。
- `voxel41_validation_result.json`、`voxel42_plain_result.json`：相同输出全帧对照和普通随机复核。
- `candidate42_package_validation_result.json`：独立包验证，不代表已部署。
- `entry.json`、`source_binding.json`、`source_comparison.json`、`tested-source-export.zip`：运行配置、实际编译源和仓库对应。导出中复制的辅助文件不一定为编译依赖；报告说明旧录像头文件未覆盖。
- `indices115_buildfix_result.json`：未达门槛的后续试验，不能当42改动。
- `edges117_source_audit.json`：后续只读归因，没有117候选或回放。
- `finish_edges117_audit_1.json`：更新前的设备空闲、包与8号哈希核对。

固定诊断和普通随机结果单独保留。不得以固定诊断的逐位一致替代普通环境验证，也不得把普通环境不同地图工作量的墙钟差全部归因于优化。不含原始录像或数据库。
