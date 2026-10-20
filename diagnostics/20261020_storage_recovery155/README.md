# 写盘与恢复155证据

详见[修复、部署及覆盖限制](../../docs/STORAGE_RECOVERY_REPAIR155_20261020.md)。

`receipts/repair153_replay.json` 保留长录像续建覆盖不足的原始 `validation_failed`；`receipts/repair155_acceptance.json` 说明互补覆盖，不能把长录像改称完整移动恢复通过。其他失败回执一并保留，未采用重复运行取最好结果。

`display/rviz_saved_rgb.png` 是Jetson实际打开历史数据的截图；无实时相机/运动验收。`deployment/` 是8号新建入口的文件更新与回退证据；模式2/3、5E和导航保护哈希未变。复现脚本含原机绝对路径与一次性输出目录，不是通用部署命令。没有录像或数据库。

`manifest.json` 核对远端筛选导出的原始证据；新增说明、复现脚本和源码绑定另由仓库根清单覆盖。
