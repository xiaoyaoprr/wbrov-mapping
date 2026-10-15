# 第三方来源与许可说明

本仓库保留源码中的版权头、LICENSE、COPYING 和其他许可声明，不为全部代码统一声明一种新许可证。

- `src/rtabmap_core/LICENSE`：RTAB-Map 顶层 BSD 3-Clause 许可。
- `src/rtabmap_ros/LICENSE`：RTAB-Map ROS 顶层 BSD 3-Clause 许可。
- RTAB-Map 内嵌第三方算法可能采用不同条款。例如 ORBextractor 文件包含其自身许可声明，TORO 文件含 CC BY-NC-SA 声明；具体适用范围以相应文件和组件的许可文本为准。
- ZED、CUDA、ROS 和其他外部库没有以二进制形式打包到本仓库，按各自安装来源和许可使用。
- 自有建图改动、归档说明和辅助模块未在本次整理中被新增开源授权。私有备份不会替代原有权利或许可要求。

逐文件来源及校验摘要位于 `provenance/`。重新分发或公开仓库前，应逐项保留和遵守实际包含组件的声明；不能仅根据 RTAB-Map 根 LICENSE 推断所有内嵌代码的许可。
