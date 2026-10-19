# 第三方来源与许可说明

本仓库保留源码中的版权头、LICENSE、COPYING 和其他许可声明，不为全部代码统一声明一种新许可证。

- `src/rtabmap_core/LICENSE`：RTAB-Map 顶层 BSD 3-Clause 许可。
- `src/rtabmap_ros/LICENSE`：RTAB-Map ROS 顶层 BSD 3-Clause 许可。
- `src/visualization/rviz_mono/rviz2_mono.cpp`：基于 RViz 启动入口，保留文件内 BSD 3-Clause 许可及版权声明；链接系统 RViz 库，未打包这些库。
- RTAB-Map 内嵌第三方算法可能采用不同条款。例如 ORBextractor 文件包含其自身许可声明，TORO 文件含 CC BY-NC-SA 声明；具体适用范围以相应文件和组件的许可文本为准。
- ZED、CUDA、ROS 和其他外部库没有以二进制形式打包到本仓库，按各自安装来源和许可使用。
- 图像收据摘要使用系统 OpenSSL Crypto 库；本仓库未打包其二进制或改写其许可，构建需要 OpenSSL 开发文件。
- RGB-D 队列无损编码使用 Zstandard；本源码仓库不附带其源码或库文件。原生构建需要 libzstd 开发文件（含静态库）。相机程序将系统静态库私有链接，避免与 ZED SDK 导出的旧 ZSTD 函数混用；Python 解码通过 ctypes 调用系统动态库。分发相机二进制时须随包保留所用系统 Zstandard 的版权与许可证说明。
- 自有建图改动、归档说明和辅助模块未在本次整理中被新增开源授权。私有备份不会替代原有权利或许可要求。

逐文件来源及校验摘要位于 `provenance/`。重新分发或公开仓库前，应逐项保留和遵守实际包含组件的声明；不能仅根据 RTAB-Map 根 LICENSE 推断所有内嵌代码的许可。
