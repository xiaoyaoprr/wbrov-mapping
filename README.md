# WBROV 水下建图算法

2026-10-15 从 Jetson 当前建图入口及对应源码整理的源代码归档，包含 ZED 双流前端、定制 RTAB-Map Core/ROS、AprilTag 观测和多视角锚点、重定位、重力约束、地图可视化与设备标定配置。

当前入口指向的运行包标识为 `168660dbaf7e786a4d0682455b19e8672bbd5930ac43eeb0d47b7b10660c8f0b`。归档保留原文件和来源记录，适合代码备份、审查和后续开发。本次没有重新编译或启动设备，不能据此认定已完成现场验证。

## 代码目录

| 目录 | 内容 |
| --- | --- |
| `src/zed_rgbd_publisher/` | ZED 双流图像、深度、IMU 发布器及独立算法测试源码 |
| `src/rtabmap_core/` | RTAB-Map 0.23.7 源码基线和建图、重力约束、优化器等定制改动 |
| `src/rtabmap_ros/` | ROS 2 消息、里程计、SLAM 包与时间戳转换保护 |
| `src/apriltag/` | Tag 检测、位姿几何保护、关键帧绑定、多视角锚点生成 |
| `runtime/libexec/` | 当前运行包中的安全重定位和转角恢复 Python 模块 |
| `runtime/tools/` | 原运行包检查工具 |
| `src/native_helpers/` | 图像健康观测、发布者身份校验和消息来源适配器源码 |
| `src/visualization/` | 图优化结果显示和 RViz 配置 |
| `config/` | DDS 配置及现机相机标定文件 |
| `launch/original/` | 现机建图启动脚本的原样副本 |
| `provenance/` | 运行清单、逐文件来源与 SHA-256、构建配置摘录 |

## 核验归档

需要 Python 3.9 或更新版本。在仓库根目录执行：

```bash
python tools/verify_package.py
```

该命令只读取文件并校验 `MANIFEST.sha256`，不会导入建图模块或打开硬件。`provenance/validation.json` 记录本次检查结果。

已核对当前运行清单指定的 20 个 Core/ROS 源码文件；发布器源码与原构建清单逐项匹配；运行辅助脚本与现机运行清单匹配。其余 Core/ROS 文件从对应源码基线补齐，部分 ROS 资源由本地完整源码备份补充，每个文件的来源单独记录。因此本仓库不宣称可以逐字节复现当前二进制。

## 构建与运行

设备环境为 Ubuntu 22.04.5 / AArch64，建图入口使用 ROS 2 Humble、ZED SDK 与 CUDA。Core 源码版本为 RTAB-Map 0.23.7；发布器要求 ZED SDK 4 系列或满足其 CMake 约束的兼容安装，CUDA 版本由 ZED 配置指定。系统 SDK 和编译好的动态库需由对应环境提供。

构建顺序、依赖和限制见 [docs/BUILDING.md](docs/BUILDING.md)。各模块保留自己的 CMake/package.xml，不提供自动部署或自动启动脚本。

`launch/original/start_rtabmap_calib_ab.sh` 保留设备绝对路径、二进制哈希和进程身份检查，用于还原当前启动配置。它不是更换机器后可直接执行的通用启动器。重新构建后应单独形成运行包、更新清单和入口哈希，再按项目既有流程验证。

本归档不包含地图数据库、录像、运行日志、密码、SDK 二进制和车辆控制程序。标定文件对应现机相机及水下标定条件，使用其他相机或安装配置时应重新标定。

## 来源与许可

第三方代码保留原版权与许可文件。RTAB-Map 顶层许可不能代表其全部内嵌组件；详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。本次归档没有替用户为自有代码新增开源许可证；仓库按私有仓库准备。
