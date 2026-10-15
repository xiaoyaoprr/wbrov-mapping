# WBROV 水下建图算法

本仓库基于 2026-10-15 的 Jetson 源码归档，包含 ZED 双流前端、定制 RTAB-Map Core/ROS、AprilTag、重定位、重力约束和地图可视化。本次已更新为主图隔离、临时子图保留和 AprilTag 位置显示的候选源码。

**候选状态：原生编译及部分录制回放检查通过，非 Tag 自动接回和严格回放源时序验收仍未通过，尚未进行水下现场验证。** 已观察到主图/临时子图隔离、局部健康时继续记录及主图/局部 AprilTag 位置消息；完整结果和限制见 [候选更新说明](docs/SUBMAP_TAG_CANDIDATE.md)。

桌面8号已完成文件部署，两个启动脚本现在绑定运行包 `e14fc23acd43991952aa0c9e7de0716f9eadf027959cedca07a1ddc16ea193a1`。已修复首次启动时显示运行副本漏拷依赖导致的提前退出；安装后依赖检查和隔离 ROS 显示节点测试通过，完整8号相机链路仍需现场复测。部署与回退信息见 [8号部署记录](docs/DEPLOYMENT_20261015.md)。原始归档可通过 `234b053` 或 `source-2026-10-15` 查阅。

## 代码目录

| 目录 | 内容 |
| --- | --- |
| `src/zed_rgbd_publisher/` | ZED 双流图像、深度、IMU 发布器及独立算法测试源码 |
| `src/rtabmap_core/` | RTAB-Map 0.23.7 源码基线和建图、重力约束、优化器等定制改动 |
| `src/rtabmap_ros/` | ROS 2 消息、里程计、SLAM 包与时间戳转换保护 |
| `src/apriltag/` | Tag 检测、位姿几何保护、关键帧绑定、多视角锚点生成 |
| `runtime/libexec/` | 候选安全重定位、Tag 观测与显示模块，依赖按原生运行回执补齐 |
| `runtime/tools/` | 原运行包检查工具 |
| `src/native_helpers/` | 图像健康观测、发布者身份校验和消息来源适配器源码 |
| `src/visualization/` | 图优化结果显示和 RViz 配置 |
| `config/` | DDS 配置及现机相机标定文件 |
| `launch/original/` | 更新前建图启动脚本的原样归档 |
| `launch/deployed_20261015/` | 已部署的建图脚本和8号外层脚本，绑定同一新运行包 |
| `provenance/` | 初始设备归档依据，以及 `submap-tag-20261015/` 中的候选来源和验证摘要 |
| `tests/` | 原生几何/门控夹具、录制回放及消息审计测试 |

## 核验归档

需要 Python 3.9 或更新版本。在仓库根目录执行：

```bash
python tools/verify_package.py
python tools/test_mapping_candidate.py
```

第一条命令只读取文件并校验 `MANIFEST.sha256`。第二条运行不启动 ROS 节点的 Python 行为测试；未安装 NumPy/OpenCV 时会明确跳过三个数值投影测试。加 `--require-opencv` 可要求这些测试必须执行。候选验证摘要在 `provenance/submap-tag-20261015/validation.json`；原 `provenance/validation.json` 保留初始归档结果。

初始归档核对了设备清单指定的 20 个 Core/ROS 文件及发布器、运行脚本，其余基线和补充来源单独记录。本次修改的源码以候选 `source-map.json` 为准，不再宣称它们与正式设备运行包逐字节一致，也不宣称可以逐字节复现二进制。

## 构建与运行

设备环境为 Ubuntu 22.04.5 / AArch64，建图入口使用 ROS 2 Humble、ZED SDK 与 CUDA。Core 源码版本为 RTAB-Map 0.23.7；发布器要求 ZED SDK 4 系列或满足其 CMake 约束的兼容安装，CUDA 版本由 ZED 配置指定。系统 SDK 和编译好的动态库需由对应环境提供。

构建顺序、依赖和限制见 [docs/BUILDING.md](docs/BUILDING.md)。各模块保留自己的 CMake/package.xml，不提供自动部署或自动启动脚本。

`launch/deployed_20261015/` 保留当前设备的绝对路径、二进制哈希和进程身份检查，`launch/original/` 保留更新前配置。它们不是更换机器后可直接执行的通用启动器。重新构建后应单独形成运行包、更新清单和入口哈希，再按项目既有流程验证。

本归档不包含地图数据库、录像、运行日志、密码、SDK 二进制和车辆控制程序。标定文件对应现机相机及水下标定条件，使用其他相机或安装配置时应重新标定。

## 来源与许可

第三方代码保留原版权与许可文件，来源说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。本次更新没有替用户为自有代码新增开源许可证，也未修改仓库可见性。
