# WBROV 水下建图算法

**2026-10-19：8号新建地图已使用 `mapping_combo42_camera6`。** 已修复ZED与系统ZSTD符号混用导致的首次录制崩溃；真实入口启动、RGB、点云、三轴位姿显示和正常保存通过。**本次抬头后的恢复未通过，写盘反压仍可阻塞采集，不能视为完整移动建图验收成功。** 具体数据见[8号实机记录](docs/PRODUCTION8_ACCEPTANCE_20261019.md)，版本对应见[源码绑定](provenance/production8-20261019/README.md)。

当前实现包含持续录制、后台逐帧处理、定位证据与主图写入许可分离、自动积累Tag参考，以及状态/Tag/可信轨迹/三轴显示。后台历史位姿不是实时导航输入；旧图重定位、路径规划导航和水下精度不属于本次通过范围。8号模式2/3保留旧入口。

combo42固定四组离线回放共23,967帧全部处理并核对源时间戳；三组有恢复后续建，大录像末尾为`interrupted_saved`，不能称恢复成功。[性能与失败记录](diagnostics/20261019_processing_speed/README.md)保留各次结果，`dirscan43`和`f2m_inverse44`仅为未采用的独立实验。最近实机6412个已采帧全部保存，但采集中出现最长14秒空档，完整处理已采数据不代表采集连续。

本次公开源码与诊断包括第五至第七轮累积修改、上述相机修复及成功/失败证据。历次说明按日期阅读：[第五轮](docs/ROUND5_20261017.md)、[第六轮](docs/ROUND6_LOCALIZATION_AND_WRITES_20261017.md)、[第七轮](docs/ROUND7_CONTINUOUS_MAPPING_20261018.md)。原有Claude复审材料、第四轮候选和所有失败记录保留。

历史 `launch/deployed_20261015/` 的目录名沿用早期布局，部分文件已含候选修改，不是当前8号的原样副本。当前已部署相机的精确编译源码另存于`provenance/production8-20261019/camera6-build-source.zip`；后台worker对应当前源码。根目录`MANIFEST.sha256`核验本次全部公开文件，早期运行清单不能作为当前部署依据。

本仓库基于 2026-10-15 的 Jetson 源码归档，包含 ZED 双流前端、定制 RTAB-Map Core/ROS、AprilTag、重定位、重力约束和地图可视化。后续更新包含主图隔离、临时子图保留、AprilTag位置显示和第二轮候选一致性修正。

**早期子图候选记录：** 原生编译及部分录制回放检查通过，当时非 Tag 自动接回和严格回放源时序验收未通过，未进行水下现场验证。主图/临时子图隔离、局部健康时继续记录及 AprilTag 位置消息的早期结果见 [候选更新说明](docs/SUBMAP_TAG_CANDIDATE.md)。

**历史第三轮记录（已被上述版本替代）：** 当时桌面8号绑定运行包 `74d38aba069fcfbb0872b705ca44b6ea5b95e96c5362a4f1aa214e44bb04a77f`。第三轮增加短暂几何退化时的限时停写与旧图验证、具体触发原因、未对齐子图计数和固定状态面板。首次实机失败记录保留；调整静止场景后，同一候选及安装后的真实相机→建图→RViz→正常保存均通过，窗口分别在36.1/36.6秒可见。仅开启30%灯光，未发送运动或解锁，结束已关灯。偶发GLX黑屏未证实修复，完整无Tag接回和水下连续建图仍待验收，见[第三轮记录](docs/ROUND3_20261016.md)。前次结果见[第二轮记录](docs/ROUND2_20261016.md)，原始归档可通过`234b053`或`source-2026-10-15`查阅。

## 代码目录

2026-10-16 移动使用后出现主图暂停与 Tag 无法恢复的 [问题清单](docs/PAUSE_TAG_REVIEW_20261016.md) 和 [诊断包](diagnostics/20261016_pause_and_tags/README.md) 保留用于交叉复审，包含明确标注为不完整的主图点云、分量位姿及日志统计。后续修正按各轮文档和实测记录核对。

早期诊断和候选失败记录保留；最新安装版本的启动、主图显示与保存实测见 [8号验收记录](docs/PRODUCTION8_ACCEPTANCE_20261019.md)。

| 目录 | 内容 |
| --- | --- |
| `src/zed_rgbd_publisher/` | ZED 双流图像、深度、IMU 发布器及独立算法测试源码 |
| `src/rtabmap_core/` | RTAB-Map 0.23.7 源码基线和建图、重力约束、优化器等定制改动 |
| `src/rtabmap_ros/` | ROS 2 消息、里程计、SLAM 包与时间戳转换保护 |
| `src/apriltag/` | Tag 检测、位姿几何保护、关键帧绑定、多视角锚点生成 |
| `runtime/libexec/` | 候选安全重定位、Tag 观测与显示模块，依赖按原生运行回执补齐 |
| `runtime/tools/` | 历史运行包检查工具及对应时期的源码哈希 |
| `src/native_helpers/` | 图像健康观测、发布者身份校验和消息来源适配器源码 |
| `src/deferred_mapping/` | 后台逐帧建图、已封存输入处理及点云组装 |
| `src/visualization/` | 图优化结果显示和 RViz 配置 |
| `config/` | DDS 配置及现机相机标定文件 |
| `launch/original/` | 更新前建图启动脚本的原样归档 |
| `launch/deployed_20261015/` | 历史入口与后续候选修改，不代表当前8号的原样副本 |
| `launch/candidate_quality_pause/` | 第三轮候选入口；对应当时的静止复测 |
| `launch/candidate_receipt_resume/` | 接收恢复与有界队列修复的候选来源，已通过静止实测并部署 |
| `provenance/` | 初始设备归档依据，以及 `submap-tag-20261015/` 中的候选来源和验证摘要 |
| `tests/` | 原生几何/门控夹具、录制回放及消息审计测试 |

## 核验归档

需要 Python 3.9 或更新版本。在仓库根目录执行：

```bash
python tools/verify_package.py
python tools/test_mapping_candidate.py
```

第一条命令只读取文件并校验 `MANIFEST.sha256`。第二条运行不启动 ROS 节点的 Python 行为测试；未安装 NumPy/OpenCV 时会明确跳过三个数值投影测试。加 `--require-opencv` 可要求这些测试必须执行。候选验证摘要在 `provenance/submap-tag-20261015/validation.json`；原 `provenance/validation.json` 保留初始归档结果。

初始归档核对了设备清单指定的 20 个 Core/ROS 文件及发布器、运行脚本，其余基线和补充来源单独记录。`provenance/submap-tag-20261015/` 内的 `source-map.json` 和 `receipt-resume-candidate.json` 保留历史来源；第三轮编译输入与实测结果在 `provenance/round3-20261016/`，当时设备绑定在 `provenance/submap-tag-20261015/deployment.json`；前次 `receipt-resume-*.json` 保留历史证据。入口与已编译源码按哈希核对，不宣称本归档能逐字节复现二进制。

## 构建与运行

设备环境为 Ubuntu 22.04.5 / AArch64，建图入口使用 ROS 2 Humble、ZED SDK 与 CUDA。Core 源码版本为 RTAB-Map 0.23.7；发布器要求 ZED SDK 4 系列或满足其 CMake 约束的兼容安装，CUDA 版本由 ZED 配置指定。系统 SDK 和编译好的动态库需由对应环境提供。

构建顺序、依赖和限制见 [docs/BUILDING.md](docs/BUILDING.md)。各模块保留自己的 CMake/package.xml，不提供自动部署或自动启动脚本。

`launch/deployed_20261015/` 保留历史设备的绝对路径、二进制哈希和进程身份检查，`launch/original/` 保留更新前配置。它们不是更换机器后可直接执行的通用启动器。重新构建后应单独形成运行包、更新清单和入口哈希，再按项目既有流程验证。

源码归档不包含地图数据库、录像、密码、SDK 二进制和车辆控制程序。`diagnostics/` 按用户授权附有筛选后的运行日志、屏幕截图和部分点云；各目录说明其成功与失败范围，不包含完整录像或地图数据库。标定文件对应现机相机及水下标定条件，使用其他相机或安装配置时应重新标定。

## 来源与许可

第三方代码保留原版权与许可文件，来源说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。本次更新没有替用户为自有代码新增开源许可证，也未修改仓库可见性。
