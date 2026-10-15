# 主图隔离、临时子图与 AprilTag 候选更新

本次更新将 2026-10-15 候选包的源码合入对应目录，并补齐其原生运行回执要求的 Tag 依赖。它尚未替换机器人上的正式运行包。

**非 Tag 自动接回仍未通过录制回放验收。** 原生合成几何测试通过，录制回放也验证了主图隔离和 Tag 位置消息，但不能据此认为断图和重叠问题已全部解决。

## 建图行为

1. 位姿质量合格且已注册到主图时，继续向主图写入。
2. 视觉退化时尝试有时间、运动和方向约束的短时深度恢复。ZED 图像与立体深度相关；IMU 不承担无限期位置积分。
3. 局部估计也不可信时，暂停所有地图写入，保留已有地图并继续计算定位。
4. 局部恢复但尚未锚定主图时，在独立坐标下记录临时子图；再次真实失锁后分开，不建立跨空档的假邻接边。
5. 主图恢复须通过原始 RGB-D、身份凭据、多组对应、轨迹连续性、固定旧图和观察期检查。未通过时临时图保持独立，不进入主图点云。

连续 8 个 lost 样本仍是前端重建条件，不是固定时长或恢复主图写入的条件。`Rtabmap/StartNewMapOnLoopClosure` 保持 `false`，允许健康的未对齐子图保留数据。深度恢复仍须显式设置 `WBROV_DEPTH_RECOVERY=1`；原正式 launcher 未随源码更新而启用候选功能。

## AprilTag 和子图显示

主图中的 Tag 使用 `map` 坐标；未注册子图中的 Tag 只使用自身局部 frame。同一 ID 在两个未对齐子图中不会自动合并。位置绑定实际角点、Tag 边长、CameraInfo、相机外参、节点与会话/版本身份；图优化后重新投影。平面双解歧义或凭据不足时，只保留未定位状态。

`/wbrov/tags/main_markers` 输出主图 Tag，`/wbrov/submaps/<key>/display` 输出局部图与 Tag。显示端合并重复状态，变化最多按 5Hz 绘制，冻结快照约 1Hz 刷新；失效坐标仍立即清除。独立 RViz 配置不提供到 `map` 的假设 TF，也不自动打开窗口。

协议见 [SUBMAP_TAG_INTERFACE.txt](SUBMAP_TAG_INTERFACE.txt) 和 [TAG_VISUALIZATION.md](TAG_VISUALIZATION.md)。已验证真实 ROS 消息；本轮没有启动 RViz 渲染器。

## 验证结果及未通过项

| 项目 | 结果与边界 |
| --- | --- |
| Jetson 原生编译 | Core、F2M、Wrapper 编译和链接通过，三个 CTest 门控通过；使用隔离目录与已校验父运行环境 |
| 原生恢复事务 | 合成 76 秒连续子图在末端取得三个旧图锚点后通过准备/观察/提交检查；测试选定候选 ID，不代表录制检索成功 |
| Tag 和显示 | 13 项显示规则、10 项 ACK、25 项 Tag 测试通过；Tag 中三个真实 OpenCV 测试在 Jetson 上执行，没有跳过；真实 ROS 节点和 DDS 消息检查通过 |
| 最终 Tag 回放 | 1200 帧、60.78 秒，采纳 47 帧；主图 9 节点，四个临时子图分别 14、11、5、8 节点；主图 Tag 为 3、4，局部 Tag 为 1、2、10 |
| 暂停隔离 | 853 个有既有主图的受监测暂停区间内，未观察到主图增长或点云载荷变化；有限消息审计不证明所有异步瞬间及空间精度 |
| 非 Tag 接回 | 2000 帧重复片段找到了旧关键帧，但未形成三个合格候选，也未完成观察期/提交；重复视频不是实际回航 |
| 回放源时序 | 最终 Tag 回放的发布迟到 P95 为 33.77ms，超过既定 30ms 判据，`source_realtime_valid=false`；没有重打时间戳绕过门控 |

接回实验中，一次约 0.001998m / 0.285697° 的变换差仍被协方差归一化一致性检查拒绝；其他对应因网格覆盖或有效深度内点不足被拒绝。需要继续检查候选保留阶段与噪声模型，不能直接关闭最终几何/事务检查以获得接回。

完整验证摘要与原始证据的哈希位于 [provenance/submap-tag-20261015](../provenance/submap-tag-20261015)。失败后修复的显示节点属性冲突、ROS 浮点字段类型及重复刷新问题均计入说明。歧义 Tag 正例改为负例，没有放宽运行阈值。

没有实现不确定空档回填、压力计融合、IMU 平移不确定度搜索、测量好的 Tag 全局先验或 TSDF；没有现场水下精度、实时 ZED/GPU 或控制验证。

## 在仓库中运行检查

只读完整性检查和不启动 ROS 的 Python 行为测试：

```bash
python tools/verify_package.py
python tools/test_mapping_candidate.py
```

无 NumPy/OpenCV 时三个投影测试会明确跳过；需要全部执行时使用 `python tools/test_mapping_candidate.py --require-opencv`。本轮 Windows 仓库检查与之前的 Jetson 原生结果分别记录。

独立 C++ 门控测试（需要 CMake 和 C++17 编译器）：

```bash
cmake -S tests/native -B build/mapping-tests
cmake --build build/mapping-tests --config Release
ctest --test-dir build/mapping-tests -C Release --output-on-failure
```

有 Eigen/OpenCV 时可加 `-DWBROV_TEST_DEPTH_GEOMETRY=ON`。Core 和 Wrapper 的集成夹具也在 `tests/native`；它们需要整套匹配的 Core/ROS 编译环境和独立输出目录，未加入默认无硬件测试。此处新增 CMake 入口尚未在 Windows 编译，已有 Jetson 证据针对相同测试源码的隔离构建。

`tests/recorded_replay/replay_single_map.py` 保留了实际使用的回放驱动，绑定已有 Jetson 运行回执及录制数据路径。仓库不包含录制数据或动态库；它不是任意机器上可直接运行的演示脚本。

## 来源与评审核对

`source-map.json` 逐文件记录候选包路径、哈希及更新前哈希；额外 Tag 依赖按相同原生运行回执校验。原有设备清单和导出记录保留为初始归档 `234b053` 的历史依据，不再代表更新后的源码与正式设备逐字节相同。

附件评审的三点需要限定：`Mem/UseOdomGravity=false` 不用里程计替代 IMU，存在有效 IMU 时仍可建立重力约束；`LandmarksIgnored=true` 不会使受控 Tag 恢复事务失效；主图应由可信注册关系认定，不能直接选择当前或最大的连通分量。`RGBD/AngularUpdate=0.05` 约为 2.86°。

本次未更改仓库可见性、许可、正式启动入口或设备配置。
