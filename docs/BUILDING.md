# 构建说明

仓库包含初始设备归档和后续主图/子图候选源码。候选曾在 Jetson 隔离目录完成增量原生编译与录制回放，证据见 [候选更新说明](SUBMAP_TAG_CANDIDATE.md)；整个仓库的全新环境构建尚未验证，也未进行实时相机或水下测试。

仓库提供的 `tests/native/CMakeLists.txt` 已使用提交 `601b051` 的原样源码在 Jetson AArch64 上补验：Release 配置，开启 `WBROV_TEST_DEPTH_GEOMETRY`，5 项独立 CTest 全部通过。该结果只覆盖这些几何和门控测试，不等同于完整 Core/ROS 构建或录制数据接回主图成功。

## 环境与依赖

- Ubuntu 22.04、ROS 2 Humble、CMake、C/C++ 编译器和 Python 3.10。
- RTAB-Map Core 依赖由 `src/rtabmap_core/CMakeLists.txt` 定义，包括 OpenCV、PCL、Eigen、SQLite 及所选图优化后端。现机历史构建启用了 g2o、GTSAM 等选项，具体选择见 `provenance/core-build-flags.json`。
- ROS 包依赖由各 `package.xml` 定义，使用当前归档的 `rtabmap_msgs`、`rtabmap_conversions`、`rtabmap_odom` 和 `rtabmap_slam`，避免混用系统中不同版本的 Core/ROS ABI。
- 发布器需要 ZED SDK、与 SDK 匹配的 CUDA、ROS 2 和 OpenCV。SDK 和 CUDA 安装包不在本仓库内。
- AprilTag 与恢复模块还需要对应 Python/ROS 消息依赖。按源码 import 和既有设备环境恢复，未提供经过全新环境安装验证的锁定依赖集。

## 建议构建顺序

1. 在新的构建目录中配置并编译 `src/rtabmap_core`，使用专用安装前缀，不覆盖设备现有运行包。
2. 将该前缀加入构建查找路径，使用 colcon 构建 `src/rtabmap_ros` 中需要的 ROS 2 包，传入 `--cmake-args -DCMAKE_CXX_STANDARD=17`；确保运行时使用同一套 Core 和 ROS 库。
3. 在对应 ROS 环境中构建 ZED 发布器，详见 [发布器构建说明](../src/zed_rgbd_publisher/BUILDING.md)。
4. 构建原生辅助模块，详见 [原生模块构建说明](../src/native_helpers/BUILDING.md)。
5. 单独整理 runtime 并生成新的文件清单；原始 launcher 中的哈希、权限、路径和进程契约不能直接沿用到新的二进制。

Core 的初始配置示例（仅供开发构建起步，未在本次交付执行；它不复现现机全部构建选项）：

```bash
cmake -S src/rtabmap_core -B build/rtabmap_core \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_INSTALL_PREFIX="$PWD/install/rtabmap_core" \
  -DBUILD_APP=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=ON
cmake --build build/rtabmap_core --parallel 2
cmake --install build/rtabmap_core
```

## 已知边界

初始设备归档匹配了清单中 20 个 Core/ROS 源码摘要。其他文件采用对应历史源码树补齐，149 个 ROS 资源文件来自本地完整源码备份。候选更新的文件与附加依赖在 `provenance/submap-tag-20261015/source-map.json` 中另行记录；旧设备清单是历史依据，不是当前候选源码的完整性清单。

为保留源码身份，归档未改写原始文件内的设备路径。`src/native_helpers/message_info/CMakeLists.txt` 是本次补充的可移植构建定义，已在其说明中标明未经编译验证。未包含 Android 预编译 JAR、演示数据和归档目录；本说明的构建范围是 Jetson/Linux 建图链。

`runtime/tools/check_private_runtime.py` 面向完整的设备运行包。它需要本归档刻意未携带的二进制、权限和目录布局；不要把它的结果当作源码归档的完整性检查。源码归档请使用根目录的 `tools/verify_package.py`。

本次恢复的精确 `Rtabmap.cpp` 包含 `optimizer_trace.inc` 诊断调用。该辅助文件使用 C++17 的 `std::filesystem`，要求 `WBROV_PRIVATE_OPTIMIZER_TRACE` 指向已存在的目录，否则会抛出异常。原始启动脚本包含对该变量的管理；重新构建后必须单独核查这条诊断链，不能仅依靠源码摘要推断二进制运行行为。

候选 `OdometryF2M.cpp` 需要同目录的 `DepthGeometryRecovery.h`；Core 还需要本次提交的证书和 `.inc` 文件，Wrapper 需要配套的门控、子图发布头文件。Tag 消费者需要同目录中匹配的缓存、候选重试和身份检查模块，不能只复制单个入口文件。独立测试的运行方法见候选更新说明。

保留的上游 ROS 1 工具 `src/rtabmap_ros/rtabmap_util/scripts/gazebo_ground_truth.py` 存在原始缩进错误，Python 3 语法检查不通过。它未被当前建图 launcher 引用；归档保留原字节以便追踪，没有为打包顺便改写该旧工具。其他纳入本次检查的 Python 文件通过语法解析；这不代表外部依赖已安装或程序已经运行验证。
