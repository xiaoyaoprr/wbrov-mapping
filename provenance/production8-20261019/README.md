# 2026-10-19 设备版本与源码对应

8号新建模式已绑定 `mapping_combo42_camera6`，模式2/3仍为原入口。本目录记录已部署相机、已测试后台处理器的来源；不是新的安装程序。

- `source-binding.json`：相机与worker编译输入、二进制哈希、实机验收边界。
- `camera6-build-source.zip`：相机6实际编译的31个输入文件，逐个与原生构建回执SHA匹配；不含SDK、动态库或录像。
- `src/zed_rgbd_publisher/` 的三个共享文件保留worker39读取优化：`rgbd_frame_codec.hpp`、`rgbd_spool.hpp`、`rgbd_spool_test.cpp`。设备上的相机沿用相机5的读入实现，只更新链接隔离与SDK回归测试。因此，要复查相机6的14项CTest，须解包本目录源码到独立目录再构建，不能把共享头文件的最新组合称为已部署二进制的原样输入。
- worker39的40个构建输入均与仓库文件逐字节匹配。Python队列/解码脚本的换行已对齐冻结源码，未改变行为。

构建仍需原设备的ROS 2、CUDA、ZED SDK、OpenCV和Zstandard开发库。相机显式链接私有静态`libzstd.a`，避免SDK内置旧版函数与系统新版函数混用。没有声明跨环境逐字节复现二进制。

实机RGB、点云、保存通过；抬头后的恢复及采集不中断未通过。详情见[实机记录](../../docs/PRODUCTION8_ACCEPTANCE_20261019.md)。旧部署回执保留各自日期，不能据其推断当前状态。
