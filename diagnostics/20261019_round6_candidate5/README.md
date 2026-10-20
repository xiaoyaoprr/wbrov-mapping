# 候选5：有界压缩内存队列

本目录没有整段录像；仅保存源码绑定、测试及失败证据。

- `round6_candidate5/result.json`：Jetson构建与SHA；相机16项CTest、SDK/门控/质量验证通过。
- `round6_candidate5/camera_tests.log`：分别暂停write和fsync，3000个微型帧总量完整、字节一致；两种故障注入均不提前发布失败段。
- `package_round6_candidate5_result.json`：33项Python测试及入口check-only通过；包含退出cmdline暂失的有限等待回归。
- `sha256.json`：从设备取回的16个原始构建文件摘要。

实际field v4在遥控灯光就绪检查超时退出，没有启动相机、RGB-D采集或建图，因此没有新版实机显示/连续采集/保存通过证据。没有重复启动直到通过，也没有切换8号。具体失败入口记录见`field_v4_startup/`。

此前候选4的7533帧完整保存与5次采集空档属于另一个二进制，保存在相邻candidate4目录，不能用其保存结果替代本候选验收。
