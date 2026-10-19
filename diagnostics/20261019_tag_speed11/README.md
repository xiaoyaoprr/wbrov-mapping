# Tag 恢复、F2M 帧内求逆缓存证据

结论与界限见[验证说明](../../docs/TAG_RESTORE_SPEED11_20261019.md)。候选已恢复Tag，保存录像中检测、恢复后续建、保存及实际RViz地图显示通过；8号未切换，未作现场移动验收。

- `replay/result.json`：四组固定串行全帧对照，共18,142帧。四组的源时间、保存哈希与关闭数据库核对通过；短片均恢复1次。长片无新切链，不能充当恢复验收。
- `replay/*/mapping/frames.csv`：原始逐帧结果；新旧图不相同，差异没有隐藏。原始视频与数据库未加入仓库。
- `tag_speed11_fixed_random2/result.json`、`fixed/`：诊断用固定随机过程下的1,581帧对照；逐帧结果、点云与Tag导出相同。85.442对87.190秒为核心处理的单次耗时，不是实际全链路稳定收益。
- `tag_speed11_fixed_random/`：首次诊断缺少trace目录的原始失败。
- `tag_speed11_build/result.json`：首次ASAN失败；`resume_result.json`：空白对照复现NVIDIA析构错误，UBSAN及原生构建/回归通过。ASAN仍未通过。
- `proposal/manifest.json` 是优化提出时的原始未编译记录，保持不改；最新采用范围见 `source_binding.json` 与原生构建回执。该目录保留用于核对的旧、新F2M源码和单项探针。
- `package.json`、`entry.json`：实际候选哈希与38项Linux检查；Tag模块重新启用。
- `audit/`：构建/回放脚本、差异定位、实际RViz截图与最终空闲检查。固定随机插件仅在离线诊断使用，不部署。
- `remote_evidence_sha256.json`：77份原始小证据的摘要。仓库顶层MANIFEST同时覆盖后来增加的审计、截图与说明。

普通完整回放短片快4.15%、长片慢0.69%，因此整体提速没有证实。本轮采用的仅是通过逐位核验的局部冗余计算消除，不以改变几何、减少帧数或降低点云密度换速度。
