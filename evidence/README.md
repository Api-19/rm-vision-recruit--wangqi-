# 考核成果与证据索引（2026-10-19）

本目录汇总真实运行成果与文件位置。所有成果仅在本地；用户尚未重新授权上传。AI实现、资料作答、本人理解分别记录。`manifest.json`为本次结果文件SHA256与大小，可核对文件完整性。

| 考核内容 | 保存的成果 | 边界 |
|---|---|---|
| 题一环境 | [运行输出](2026-10-19/environment.txt)、[展示图](2026-10-19/environment.png)、[版本快照](../q1_env/results/requirements-observed.txt) | 实际macOS环境；展示图是日志排版，不冒充桌面截图 |
| 必要检查 | [18项测试原始日志](2026-10-19/tests.txt)、[展示图](2026-10-19/tests.png) | 包含合成几何测试，不是本人实拍测距结果 |
| 题二标定 | [K/D与逐图结果](../q2_calib/results/display_27mm_19views_2026-10-18/calibration.json)、[报告](../q2_calib/results/display_27mm_19views_2026-10-18/REPORT.md)、[误差图](../q2_calib/results/display_27mm_19views_2026-10-18/calibration_quality.png) | 19张本人照片，27mm为本人报告值；尚缺A4尺测验证 |
| 标定发挥对比 | [报告](../q2_calib/results/display_27mm_19views_2026-10-18/variants/REPORT.md)、[表](../q2_calib/results/display_27mm_19views_2026-10-18/variants/summary.csv)、[图](../q2_calib/results/display_27mm_19views_2026-10-18/variants/comparison.png) | 前5张、8张估计近正对子集、错误20mm；不是独立采集的严格正对实验 |
| B点云处理 | [原始点云投影](../qB_sentry/results/cloud_map/01_raw.png)、[调平](../qB_sentry/results/cloud_map/02_levelled.png)、[候选地面](../qB_sentry/results/cloud_map/03_ground_candidate.png)、[地形图](../qB_sentry/results/cloud_map/04_terrain_map.png)、[记录](../qB_sentry/results/cloud_map/processing.json) | 实际输入PCD的程序可视化，不伪称点云软件操作截图；语义仍需核查 |
| B栅格 | [图像](../qB_sentry/results/cloud_map/map.pgm)、[配置](../qB_sentry/results/cloud_map/map.yaml) | 0.05m；2.5D及局部插值假设 |
| B查询与检查器 | [汇总](../qB_sentry/results/given_map/summary.csv)、[检查器日志展示](../qB_sentry/results/checker_output.png)；原始日志在given_map各算法子目录 | 1/2/4/5通过，3/6/7端点不合法；不移动查询端点 |
| B搜索发挥 | [对比图](../qB_sentry/results/given_map/comparison.png)、[查询1动画](2026-10-19/search_query1.gif)、[本次统计](2026-10-19/search_query1.json) | 动画按各自扩展比例回放，不能以动画速度比较耗时；图中扩展计数可包含重复扩展 |
| B自建图比较 | [配准叠图](../qB_sentry/results/auto_registration/registration.png)、[候选得分](../qB_sentry/results/auto_registration/candidates.json)、[条件性结果](../qB_sentry/results/auto_registration/path_comparison.json) | 朝向有歧义，未获独立地标确认 |
| B决策发挥 | [状态逻辑](../qB_sentry/results/decision/state_machine.md)、[优先级图](2026-10-19/decision_priority.png)、[仿真图](../qB_sentry/results/decision/simulation.png)、[轨迹数据](../qB_sentry/results/decision/simulation.csv) | 离线编排场景，不是实战验证 |
| 附加题 | [二至四作答](../extra/README.md) | 资料来源已核验；附加一训练/ONNX/测速未完成 |

## 私人照片与去畸变成果（保存在本地，不入Git）

仓库内 `q2_calib/input/2026-10-18-display-27mm-19/` 保留：

- `png/`：19张无缩放转换图。
- `fit/corners_*.png`：角点检测叠图。
- `fit/undistort_comparison.png`：原图和去畸变完整对照。
- `fit/undistort_preview.jpg`：便于打开的缩小预览，不能拿预览像素配原K测距。
- `fit/observations.npz`：实际检测点缓存。
- `LOCAL_FILES.json`：本地保存清单（相对路径与大小）。

原HEIC仍在用户Downloads，未改写。含照片图像不提交Git；以后是否提交及如何裁剪，仍按用户最终授权处理。此目录中的去畸变对照是真实照片处理成果，不能用测试里的合成棋盘替代。

## 尚缺的提交证据

A4至少5组真实尺测及误差曲线、同距三角度分析、Ubuntu22.04命令截图、场地异常实体/地标核查，以及本人学习总结。真实桌面截图尚未采集；现有图表与原始日志已保存，日志展示图均明确注明身份。不要将当前目录视为已满足全部提交要求。
