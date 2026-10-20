# B题六项发挥：实现、运行证据与剩余不确定性

2026-10-20，Agent实现并运行，用户学习状态不随工程完成升级。原始点云/题目地图只读，全部7组原始端点不移动。已完成六项的程序与实验材料；**异常实体身份与绝对朝向缺少独立真值，不能把报告齐全等同所有语义已认证正确。**

| 发挥项 | 本次成果 | 证据边界 |
|---|---|---|
| 1 坡道/台阶 | 两段实际缓坡剖面、一处20cm跳变；高度/坡度/台阶分类图与合成几何回归 | 实际局部验证，非全地图逐格真值；坡面几何可走仍需底盘余量 |
| 2 异常物体 | 几何异常分类、统计剔除与地下点剔除、区域点簇审查和前后图 | 已处理异常点；未取得“不应存在的具体实体”身份或原始场景说明，不能认证该语义要求全部完成 |
| 3 地图对正 | 4面墙法线的轴向残差、俯视前后图、米制地图 | 轴对正已验证；0°/180°正反仍有歧义，二者不混淆 |
| 4 搜索可视化/比较 | 既有搜索扩展图/动画，新增3次重复耗时统计及路径代价核对 | 同图同起终点比较；八邻居网格代价与包含原始端点连接的CSV长度区分 |
| 5 自建图规划比较 | 0°/180°两种假设全部7组×两种算法；额外18.045m本地图内路径 | 题目7组均无合格自建图路径，如实报告；不要求为了“成功”删障碍、移动端点 |
| 6 决策/仿真 | 7个状态/优先级情境验证、5状态覆盖、实际转移记录、仿真GIF | 编排的离线场景，阈值是示例，无真实硬件或比赛规则认证 |

## 1. 坡道与台阶

[地形分类](terrain_classes.png)、[实际高度剖面](terrain_profiles.png)、[区域统计](ramp_candidates.json)、[样本记录](terrain_evidence.json)。

- ramp_A：固定x=21.765m，y=2.3→3.5m，高度约0.20→0m；连续采样最大相邻跳变约1.57cm。所在较大连通坡面平面拟合坡度约9.09°，RMS约1.46cm。
- ramp_B：固定x=11.625m，y=0.35→0.85m，高度约0.21→0.30m，最大相邻跳变约1.88cm。所在坡面拟合坡度约9.57°，RMS约0.78cm。
- 台阶剖面：y=2.675m，x=7.925→8.575m，出现20.06cm跳变，超过题目12cm上限；边缘和邻域保守阻挡。
- 样本缓坡中心格在生成图中均为free，但半径0.30m圆底盘沿整段可放比例仅17%/0%，受边缘/邻域未知或障碍约束；不能把一条高度平滑的中心线当作整个机器人一定可走。坡道分类与最终膨胀后的可行性是不同层次。
- 分类器不使用全局“z高于某值就是障碍”的规则；升高的平地仍可通过几何分类。合成15°坡面、20cm台阶、0.5m高平地的回归检查通过。30°坡度上限为演示参数，题目未给最大爬坡能力。

初次选择的两个长剖面跨越了多种结构，不能当作单个坡道证据；检查图后改用上述局部剖面，未修改高度数据以制造结论。

## 2. 异常点、可疑结构与成因

[处理前后](anomalies_before_after.png)、[数量](anomaly_counts.json)、[分层观察](height_slices.png)、[局部三维结构](object_roi_views.png)、[点簇候选](cluster_candidates.json)。

- 0.05m体素后1009274点。统计离群判据（20近邻、std_ratio=2.5）排除10374点；低于已选最低场地平面15cm的点1768个；两类并集10496点，保留998778点。
- 被剔除点在XY和高度上散布、许多低于最低支撑平面，缺少稳定表面支持。与零散采样噪声/异常回波或仿真噪声注入相符；**成因是基于几何的假设，PCD无时序、对象标签、仿真模型，不能确证由哪种机制造成。**
- 对较密集的柱体、中央条形结构、边界设施只保存ROI和聚类候选，未仅凭“形状奇怪”删除。外围高墙在导航高度裁剪中不参与地面，但不将其冒充已确认的异常实体。
- 地下15cm阈值依赖所选平面为最低场地地面的假设；若存在更低合法楼层，必须撤销该裁剪。原文件保留，处理完全可重跑。

因此第2项完成了异常几何识别/处理实验与合理成因分析，**具体异常实体语义仍需原仿真场景/CAD/对象说明确认**。没有把点云稀疏化当成物体识别真值。

## 3. 轴对正与栅格语义修复

[对正图](axis_alignment.png)、[4墙残差](axis_alignment.json)、[地图](cloud_map/map.pgm)、[配置](cloud_map/map.yaml)。四墙对正后的XY法线距离最近图像轴最大偏差0.01523°。采用同一云内拟合的墙作内部一致性检查，未声称独立测量精度。

本次发现原生成器将未知写为205。negate=0时其占据概率1-205/255≈0.196，低于生成图free_thresh=0.25，会被误解为空闲。现改128（概率≈0.498，在free/occupied阈值之间），并验证加载后的free严格等于254像素；回归使用中央缺测孔洞保证覆盖该缺陷。初版测试使用全覆盖平面没有未知像素，失败在测试前提；改为确有缺测孔洞后再验证，不放宽安全断言。

原cloud_map的几何未改变，只修复未知灰度；旧auto_registration目录明确标记HISTORICAL。当前结果以本目录为准。

## 4. 搜索过程与性能

原有逐查询扩展图和[查询1回放](../../../evidence/2026-10-19/search_query1.gif)仍有效（基于给定图，不受自建图灰度修复影响）。[重复实验](search_repeated.csv)、[对比图](search_repeated.png)在同一给定地图、半径0.3m、查询1/2/4/5上各执行3次。两算法的网格最短代价一致，报告中位耗时而非以一次快慢断言普遍优劣。动画按各自扩展百分比推进，非相同实际时间。

原root path1/2/4/5.csv与检查器输出保留；3/6/7非法端点不生成路径。曼哈顿与权重2实验是额外比较，在八邻居下不保证最优，也不保证一定减少扩展。

## 5. 两种配准假设与自建图比较

[候选](registration_candidates.json)、[0°叠图](orientation_0/registration.png)、[180°叠图](orientation_180/registration.png)、[汇总CSV](comparison_summary.csv)。两候选平均截断距离分别0.215674m与0.216487m，差约0.000812m，保留歧义，未伪造人工地标。

| 朝向假设 | 查询 | 自建图A*结论 | 参考图长度m或结论 |
|---|---|---|---|
| 0° | 1 | goal_disk_collides_or_crosses_boundary | 21.22368757857423 |
| 0° | 2 | start_disk_collides_or_crosses_boundary | 27.6391204718293 |
| 0° | 3 | start_disk_collides_or_crosses_boundary;goal_disk_collides_or_crosses_boundary | start_disk_collides_or_crosses_boundary |
| 0° | 4 | start_disk_collides_or_crosses_boundary | 26.029322497184637 |
| 0° | 5 | goal_disk_collides_or_crosses_boundary | 57.22292998349174 |
| 0° | 6 | goal_disk_collides_or_crosses_boundary | goal_disk_collides_or_crosses_boundary |
| 0° | 7 | start_disk_collides_or_crosses_boundary;goal_outside_map | goal_outside_map |
| 180° | 1 | start_disk_collides_or_crosses_boundary;goal_disk_collides_or_crosses_boundary | 21.22368757857423 |
| 180° | 2 | disconnected_on_conservative_grid | 27.6391204718293 |
| 180° | 3 | start_disk_collides_or_crosses_boundary;goal_disk_collides_or_crosses_boundary | start_disk_collides_or_crosses_boundary |
| 180° | 4 | start_disk_collides_or_crosses_boundary;goal_disk_collides_or_crosses_boundary | 26.029322497184637 |
| 180° | 5 | start_disk_collides_or_crosses_boundary;goal_disk_collides_or_crosses_boundary | 57.22292998349174 |
| 180° | 6 | start_disk_collides_or_crosses_boundary;goal_disk_collides_or_crosses_boundary | goal_disk_collides_or_crosses_boundary |
| 180° | 7 | goal_outside_map | goal_outside_map |

同样完成Dijkstra，日志和每查询图分别在两个orientation目录下。给定图有4组有效路径，自建图两朝向均未产生合格题目路径：端点圆底盘碰撞、图外、或保守安全格不连通。对比流程已完成，但结果说明当前地形/配准及保守未知会阻碍同一查询，不能断言真实场地无路，也不能认证自建图与参考图等价。

另在自建图最大4连通安全区域内选择两端点，生成[18.045m演示路径](local_demonstration_path.png)与CSV，逐线段连续0.30m圆底盘检查通过。它仅验证算法/地图链路可用，**不是替代或移动题目查询**。

## 6. 决策状态机

[转移逻辑](decision/state_machine.md)、[场景测试](decision/scenarios.json)、[真实仿真转移](decision/transitions.json)、[240步数据](decision/simulation.csv)、[二维GIF](decision/simulation.gif)。

优先级：低血撤退→弹少补给→前哨失守/时间紧回防→发现敌位置移动观察→巡逻。7个情境核验包括低血和前哨失守同时出现的优先级；240步实际轨迹覆盖5状态，记录7次状态切换。没有把表格中“预期”冒充真实传感器观测，事件是明确编排的离线验证。

## 复现与记录

```bash
python qB_sentry/extended_study.py --cloud '/path/to/给新生 2/data/rmuc_2025_map.pcd' --reference-map '/path/to/给新生 2/data/map/rmuc_2025.yaml' --queries '/path/to/给新生 2/data/map/queries.json'
python tools/b_extended_evidence.py
python run.py test
```

[运行日志](run.txt)、[测试日志](tests.txt)、run_provenance.json保留输入文件哈希。leveled_cache.npz/terrain_cache.npz仅为本地中间数据，不进Git。给定图、端点、原始PCD不改写。正式报告仍须学生本人理解、写学习总结并准备无AI修改。

工具资料：2026-10-20查阅[Open3D PointCloud](https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html)、[SciPy label](https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.label.html)，本机版本另见环境记录；不承诺与文档release版本完全相同。
