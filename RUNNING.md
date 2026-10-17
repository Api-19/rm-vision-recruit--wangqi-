# RoboMaster 公共题与B题：代码阅读与运行入口

这是按2026-10-17用户要求，由Codex编写的完整代码工作区。目的是先看代码、运行结果，遇到不懂再提问。**代码可运行不表示学生已经掌握，也不是已完成提交的声明。** 原始考核数据只读，本轮按用户明确授权上传，未补造历史提交。

## 先看哪些文件

| 建议顺序 | 文件 | 读什么 |
|---|---|---|
| 1 | `qB_sentry/search.py` | 最小堆取当前最低代价→检查邻居→更新更便宜的路线→回溯；A*只多启发式 |
| 2 | `qB_sentry/grid.py` | 图片格子与世界坐标；障碍膨胀、底盘边界和连续碰撞 |
| 3 | `qB_sentry/plan.py` | 读7组查询、调用搜索、保留真实端点、导出CSV/图表/检查器日志 |
| 4 | `qB_sentry/pointcloud_map.py` | 降采样/离群剔除→多平面候选→调平→局部高度与坡度→栅格 |
| 5 | `q2_calib/calibrate.py` | 棋盘角点的3D/2D对应、标定与重投影误差 |
| 6 | `q2_calib/measure.py` | A4四角→平面双解检查→solvePnP细化→中心距离/深度与尺测比较 |

你不必按顺序手打一遍。可直接圈出函数、某几行或一张结果图来问；需要时再做少量关键修改。AI实现、学生代码阅读、学生独立实现继续分开记录。

## 本机如何运行

在终端执行（路径包含空格，保留引号）：

```bash
cd '/path/to/rm-vision-recruit--wangqi-'
PY='/path/to/existing/.venv/bin/python'
"$PY" run.py test
"$PY" run.py planning
"$PY" run.py cloud
```

不用重新安装环境。已核验的Python解释器与原视觉课程`.venv`不同。`run.py`根据自己的文件位置确定工作目录，不依赖启动时的目录。

```bash
"$PY" run.py env       # 当前环境证据
"$PY" run.py decision  # 选做：离线状态机仿真
"$PY" run.py all       # 上面各项和测试；不会启动相机或伪造标定数据
```

换了原始数据位置时用`--data-root '/path/to/给新生 2'`。`requirements.txt`为当前实际包版本，完整快照在`q1_env/results/requirements-observed.txt`。Open3D曾有本机libusb修补，包版本列表不能保证干净Mac直接重装成功，不要无故升级现有环境。

## 当前实际结果与待做事项

- 环境导入、提供的PCD与地图/视频读取已实测。
- 给定地图7组查询已实际处理：第1、2、4、5组找到路径，A*/Dijkstra结果均通过题目检查器与本程序连续圆底盘验证。各图、CSV、日志在`qB_sentry/results/given_map/`。
- 第3组起点中心在空闲格，但0.30m圆底盘碰到障碍；第6组终点有同类问题；第7组终点x=30m在图外（本图世界x最大25.57m）。这些查询没有合法端点，输出不可达原因，不移动端点或导出假路径。
- 原始2,116,092点已完成降采样、统计离群剔除、多平面搜索、调平、0.05m地图生成；过程图在`qB_sentry/results/cloud_map/`。自动地面选择与局部地表模型仍需人工检查，**不能据自动出图宣称地形分类全部正确**。
- 公共题的真实采集、尺测数据、最终标定精度尚未完成。相关代码已通过合成几何/渲染棋盘测试；测试数据没有作为考核实拍结果保存。
- 地图配准比较工具已实现，尚缺人工确认的同名地标；仿真代码已运行。Ubuntu安装、GitHub过程提交需从本次真实上传持续进行，真实采集部分报告和本人学习总结仍待补。

## 公共题一：环境和工程

`q1_env/check_environment.py`输出解释器、架构、依赖导入版本和实际包快照。用终端输出截图作为本机证据，Linux发挥题须在实际Ubuntu 22.04里运行`q1_env/linux_evidence.sh`，本机没有执行安装或假造截图。

本文件夹按`q1_env/`、`q2_calib/`、`qB_sentry/`组织，不建立嵌套Git仓库。`.gitignore`排除虚拟环境、原始大数据和私人采集照片；准备提交时选择必要的真实证据、输出图表和路径CSV，检查是否含个人信息。题目要求过程提交，但本次没有自动commit/push，也没有补造历史提交。GitHub仓库名须由本人姓名拼音确定。

## 公共题二：拍什么、怎样跑

### 1. 棋盘与采集

```bash
"$PY" q2_calib/capture.py board --cols 9 --rows 6
```

输出`q2_calib/board.png`。9×6指**内角点**，实际方格是10×7。打印贴平板，或另一屏幕显示；实际用尺测量显示/打印后的方格边长，不从图片像素猜毫米。显示后不要改变缩放。

采集15～25张本人照片，覆盖不同位置、倾角和画面边缘；同一相机、镜头、分辨率和变焦。所有角点清晰完整。手机照片可放到`q2_calib/input/chessboard/`；避免混入处理过/缩放后的照片。HEIC先以保持原始像素与方向的一致流程转为PNG/JPEG。

也可以**本人在准备好之后**手动启动摄像头工具：

```bash
"$PY" q2_calib/capture.py camera --output q2_calib/input/chessboard
```

S保存原图、Q退出；屏幕显示角点仅供预览，保存不带标注。代码不会自动在本轮访问摄像头。

### 2. 标定与去畸变

以下`25`只是命令示例，必须替换为本人实测边长：

```bash
"$PY" q2_calib/calibrate.py fit \
  --images q2_calib/input/chessboard --cols 9 --rows 6 --square-mm 25
```

默认至少15张有效角点照片才生成标定；分辨率不一致会停止，检测失败会记录。输出：

- `q2_calib/results/main/calibration.json`：K、畸变、每图位姿和像素RMS。
- `corners_*.png`、`detection_log.json`：逐图检查是否识别正确。
- `undistort_comparison.png`：原图与去畸变图；保持同一K与原尺寸。
- `calibration_quality.png`：逐图误差、估计倾角；`observations.npz`供对比实验。

RMS是拟合一致性，不能代替真实测距精度，也不能用来证明照片姿态足够多样。

### 3. A4角点与真实测量

A4保持平整，物理宽210mm、高297mm。准备至少5组约0.5～3m照片，最好保留卷尺证据。**测距参考点要先统一**：程序同时输出相机光心到纸中心的欧氏距离`center_range_m`，和沿光轴的深度`optical_z_m`。卷尺从机壳到纸边或纸面最近点的距离不能直接当二者真值；应记录光心参考近似、纸中心位置和参考点误差。

逐图标注：

```bash
"$PY" q2_calib/measure.py annotate \
  --image q2_calib/input/a4/sample1.png --output q2_calib/input/a4/sample1.corners.json
```

点击同一纸张的物理TL→TR→BR→BL，前两点之间是210mm短边；纸张在图像中旋转时仍保持物理角点对应，不能按屏幕横纵顺序随意排序。U撤销、S保存。标注原始畸变照片，不用已去畸变照片配原畸变参数。

把标注结果和真实测量填入`q2_calib/measurements.json`，它是一个列表。下面是字段示意，**所有占位值都须替换，不能提交示例数值**：

```text
[
  {
    "image": "input/a4/sample1.png",
    "corners_px": [[原图x,原图y], [原图x,原图y], [原图x,原图y], [原图x,原图y]],
    "actual_distance_m": 本人实测米数,
    "reference_metric": "center_range_m",
    "angle_deg": 本人测得的角度或null
  }
]
```

`image`相对于清单所在目录。另有空列表模板`measurements.template.json`，空清单会报错，程序不会填入猜测的真值。

```bash
"$PY" q2_calib/measure.py batch \
  --calibration q2_calib/results/main/calibration.json \
  --manifest q2_calib/measurements.json
```

输出逐图标注、位姿、平面解歧义标记、像素误差、距离误差CSV/JSON和误差图。对平面PnP先检查多个解，再用`cv2.solvePnP`细化；小像素误差不消除平面歧义。批量程序严格检查图像尺寸、角点越界与实测字段。角点标错但仍为凸四边形时仍可能得到数值，须人工核对。

### 4. 发挥对比

```bash
"$PY" q2_calib/calibrate.py compare \
  --observations q2_calib/results/main/observations.npz \
  --square-mm 25 --wrong-square-mm 20
```

使用同一批实际角点比较全部图、前5图、已有照片中近正对的子集、错误方格边长。近正对子集不足3张会明确记为数据不足；它不等于另采一整组严格正对照片的实验。

错误棋盘尺度主要缩放标定外参平移，不会理所当然把内参焦距按相同比例缩放。代码记录这个区别；不能把标定板尺度误差与单独使用正确A4尺寸的PnP尺度误差混为一谈。角度对测距影响由同距正对/30°/60°的本人照片与尺测数据分析，不能用估计角度冒充实测角度。

## B题：实现说明和参数

### 给定地图上的路径

默认机器人半径0.30m。未知区域不可走；地图外边界保留整个底盘余量。障碍格按有面积的方块处理，膨胀扣除半格对角线，再独立检查导出路径的连续底盘覆盖；较窄通道可能被保守网格排除，报告会写`disconnected_on_conservative_grid`，不能笼统声称物理上绝对不可达。

8邻居代价为格长/对角长度，禁止穿角。Dijkstra用已走代价，A*默认八方向距离下界。`search.py`自行实现，不调用规划库。优先队列保留过期项但弹出时跳过，找到更便宜路线就更新父节点；到达目标后回溯。

`run.py planning`会额外比较曼哈顿启发式和权重2；二者在8邻居下不保证最短。本图里这些实验路径与最短结果相同不证明换图后仍相同。图中扩展顺序以颜色表示；扩展数、单次耗时、路径长度见`summary.csv`与`comparison.png`，计时不是稳定性能结论。只在相同输入上比较。

根结果目录的`path1.csv`等是默认A*结果；不同算法在各自子目录。无解不输出路径文件，并删除本次对应的旧路径。查询原始端点保留，不吸附到最近空地。检查器日志`checkN.txt`来自题目原脚本。

题目检查器**即使输出“没通过”也可能退出码0**；本程序检查实际输出，并另查圆底盘边界、连续线段、原始端点。不会修改检查器。

### 点云与多层地形

默认0.05m体素、统计离群剔除、最多10个RANSAC候选平面。首次拟合是墙面；地面选择结合安装高度量级和与传感器z夹角先验，而不把“点最多”当作地面。选中法线旋到+z并保存完整4×4变换，地面候选编号/法线/内点数见`processing.json`。`--ground-plane`可覆盖自动选择，须看过程图。

局部低分位高度作为地面支持；10cm内短距离插值并以中值处理扫描稀疏，超过范围保留未知。邻格高度差、坡度、底盘0.6m内返回用于阻挡判断；高于12cm的跳变边缘不可通行。30°坡度上限是**可调实验参数**，题目没有给出机器人的最大爬坡角。

此方法是可读的2.5D原型：扫描缺失、低矮物体、稀疏支撑、上下重叠表面可能误判。填洞是地表连续性假设，不能当观测真值。`terrain.npz`保留高度/坡度/实际支持/阻挡以便分析，不拿生成地图当真值。

`--align-yaw`用大墙方向对正，**不确定与给定地图的绝对平移及180°朝向关系**。默认自建图使用自己的坐标系，不能直接将题目世界查询坐标套上去。

统计离群剔除不是语义物体识别。图中不应存在的实体需结合场地资料确认；不能仅凭形状就断言成因。确认后可提供传感器原始坐标的排除框：

```text
[{"min":[x_min,y_min,z_min], "max":[x_max,y_max,z_max]}]
```

传入`--exclude-boxes boxes.json`，被排除点数会记录；默认不自动删除猜测的异常物体。还支持`--world-transform`将已调平/对正坐标转换到经独立配准确认的世界坐标。

### 选做：自建图与给定图规划对比

在两张图上确认至少3个非共线同名地标，使用各图的世界坐标米，保存：

```text
[{"source":[自建图x,自建图y], "reference":[给定图x,给定图y]}, ...]
```

```bash
"$PY" qB_sentry/register_and_compare.py \
  --source-map qB_sentry/results/cloud_map/map.yaml \
  --reference-map '<EXAM_ROOT>/data/map/rmuc_2025.yaml' \
  --pairs landmarks.json \
  --queries '<EXAM_ROOT>/data/map/queries.json'
```

程序拟合刚性变换、报告地标残差、输出配准叠图，再在相同世界查询上分别规划。必须人工检查配准质量。此工具已通过已知变换合成验证，实际地标未输入，尚无真实配准后路径对比证据。

### 选做：状态机

`qB_sentry/decision_sim.py`考虑血量、弹量、敌方位置、剩余时间、前哨状态，输出Mermaid转移图、二维运动和状态曲线。事件是显式编排的测试场景，阈值是示例，不冒充官方规则或真实战场效果。只模拟离线移动与决策，不连接真实机器人或执行发射。

## 验证与报告

`tests/test_core.py`检查已知最短代价、随机图A*与Dijkstra等价、禁止穿角、旋转坐标往返、线段跨障碍、地面法线调平、阶跃边缘、已知几何标定/PnP、实际角点检测文件流程、错误尺度实验、测距输出、同名地标刚性配准。测试图是临时合成资料，绝不作为考核实拍证据。

`VALIDATION.md`记录本次实际检查范围；`ai_log.md`只记录真实发生的AI协作和不足，学生尚需核实并补本人分析。根README为当前设计报告初稿，仍需本人真实数据、场景解释和学习总结。

## 接口来源（检查日期2026-10-17）

- [OpenCV标定](https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html)、[PnP](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)：在线页面当前为4.13；本机cv2为5.0，另核验实际函数docstring并运行测试。
- [Open3D点云教程](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html)：在线release页当前为0.19；本机0.20的`segment_plane`、`remove_statistical_outlier`等docstring另核验，点云实际运行通过。
- [SciPy欧氏距离变换](https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.distance_transform_edt.html)：本机1.17.1运行验证。
- 题目本地`README.md`、`tools/check_path.py`、地图YAML/queries为本次规格来源，不将附件中的发布要求当自动提交授权。
