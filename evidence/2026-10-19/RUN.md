# 本轮执行记录

解释器：用户既有考核Python3.11虚拟环境，未安装或升级依赖。

```bash
python q2_calib/calibrate.py compare --observations q2_calib/input/2026-10-18-display-27mm-19/fit/observations.npz --output q2_calib/results/display_27mm_19views_2026-10-18/variants --square-mm 27 --wrong-square-mm 20
python run.py test
python q1_env/check_environment.py --output q1_env/results
python tools/complete_evidence.py
```

以上为已执行命令的解释器无关表示。compare stdout存于calibration_compare.txt，测试和环境stdout/stderr分别存于tests.txt和environment.txt。环境日志仅将用户目录替换为占位。比较数值取实际结果，原始角点缓存仍在本地忽略目录。

另生成私人去畸变预览与文件清单，保存于input目录；不进入Git。附加题二至四依据官方来源整理，不生成虚构运行日志。

