# 环境与工程证据

- `results/environment.json`：本机真实导入检查，个人绝对路径已在上传副本中归一化。
- `results/requirements-observed.txt`：实际包快照。
- `check_environment.py`：重跑证据生成；不安装/修改系统。
- `linux_evidence.sh`：仅供实际Ubuntu 22.04中运行，当前未安装/验证Ubuntu。

环境由Claude辅助准备，Codex后验验证，详见根AI记录。本机Open3D曾修补libusb加载，包列表不足以证明干净机器可复现。仓库的真实提交从本次上传开始，不伪造此前过程提交；后续持续更新。
