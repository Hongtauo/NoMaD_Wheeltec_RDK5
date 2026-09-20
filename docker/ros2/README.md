# ROS 2 Humble + NoMaD 系统 Python 环境

使用 `/usr/bin/python3`（3.10），不创建 Conda 或 venv。Humble、OpenCV 和
`cv_bridge` 由 apt 管理；PyTorch 与模型依赖通过 `python3 -m pip` 安装。

当前校园网部署只有电脑与小车 **两个物理节点**：电脑上的 `nomad-campus-api`、`nomad-campus-rviz` 是两个容器，不是两台额外设备；小车控制与安全网关直接在车端 ROS 2 运行。WSL／Docker Desktop 属于电脑的软件运行环境。完整职责见[校园网 API 架构](../../ros2_ws/docs/campus_api.md#地址与职责)，日常启动见[校园网启动说明](../../docs/校园网导航启动与暂停恢复_20260919-005412.md)；本页保留环境安装与历史验证说明，不是当前校园网服务的完整启动清单。

## 时区

2026-09-05 已将现有 `ros2-dev` 和项目旧 NoMaD 容器的本地时区设为 `Asia/Shanghai`
（北京时间 UTC+08:00），与 Windows、Ubuntu WSL 一致；不改变 Unix 时间戳或校时来源。
ROS 2 Dockerfile 也已加入该配置，尚未通过新镜像构建验收。
旧 NoMaD 容器未安装 tzdata，使用 ROS 容器内的上海 TZif 数据作为 `/etc/localtime`，
并设置 `/etc/timezone`；未安装额外软件包。容器重启保留，删除重建则需要重新配置。
明确使用 UTC（带 `Z`）的运行归档名称仍保留 UTC，历史结果不重命名。
时区调整不会修复 WSL 时钟回拨，回拨保护继续保留。

`requirements.txt` 固定直接依赖；NumPy 固定为 1.24.4，以兼容 Humble 的
NumPy 1.x 图像扩展和系统 SciPy 1.8 的 `<1.25` 要求。
Matplotlib 固定为 3.7.5，避免 3.8+ 的命名空间变更与系统 Matplotlib 冲突。
保留 setuptools 59.6.0，避免影响 colcon/ament。
CUDA 运行库由 PyTorch wheel 提供，本推理流程无需安装 nvcc 或容器内显卡驱动。

`constraints-py310.txt` 记录已验证的 pip 层传递依赖版本，安装时自动使用。
验证报告中的 `pip-freeze.txt` 是原始环境快照，含 ROS apt 包和临时 wheel 来源，
不应直接作为新环境的 `pip install -r` 输入；重建请使用本目录的安装脚本。

## 安装和验证

在已挂载项目的 `ros2-dev` 容器内运行：

```bash
bash /workspace/SocialEmbodiedAgent/docker/ros2/install_dependencies.sh
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  /workspace/SocialEmbodiedAgent/docker/ros2/verify_environment.py
```

验证包含同进程导入、CUDA 矩阵运算、torchvision CUDA 扩展、RGB/BGR ROS
图像转换、SciPy 运算、Matplotlib 3D 绘图、ROS 节点创建、talker/listener 和 `pip check`。
报告、包版本快照及测试日志写入 `outputs/environment_checks/` 下的唯一 UTC 时间戳目录。
测试仅发布演示消息，不发布底盘控制指令。

用已有视频抽帧结果进行短推理测试：

```bash
/usr/bin/python3 /workspace/SocialEmbodiedAgent/visualnav-transformer/offline_inference.py \
  /workspace/SocialEmbodiedAgent/offline_data/3-r_frames \
  --max-windows 10 --run-name 3-r-ros2-env-check
```

脚本自动在 `offline_outputs/` 下分配唯一目录。去掉 `--max-windows 10` 可运行全部窗口。

下载速度较慢时，可先安装 `aria2`，运行 `python3 docker/ros2/prefetch_cuda.py`。
该工具从 PyTorch 官方索引与 PyPI 取得 wheel 地址和 SHA256，支持分段续传；
下载全部成功后执行：

```bash
NOMAD_WHEELHOUSE=/tmp/nomad-ros2-wheelhouse bash docker/ros2/install_dependencies.sh
```

`NOMAD_WHEELHOUSE` 指向本地已下载 wheel，未指定时安装脚本直接使用官方 PyTorch 源。

## 已完成验证（2026-09-05）

`ros2-dev` 的系统 Python 安装和上述环境检查均已通过。
对 `3-r_frames` 的完整 236 窗口复验得到 `(236, 8, 8, 2)` 轨迹，
与原离线容器基线逐元素一致，平均推理耗时 133.55 ms/窗口。
这证明当前离线模型环境可用，不代表 ROS 2 推理节点或真机控制已经实现。

- [环境报告](../../outputs/environment_checks/ros2-dev__20260905T061011.368002Z/report.json)
- [轨迹与性能对比](../../offline_outputs/3-r-ros2-system-py310__nomad-goalfree__20260905T061051Z/comparison.json)
- [部署计划与后续任务](../../docs/NoMaD_ROS2核心部署计划_20260904-214702.md)

本次约 2.9 GB 临时 GPU wheel 下载缓存已清理，已安装依赖不受影响；
原始地址及 SHA256 保存在环境报告目录的 `gpu-wheel-sources.txt`，可重新下载。

## Dockerfile 初稿

Dockerfile 复用上述依赖安装脚本。当前阶段先验证现有容器；新镜像的完整构建和
全新容器验收属于后续镜像固化阶段，不能用本次容器验证代替。

在 Windows 项目根目录执行：

```powershell
docker build -f docker/ros2/Dockerfile -t socialembodiedagent/nomad-ros2:humble-cu124 .
```

镜像不复制项目源码和权重，运行时须继续挂载
`F:\SocialEmbodiedAgent:/workspace/SocialEmbodiedAgent`。
当前容器通过 editable install 使用源码；Dockerfile 使用 `PYTHONPATH` 暴露同一源码目录。
两种方式均要求这两个仓库位于项目目录：`visualnav-transformer`、`diffusion_policy`。

PyTorch 版本组合参考：[官方历史版本安装说明](https://pytorch.org/get-started/previous-versions/#v260)。
Matplotlib 兼容性依据：[官方 3.8 命名空间变更说明](https://matplotlib.org/stable/api/prev_api_changes/api_changes_3.8.0.html)。
