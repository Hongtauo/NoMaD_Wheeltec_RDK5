# NoMaD ROS 2：图像到 waypoint

已实现无目标探索模式的 `nomad_node`，沿用仓库 `deployment/src/explore.py` 的模型流程。
配套已有[预览控制器](controller_preview.md)，仅输出 `/cmd_vel_preview`。
没有实现拓扑目标导航、避碰保障或真机闭环控制；本推理节点自身不发布速度。

## 启动

使用 `ros2-dev` 容器中的系统 Python。每个新终端均先加载环境：

```bash
source /opt/ros/humble/setup.bash
source /workspace/SocialEmbodiedAgent/ros2_ws/install/setup.bash
```

终端 1：先启动推理节点：

```bash
ros2 launch nomad_navigation nomad.launch.py \
  project_root:=/workspace/SocialEmbodiedAgent
```

等待 `NoMaD ready` 和 `state=waiting_for_images`，再在终端 2 启动视频：

```bash
ros2 launch nomad_navigation video.launch.py \
  video_path:=/workspace/SocialEmbodiedAgent/offline_data/3-r.mp4
```

积累至少 4 帧后，终端 1 应显示 `state=running` 和 `waypoint[...]`。
两个终端的 `ROS_DOMAIN_ID`、RMW 和网络范围应一致。保持底盘控制器未连接，先检查数值。

终端 3：读取结果（同样先加载环境）：

```bash
ros2 topic echo /waypoint std_msgs/msg/Float32MultiArray --once --no-daemon
ros2 topic echo /waypoint_stamped geometry_msgs/msg/PointStamped --once --no-daemon
ros2 topic hz /waypoint
```

视频默认播放一次后退出；NoMaD 在输入过期后进入 `stale_input` 并等待新图像，
不会自行退出。重启视频后重新积累完整上下文才恢复输出。按 `Ctrl+C` 停止各节点。
不要让这段视频循环后无缝衔接旧上下文：循环的内容跳变不一定能从 header 检出，
当前联合使用请保持 `loop=false`。

## 输入与输出约定

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 兼容 sensor-data QoS 的图像输入；接收队列深度 1 |
| `/waypoint` | `std_msgs/msg/Float32MultiArray` | 两个数 `[x, y]`，选择并缩放后的局部点；没有 header |
| `/sampled_actions` | `std_msgs/msg/Float32MultiArray` | 官方兼容格式：`0` 前缀 + 8×8×2 个展平坐标，默认共 129 个数 |
| `/waypoint_stamped` | `geometry_msgs/msg/PointStamped` | 与 waypoint 相同的 x/y，z=0；携带最后一帧观测时间及 `base_link` 标签 |
| `/nomad/trajectory_poses` | `geometry_msgs/msg/PoseArray` | 默认 64 个位置，样本优先展平，已乘 waypoint 尺度，与 PointStamped 的 header 完全一致；姿态为占位 |
| `/nomad/status` | `std_msgs/msg/String` | JSON 状态及本次输出目录，transient-local；在状态改变时发布 |

`/sampled_actions` 采样轨迹已经过动作解归一化和累加，但尚未乘机器人速度尺度。
新增的 `/nomad/trajectory_poses` 则已经乘过该尺度，不能重复缩放。
`/nomad/status` 还包含 `trajectory_layout`（样本数、步数、选点索引、frame 和已用尺度），
供 [RViz 可视化节点](rviz_preview.md)校验和配对。
默认选取第一条轨迹的第 3 个点，即 `actions[0, 2]`。
当模型配置 `normalize=true` 时：

```text
waypoint = actions[0, waypoint_index] × max_speed_mps / observation_hz
默认尺度 = 0.2 / 4.0 = 0.05
```

这是继承官方部署的名义尺度，不是相机测距或真机标定结果。
`max_speed_mps` 在此仅用于尺度换算，不是在控制底盘，也不是对 waypoint 做限幅。
局部坐标假设 x 向前、y 向左，`base_link` 仅是配置标签：没有执行相机到机器人的 TF 变换，
真机必须确认相机朝向、坐标约定和尺度。

## 模型与预处理

- 模型配置 `context_size=3`，实际输入为历史 3 帧加当前帧，共 4 帧。
- 使用 `cv_bridge` 将输入转换为 RGB，不把 BGR 当作 RGB。
- 按官方部署直接 BICUBIC 缩放为 96×96，不中心裁剪；做 ImageNet 归一化，
  拼成 `(1, 12, 96, 96)` 输入。
- goal mask 全为 1，忽略目标图；默认采样 8 条轨迹、每条 8 个二维点、10 步 DDPM。
- 严格加载现有本地 checkpoint，使用 `weights_only=True`；不自动下载权重。
- 此节点目前仅支持官方 `nomad_vint` 编码器配置，非拓扑导航模式。
- 种子控制整个 DDPM 噪声序列。官方预处理及动作解码已逐元素对比通过，
  但不承诺与先前离线脚本输出轨迹逐元素一致：先前脚本使用 BILINEAR，
  本节点匹配官方部署的 BICUBIC，且 DDPM 随机数控制方式有差异。

## 参数

节点参数在启动后只读。默认配置在包内 `config/nomad.yaml`。
模型相关 Python 依赖及两个 editable 源码仓库沿用
[系统环境安装说明](../../docker/ros2/README.md)，不能只复制 ROS 包就假设模型依赖齐全。
不通过 rosdep 自动安装另一套 PyTorch，也不创建 Conda/venv。

launch 的 `project_root` 用于计算默认路径，可另外覆盖 `model_config`、`checkpoint`、
`action_stats_path`、`output_root`，无需修改 Python 源码；`params_file` 可指定自定义 YAML。

关键节点参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `device` | `cuda` | CUDA 不可用则明确报错，不静默切换 CPU；可显式指定 CPU，但需重新评估频率和超时 |
| `warmup` | `true` | 订阅相机前预热，预热结果不发布 |
| `observation_hz` | `4.0` | 观测采样目标频率，也参与尺度换算 |
| `inference_hz` | `4.0` | 推理定时器目标频率，不超过观测频率；并非硬实时承诺 |
| `num_samples` | `8` | 候选轨迹数，1–64 |
| `waypoint_index` | `2` | 轨迹上的点索引，从 0 开始 |
| `max_speed_mps` | `0.2` | 名义 waypoint 尺度换算参数 |
| `image_timeout_sec` | `1.0` | 最新观测超时阈值，推理和记录后再次检查 |
| `max_frame_gap_sec` | `0.75` | 观测间隔超过此值则清空历史 |
| `future_tolerance_sec` | `0.2` | 容许的未来 header 时间偏差；真机需要时钟同步 |
| `seed` | `0` | 按推理尝试次数递增的随机种子基值 |
| `run_name` | `nomad-ros2` | 独立运行目录的场景前缀 |

图像、waypoint、采样轨迹、带时间戳 waypoint、状态话题及输出 frame_id 均可配置。
观测降采样设有时间抖动容差；时间戳重复/倒退、超过允许的帧间隔或 frame_id 改变会清空历史重建。

## 超时与故障边界

- 模型未就绪、上下文不足、没有新观测时不发布 waypoint；不会重复发布同一帧的结果。
- 输入过期、消息损坏、观测时序中断时清空历史，等待重新积累。
- 推理或记录结束后再次检查新鲜度，过期结果丢弃。
- 推理异常（包括 CUDA OOM）、非有限值或错误轨迹形状会锁定为 `faulted`，
  停止推理，记录错误，需人工排查后重启；不会自动无限重试占用 GPU。
- waypoint 类输出采用 reliable、volatile、队列深度 1，并设置 1 秒默认消息 lifespan。
  无时间戳的 `/waypoint` 仍不适合独立判断源图像年龄，配套预览控制器采用 stamped 输出。
- **停止发布不等于机器人自动停车。** 配套控制器已按时间戳和接收超时主动发送零值预览，
  并提供软件急停锁存；物理急停、底盘 watchdog 和真实速度接口尚未接入。

## 运行结果与验收

每次启动都在 `outputs/ros2_runs/` 创建唯一目录，格式：

```text
<run_name>__nomad-goalfree__<UTC时间戳微秒>__<随机短ID>/
  manifest.json          # 参数、模型、实际设备、状态、发布总数
  model_config.yaml
  action_stats.yaml
  events.jsonl           # 状态、清空历史、丢弃结果、错误
  inference.jsonl        # 每次发布的时间、waypoint、耗时、种子与归档路径
  windows/attempt_*.npz  # 候选轨迹、选中点、观测时间戳与当前 RGB 图像
```

推理尝试不一定都会发布；以 `inference.jsonl` 记录为已调用发布接口的结果。
每次尝试独立命名，不覆盖丢弃结果的归档。普通启动的控制台日志仍由 ROS launch 保存。

2026-09-05 已完成：33 项单元测试通过；实际 GPU 测试发布/接收到 26 组 waypoint，
平均推理 119.85 ms、中位数 116.81 ms。排除主动中断间隔后，发布间隔中位数约 0.248 秒。
图像中断停止、坏消息/旧消息拒绝、恢复后重建上下文、无 Twist 发布和干净退出均通过。
OOM 分支采用单元测试故障注入验证，没有人为耗尽真实 GPU 显存。
这不是 30 分钟长时间联调或真机安全验收。

- [真实 GPU 集成报告](../../outputs/ros2_nomad_checks/3-r__image-to-waypoint__20260905T081302.748641Z/report.json)
- [图像、候选轨迹及 waypoint 可视化](../../outputs/ros2_nomad_checks/3-r__image-to-waypoint__20260905T081302.748641Z/summary.png)
- [本次节点运行记录](../../outputs/ros2_runs/3-r-integration__nomad-goalfree__20260905T081304.626521Z__774d5832/manifest.json)

在工作空间构建并加载环境后，可复验（仅限当前项目和测试视频，不连接底盘）：

```bash
ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1 python3 scripts/verify_nomad.py
```
