# Waypoint 控制器：仅速度预览

已实现 `waypoint_controller`，把带图像时间戳的 `/waypoint_stamped` 转换成
`geometry_msgs/msg/Twist`，仅发布到 `/cmd_vel_preview`。没有连接真实底盘。
当前版本拒绝 `dry_run=false`，也拒绝将输出重映射为 `/cmd_vel` 等运动话题。
不要用外部转发器将预览话题接入机器人；话题名称本身不是硬件安全隔离。

## 启动与检查

在 `ros2-dev` 容器内，每个终端先加载环境：

```bash
source /opt/ros/humble/setup.bash
source /workspace/SocialEmbodiedAgent/ros2_ws/install/setup.bash
```

控制器终端：

```bash
ros2 launch nomad_navigation controller.launch.py \
  project_root:=/workspace/SocialEmbodiedAgent
```

其他终端按 [推理说明](nomad_inference.md) 启动 NoMaD，再启动视频。
所有终端必须使用相同的 ROS_DOMAIN_ID 和通信范围。没有就绪输入时，控制器持续输出零值。
检查终端：

```bash
ros2 topic echo /cmd_vel_preview --once --no-daemon
ros2 topic echo /controller/status --once --qos-durability transient_local --no-daemon
ros2 topic hz /cmd_vel_preview
```

按 `Ctrl+C` 停止。修改源码或包内 YAML 后，先在工作空间重新执行普通
`colcon build --packages-up-to nomad_navigation`，同时构建新增的接口依赖；不要使用当前挂载不支持的 symlink-install。
launch 支持 `params_file`、`output_root`；节点参数启动后只读。

## 控制与默认参数

沿用本仓库作者的 `deployment/src/pd_controller.py` 前向平面控制公式：
`v = x × observation_hz`，`w = atan2(y, x) × observation_hz`，之后限幅。
它不是 ROS 2 自带控制器；此实现也没有添加误差微分项。
接近侧向的点使用原地转向；后方点直接归零，距离不超过 2 cm 的点归零。
后两项是本项目的保守处理，不声称完整复刻原作者所有分支。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `control_hz` | 20 Hz | 单调时钟控制定时器，非硬实时承诺 |
| `observation_hz` | 4 Hz | 控制换算时间尺度；与模型观测参数一致 |
| `max_linear_mps` | 0.2 m/s | 只允许前进，线速度范围 0–0.2 |
| `max_angular_rps` | 0.4 rad/s | 角速度范围 −0.4–0.4 |
| `waypoint_timeout_sec` | 0.6 s | 本机单调时间计算的 waypoint 接收超时 |
| `source_timeout_sec` | 1.0 s | waypoint 携带的源图像年龄上限 |
| `camera_timeout_sec` | 1.0 s | 相机 header 年龄及本机接收超时 |
| `future_tolerance_sec` | 0.2 s | 允许的未来时间偏差 |
| `goal_tolerance_m` | 0.02 m | 零输出距离阈值 |
| `max_waypoint_distance_m` | 5.0 m | 超出名义距离的点拒绝 |
| `waypoint_frame` | `base_link` | 只接受此局部坐标系标签 |
| `camera_frame` | `camera_optical_frame` | 相机坐标系标签 |

全部参数见包内 `config/controller.yaml`。上述速度和距离只是离线预览默认值，
没有经过机器人标定。模型已完成 waypoint 名义尺度换算，控制器不再重复缩放。
假设 x 向前、y 向左；没有执行 TF 转换，也没有验证实际相机安装方向。

## 软件保护与恢复

- 启动、没有 waypoint、模型状态不为 `running`、相机或 waypoint 超时：持续零输出。
- 错误坐标系、非有限数、非平面点、旧/未来/重复 waypoint：清除旧点并归零。
- 同时检查源图像 ROS 时间和本机单调接收时间；暂停 `/clock` 不会暂停控制 watchdog。
- ROS 时间回拨：锁定 `clock_rollback` 并保持零输出。确认时钟问题后重启节点；急停复位服务不能解除时钟故障。
- `/topoplan/reached_goal` 为 true 时归零；恢复 false 后需要新 waypoint。无目标推理节点目前不发布该完成信号。
- 日志写入失败锁定零输出，需排查存储并重启。

软件急停接口为 `/nomad/emergency_stop`（Bool），true 会锁存。
先解除输入，再显式复位；仅发 false 不会恢复：

```bash
ros2 topic pub --once /nomad/emergency_stop std_msgs/msg/Bool '{data: true}'
# 排除原因后解除输入，控制器仍保持软件急停锁存
ros2 topic pub --once /nomad/emergency_stop std_msgs/msg/Bool '{data: false}'
ros2 service call /nomad/reset_emergency_stop std_srvs/srv/Trigger '{}'
```

复位成功后还需收到新的有效 waypoint。输入仍为 true 时拒绝复位。
这不是物理急停，也不检测图像遮挡、障碍物或碰撞；碰撞传感器、底盘 mux、
加速度限制和机器人端指令 watchdog 均未实现。

SIGINT/SIGTERM 正常退出时，在 ROS 上下文关闭前发送最后一条零值并有限等待 DDS 确认。
SIGKILL、进程崩溃、卡死、主机断电或断网不能靠此节点保证停车。
未来真机必须由机器人本地独立 watchdog、物理急停和速度仲裁兜底，不能把预览验收当作真机安全验收。

## 结果归档与复验

每次控制器运行单独保存，不覆盖历史目录：

```text
outputs/controller_runs/controller__dry-run__<UTC微秒时间戳>__<随机ID>/
  cmd_vel.csv    # ROS时间、单调时间、源图像时间戳、v、w、输出原因
  events.jsonl   # 状态变化
  manifest.json  # 参数、话题、计数、最后状态
```

自动验证需先构建并加载工作空间，再在未被其他节点使用的本地 ROS 域执行：

```bash
cd /workspace/SocialEmbodiedAgent/ros2_ws
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 python3 scripts/verify_controller.py
```

脚本先用合成消息验证异常分支，再运行真实视频和 CUDA NoMaD，检查预览限幅、
推理进程退出归零、控制器正常退出最后零值及没有 `/cmd_vel` 发布器。
仅清理自己启动的进程；报告和日志写入 `outputs/controller_checks/` 独立时间戳目录。

2026-09-05 默认系统时钟联调两次遇到实际 ROS 时间回拨（一次约 3.7 秒），
控制器均锁定零输出。这证明保护生效，但默认系统时钟下完整联调尚未通过。
没有修改系统时间、网络或放宽回拨保护。

可使用测试专用 `/clock` 隔离该外部条件：

```bash
ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 python3 scripts/verify_controller.py --steady-ros-clock
```

此模式仅在测试域内发布单调 `/clock`，并对三个被测节点启用 `use_sim_time`；
仍然使用真实视频、权重和 CUDA 推理。它不验证宿主机时钟稳定性，不能替代默认模式验收。
后续仍需处理时钟稳定性、30 分钟连续运行、统一 launch、rosbag2 和 RViz 验收。

### 本次验证记录（2026-09-05）

- 最终构建成功，65 项单元测试通过，`pip check` 无依赖冲突。
- 测试单调时钟下，16 项集成检查通过；收到 12 组真实 GPU waypoint、473 条预览消息，
  观测到的最大线速度 0.2 m/s、最大角速度绝对值 0.4 rad/s。
- 相机中断使用合成消息验证；模型退出使用真实推理进程验证；最终零值由订阅端确认。
- `controller.launch.py` 独立启动、无输入零输出、SIGINT 后节点干净退出已另行验证。
- 本次测试进程已停止，没有改动系统时钟、网络、镜像或真实底盘接口。

[最终集成报告](../../outputs/controller_checks/preview-safety__20260905T085550.083489Z/report.json)、
[65 项单元测试 XML](../../outputs/controller_checks/preview-safety__20260905T085550.083489Z/pytest.xml)、
[真实链路速度曲线](../../outputs/controller_checks/preview-safety__20260905T085550.083489Z/preview.png)。
同目录保留节点日志和订阅端 CSV；报告关联独立模型运行目录和控制器运行目录。

系统时钟回拨的失败记录也保留，未用测试时钟报告覆盖：
[第一次](../../outputs/controller_checks/preview-safety__20260905T084429.262536Z/report.json)、
[第二次](../../outputs/controller_checks/preview-safety__20260905T084659.902770Z/report.json)。

同日后续已统一 Windows/WSL 校时来源，但 120 秒复测仍有 4 次回拨，默认时钟完整联调仍失败。
保护代码保持不变；不要因校时服务已配置就直接连接底盘。
详见[校时排查记录](../../docs/Windows_WSL统一校时与回拨排查_20260905-181300.md)。

### 同日重启后复验

用户授权重启 WSL/Docker 后，默认真实 ROS 时钟下 16 项集成检查全部通过，
收到 12 组真实 GPU waypoint 和 464 条速度预览消息。未发布测试 `/clock`，未放宽任何保护。
[默认时钟复验报告](../../outputs/controller_checks/preview-safety__20260905T102621.822999Z/report.json)。
这是短时验证，不代表未来不会再有时钟调整，也不代表 30 分钟稳定性或真机验收通过。
