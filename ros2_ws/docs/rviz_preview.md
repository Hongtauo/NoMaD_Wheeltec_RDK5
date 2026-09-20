# RViz 视频与 NoMaD 局部决策预览

2026-09-05 已通过用户提供的实际窗口截图确认：视频、候选轨迹、waypoint 和速度箭头可见，
`Global Status: Ok`。这是局部决策可视化，不是视频场景重建，也不是真机闭环仿真。

## Windows / Docker Desktop 启动

前提：`ros2-dev` 正在运行，项目挂载到 `/workspace/SocialEmbodiedAgent`，WSLg 可用，
本地已有 `osrf/ros:humble-desktop-full-jammy` 镜像，模型依赖和权重已安装。
源文件改变后，先在 ROS 容器中构建一次：

```bash
source /opt/ros/humble/setup.bash
cd /workspace/SocialEmbodiedAgent/ros2_ws
colcon build --packages-up-to nomad_navigation
```

Windows PowerShell 终端 1，打开窗口：

```powershell
& F:\SocialEmbodiedAgent\docker\ros2\start_rviz_viewer.ps1
```

Windows PowerShell 终端 2，启动视频、模型、控制预览和 Marker 发布：

```powershell
docker exec -it ros2-dev bash /workspace/SocialEmbodiedAgent/docker/ros2/run_rviz_demo.sh
```

也可在已有容器终端中直接运行 `bash /workspace/SocialEmbodiedAgent/docker/ros2/run_rviz_demo.sh`。
模型就绪后才开始播放。脚本固定使用域 180；查看器使用默认参数即可。
不要同时手工启动另一套模型、控制器或视频。启动前检测到同域已有业务发布者时，脚本会拒绝重复启动。

视频约 60 秒，只播放一次。播放结束后模型、控制器和可视化节点保持运行，
模型进入 `stale_input`，旧轨迹消失、速度预览归零；图像面板可能保留最后一帧，不能据此判断相机仍在线。
**重播：在终端 2 按 Ctrl+C 停止本轮 launch，再执行同一启动命令。** 不需要关闭 RViz。
不采用无间隔循环，以免模型历史观测跨越视频首尾跳变。

停止：终端 2 按 Ctrl+C；再关闭 RViz 窗口。关闭窗口会自动移除临时 `nomad-rviz-viewer` 容器，
但不会停止独立运行于 `ros2-dev` 的模型链路，也不会删除项目数据。

## 图例与坐标

| 显示 | 含义 |
| --- | --- |
| 左侧图像 | `/camera/image_raw`，本视频 320×180、BGR8、4 Hz |
| 蓝线 | NoMaD 的 8 条候选局部轨迹 |
| 绿线 | 第一条候选轨迹，即目前选取 waypoint 的来源 |
| 黄点 | 第一条轨迹第 3 个点，对应 `/waypoint_stamped` |
| 橙色箭头 / 弧线 | `/cmd_vel_preview` 的线速度 / 角速度，按 2 秒示意范围绘制；不是实际位移 |
| 灰色方块 / 白箭头 | 固定原点的机器人示意 / 前向 |

Fixed Frame 为 `base_link`，假设 x 向前、y 向左。网格和路径采用模型部署的名义尺度，
不是从视频恢复的距离。静态 `nomad_preview → base_link` 只建立预览坐标，**没有 odom 或地图定位**。
旋转/缩放视图只改变观察角度，不改变数据。模型不会根据预览速度修改录像，因此不是闭环仿真。
本工具不提供避碰或真机安全保证，不向 `/cmd_vel` 发布。

## 文件与消息关系

```text
rviz_demo.launch.py
  ├─ NoMaD（等待模型就绪后才启动 video_publisher）
  ├─ waypoint_controller → /cmd_vel_preview
  ├─ nomad_visualization → /nomad/visualization → RViz MarkerArray
  └─ 固定预览 TF
video_publisher → /camera/image_raw → NoMaD 与 RViz Image
```

可视化节点通过 `/nomad/status` 中的 `trajectory_layout` 读取样本数、步数、选点索引、坐标系和尺度。
`/nomad/trajectory_poses` 为 `PoseArray`，按“样本优先、时间步其次”展平（默认 64 个 Pose）。
位置已经乘过与 waypoint 相同的尺度，姿态仅为单位四元数占位；不可当作预测朝向。
PoseArray 与 PointStamped 必须具有完全一致的观测时间戳和 frame，选点数值也必须匹配。
`/sampled_actions` 仍保留旧的未乘尺度格式，不直接用于本显示。
Marker 使用 0.5 秒生命周期，过期、故障或时间回拨时清除旧预测。

## 空白窗口排查

1. 确认视频未播完，并且日志出现 `NoMaD ready`、`state=running`。
2. Image 显示的 `Topic` 为 `/camera/image_raw`，Reliability 为 **Best Effort**。
3. MarkerArray 显示的 `Topic` 为 `/nomad/visualization`，Reliability 为 **Reliable**。
4. Fixed Frame 为 `base_link`。Humble 配置文件使用 `Topic` 字段，不是 `Image Topic` 或 `Marker Topic`。
5. 两侧使用配套脚本启动，不能只统一 ROS_DOMAIN_ID 而忽略 DDS 配置。

查看器共享 `ros2-dev` 的网络命名空间，但不共享 IPC。因此两侧进程均加载
`docker/ros2/rviz_udp.xml`，禁用默认传输并将 UDP 接口限制为 `127.0.0.1`。
设置 `ROS_LOCALHOST_ONLY=0` 是为了避免 Humble 再次添加 SHM；本机范围由 XML 白名单保证，
**不是开放局域网通信**。这一配置仅适用于共享网络命名空间的本地预览，不用于跨机器机器人部署。
依据：[Humble RMW 实现](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/participant.cpp)。

图形容器使用只读项目挂载、WSLg X11 socket、Mesa 软件渲染；不修改现有容器或 WSL 的全局网络设置。
NoMaD 仍在 `ros2-dev` 使用 CUDA，RViz 软件渲染不意味着模型在 CPU 上推理。

## 验证与结果

- 全部 84 项单元测试通过，包含新增 Humble RViz 话题配置回归测试。
- [短时集成报告](../../outputs/rviz_checks/video-decisions__20260905T105050.398048Z/report.json)：
  120 帧图像、106 组预测，检查了观测时间戳配对、轨迹尺度、EOF 清除/归零与进程清理。
  该脚本验证消息链路，不能替代 GUI 验收；GUI 由后续用户截图确认。
- 模型归档：`outputs/ros2_runs/rviz-video__nomad-goalfree__<UTC时间戳>__<随机后缀>/`。
- 控制器归档：`outputs/controller_runs/controller__dry-run__<UTC时间戳>__<随机后缀>/`。
- [稳定性测试说明](stability_validation.md)：30 分钟结果以独立报告为准，不能把短时通过当作长期通过。
