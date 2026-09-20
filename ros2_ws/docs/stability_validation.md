# NoMaD 预览链路稳定性验证

运行脚本 `ros2_ws/scripts/verify_stability.py` 使用真实 GPU、默认系统 ROS 时间和原有控制器保护。
默认测试 1800 秒，从收到至少 8 组预测后开始用单调时钟计时；模型与控制器全程不重启。
有限视频分轮重播，每次 EOF 后验证 `stale_input`、预览零速和旧路径清除，再等待至少 2 秒开始下一轮。
所以这是“持续运行的节点 + 有计划的输入间隙”测试，不是 30 分钟无间断相机流。

## 运行

先停止占用 GPU 的旧演示 launch，以免资源统计混入重复的 NoMaD 进程。
在 Windows PowerShell 前台运行，Ctrl+C 可中断：

```powershell
docker exec -it ros2-dev bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/SocialEmbodiedAgent/ros2_ws/install/setup.bash && python3 /workspace/SocialEmbodiedAgent/ros2_ws/scripts/verify_stability.py'
```

脚本自动使用域 183、与 RViz 预览相同的回环 UDP 配置。启动前拒绝已有业务发布者。
它只启动并清理自己创建的测试进程，不删除旧结果，不启动 GUI，不输出真实运动指令。
`--duration-sec 70` 用于验证测试器本身，不能算作 30 分钟验收。

## 输出和判定

每次结果写到 `outputs/stability_checks/nomad-preview__<UTC时间戳>/`：

- `report.json`：持续刷新状态、计时、计数，结束后写最终结论和退出检查。
- `telemetry.jsonl`：约每 10 秒记录计数、近期频率、进程 CPU/RSS、系统内存、磁盘余量和 GPU 指标。
- `events.jsonl`：模型/控制器状态、每轮 EOF 保护与重播事件。
- `combined_launch.log`、`video_NNN.log`、`ros_logs/`：独立启动日志。
- 模型、控制器仍写各自带时间戳的原有归档；模型目录记录在报告内。

自动检查：达到指定单调时钟时长、主 launch 存活、输入有效时推理未持续停滞、
预览/Marker 心跳不超过 2 秒中断、waypoint/速度有限、速度不超 0.2 m/s 和 0.4 rad/s、
没有检测到 ROS 时间回拨或模型 fault、没有 `/cmd_vel` 发布者、平均预测速率（含间隙）至少 2 Hz、
结束后测试进程正常退出且无业务发布者残留。EOF 每轮检查归零和轨迹清除。

资源数据需要结合前后趋势解释：CPU 百分比可超过 100%（多核总和），RSS 是进程之和，
可能重复计算共享页；`nvidia-smi` GPU 占用是整卡统计，包含其他程序。首轮 CPU 采样为初始化值。
本测试不是内存泄漏的严格证明，也未验收 RViz 持续渲染、rosbag2、跨机器网络或真实机器人安全。

## 2026-09-05 正式结果

北京时间 19:18:48 开始，19:48:50 完成清理，实际测量 1800.054 秒，报告 `status=complete`。
9 项检查通过；28 轮播放中完成 27 次 EOF 保护，测试进程正常退出，无残留业务发布者。
计时区间收到 6460 组 waypoint，含重播间隙平均 3.589 Hz；速度预览平均 20.006 Hz。
归档推理平均 113.74 ms、P95 144.95 ms。预热后 RSS 约 1909 MiB，前后活跃窗口均值基本持平，
整卡显存 5184–5727 MiB；未观察到持续增长，但不是严格的泄漏排除证明。

最后一轮存在一次 `observation_discontinuity` 上下文重置，控制器归零后约 1.3 秒恢复。
没有时间回拨或模型故障；该抖动根因尚未定位，应在连续相机输入/真机前继续诊断，不能称为全程无中断。

[原始报告](../../outputs/stability_checks/nomad-preview__20260905T111834.567394Z/report.json) ·
[结果、资源趋势和局限总结](../../outputs/stability_checks/nomad-preview__20260905T111834.567394Z/summary.md)。
本次不包含 GUI 30 分钟渲染、rosbag2、跨机器或真机验证。
