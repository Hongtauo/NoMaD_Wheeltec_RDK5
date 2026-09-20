# 真机相机 → NoMaD 推理（仅预览，不驱动小车）

更新：2026-09-09。已接通真实 Astra 压缩彩色图像与电脑 CUDA NoMaD 推理。
本轮只验收输入/推理/waypoint 链路，没有启动底盘、速度控制器或 RViz 真机轨迹显示。

## 当前需要的容器

只需要 **`nomad-robot-inference`**。不需要启动 `ros2-dev`、旧离线模型容器或 `nomad-fastdds-extlab-20260909`。
运行前关闭之前的 `nomad-robot-rviz`，不要并行运行 camera_check、ros2 echo/daemon 等使用相同固定端口配置的进程。
当前一个 Python 进程中的推理节点和审计节点共用一个 ROS Context/DDS participant；不是“每个节点开一个容器”。

- 本地镜像：`socialembodiedagent/nomad-ros2-runtime:robot-preview-20260909`。
- 镜像 ID：`sha256:352389d40d534feb0818d267ae8a655d2d3184921e83674d2a50dc1ef8f12a97`。
- 来源：停止状态的 `ros2-dev` 已安装依赖层的本地快照，不是从 Dockerfile 全新构建验收；未上传镜像，原容器未改动。
- 新容器：host 网络、GPU all、8 GiB shm、项目读写挂载到 `/workspace/SocialEmbodiedAgent`。
- 系统 Python 3.10；Humble / 原版 Fast DDS 2.6，未使用实验 2.10 overlay。
- GPU 实测 RTX 4070 Ti SUPER；PyTorch 2.6.0+cu124、CUDA runtime 12.4，`pip check` 通过。
- 新代码构建到容器 `/opt/nomad_robot_ws/install`，不覆盖原 `ros2_ws/install`。工作区源码和权重仍来自挂载目录。

如以后需要重建这个已删除的容器，可从已保存的本地镜像运行下列命令；**现有容器无需重复创建**：

```powershell
docker run -d --init --pull never --name nomad-robot-inference `
  --network host --gpus all --shm-size=8g `
  --mount "type=bind,source=F:/SocialEmbodiedAgent,target=/workspace/SocialEmbodiedAgent" `
  -w /workspace/SocialEmbodiedAgent -e TZ=Asia/Shanghai `
  --label org.socialembodiedagent.role=robot-inference-preview `
  socialembodiedagent/nomad-ros2-runtime:robot-preview-20260909 sleep infinity
```

## 启动前

1. 电脑连接小车热点，当前电脑 Wi-Fi 为 `192.168.0.180`，小车为 `192.168.0.100`；Docker Desktop host networking 已启用。
2. 小车厂商环境中运行 `ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py`，已有相机进程时不要重复启动。
3. 小车与电脑时间应接近。小车有重启回到 2000 年的问题，长期自动同步尚未解决；时间不对时本入口会拒绝，不允许用“重新打时间戳”或增大超时来绕过。
4. `robot_wifi_only.xml` 固定 participant 10、UDP 7430/7431；整个共享 host 网络同时只运行本入口，不能直接并行上一条 RViz 命令。
5. 当前 runner 严格检查已验收的 IP/端口组合。IP 改变时，需要重新核验 XML 和 runner 的 `validate_profile` 地址约束，而不是只改一个 shell 环境变量。

## Windows 一条命令运行

在项目根目录 PowerShell 中：

```powershell
cd F:\SocialEmbodiedAgent
.\docker\ros2\run_robot_inference.ps1 -Seconds 60
```

脚本会核对地址、容器网络/GPU/挂载，必要时启动现有容器，重新构建源码，再执行有限时长推理。
`-Seconds` 指模型加载完成后的测试时长，范围 10–3600 秒；默认 60 秒。每次输出新目录，不覆盖历史结果。
模型就绪后先积累四帧观测，再开始 waypoint 输出；审计节点每 10 秒报告接收与发布数量。

容器内等价入口（先完成构建；新终端不要 source 2.10 overlay）：

```bash
source /opt/ros/humble/setup.bash
source /opt/nomad_robot_ws/install/local_setup.bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/SocialEmbodiedAgent/docker/ros2/fastdds_extlab/robot_wifi_only.xml \
python3 /workspace/SocialEmbodiedAgent/ros2_ws/scripts/run_robot_inference.py --seconds 60
```

正常到时后节点及 Context 自动退出，容器保留运行，没有推理后台任务。可用 `docker stop nomad-robot-inference` 停止空闲容器。
Windows 的 `docker exec` 客户端 Ctrl+C 不保证信号转发给内部进程；如需立即停止，另开 PowerShell 执行上述 `docker stop`。不要仅关闭终端就认为推理一定停止。

## 数据关系与话题

```text
小车 /camera/color/image_raw/compressed（约30Hz）
  → 电脑 NoMaDNode：JPEG → RGB，保留相机原始 stamp/frame_id
  → 约4Hz抽样的4帧上下文 → CUDA NoMaD（96×96输入）
  → /nomad/robot_preview/... 私有预览输出
  → 同进程审计节点 + 每次独立结果归档
```

| 话题 | 类型/含义 |
| --- | --- |
| `/nomad/robot_preview/waypoint` | Float32MultiArray，选中的二维点 |
| `/nomad/robot_preview/waypoint_stamped` | PointStamped，保留对应输入图像时间戳 |
| `/nomad/robot_preview/sampled_actions` | Float32MultiArray，8条候选×8步×2维（含原兼容前缀） |
| `/nomad/robot_preview/trajectory_poses` | PoseArray，标称尺度下的候选位置 |
| `/nomad/robot_preview/status` | 模型状态 |

没有 `/cmd_vel`、Twist 发布者或控制器，也没有发布全局 `/waypoint`，避免混入小车已有接口。
私有命名与 XML **不是 DDS 权限隔离或物理安全机制**；本轮靠不构造控制器/速度发布者和图审计限制范围，不应直接用于真机运动。

`base_link` 是模型输出的名义局部坐标标签，并未完成相机外参到车体的标定验证。当前尺度 0.05 来自原部署公式，
不等于已标定的实际距离；输出图不是小车已经走过的路线，也不是安全避障保证。

## 保护与验收

- `image_transport` 默认为 `raw`，本入口显式选择 `compressed`；原视频流程保持兼容。
- 只读参数、JPEG/RGB检查；无效/陈旧/未来帧、时间回拨、frame_id 变化与观测不连续清理上下文。
- 输入超时仍为 1 秒、最大帧间隔 0.75 秒、未来容差 0.2 秒；推理/记录期间过期的结果不会发布。
- 启动前检查固定端口空闲、至少3张不同的新鲜相机图像、私有输出话题未被其他进程占用。
- 验证实际 CUDA runtime、模型发布话题白名单，无 Twist；实际 DDS waypoint 与归档逐条匹配，候选与PoseArray形状/时间戳检查。
- 相机无效、连续断流超过1秒或模型 fault 时测试失败并清理；不自动更改网络、时钟或小车配置。
- 最终入口增加持续出点检查：模型就绪后给予5秒启动宽限，之后实际 DDS waypoint 超过2秒未到达即失败。此验收检查没有放宽模型原有的1秒输入/点时间戳保护。

## 结果位置与绘图

```text
outputs/ros2_robot_inference/robot-camera__UTC微秒__UUID/
  report.json                  # 整次验收、计数、时延与运行环境
  dds_points.json              # 审计节点实际收到的点
  model/robot-camera__nomad-goalfree__UTC__UUID/
    manifest.json
    inference.jsonl
    events.jsonl
    windows/attempt_*.npz      # 当前RGB、原图像时间戳、候选轨迹和选中点
  preview__UTC.png             # 另行生成的画面/候选路径对照图
```

独立绘图，不运行 ROS，也不覆盖先前图：

```bash
python3 /workspace/SocialEmbodiedAgent/ros2_ws/scripts/plot_robot_inference.py \
  /workspace/SocialEmbodiedAgent/outputs/ros2_robot_inference/实际运行目录名
```

相机 age/waypoint age 含跨机时钟偏差和处理/排队时间，不等于纯网络时延；
审计图像回调率受同步推理及 depth=1 影响，不应当作相机硬件掉帧率。NoMaD 只需约4Hz观测，并非处理每一张30Hz图像。

## 已完成与未完成

- 95项包单元测试在源码和新容器构建安装目录分别通过；启动器13项纯逻辑测试在 Windows/Linux 均通过。
- 首次60秒真机测试：238组实际 DDS waypoint，平均推理122.90ms，waypoint age p95约333ms，模型正常停止。
- [首轮验收](../../outputs/ros2_robot_inference/robot-camera__20260909T111205.015484Z__60e63b93/report.json)。
- 180秒复测：712组实际 DDS waypoint，平均推理126.78ms、p95 158.71ms；点间隔平均251.73ms、最大494.06ms，waypoint age p95约341ms。模型事件仅等待输入→运行，无上下文重建或模型fault。
- [180秒报告](../../outputs/ros2_robot_inference/robot-camera__20260909T111414.783080Z__c7a4cf76/report.json) / [相机与候选轨迹](../../outputs/ros2_robot_inference/robot-camera__20260909T111414.783080Z__c7a4cf76/preview__20260909T111822.553140Z.png)。
- 60/180秒结果生成于补充“持续出点检查”之前；最终入口另做30秒真机复测通过，115组点，含 `continuous_preview_output` 验收项：[最终版本报告](../../outputs/ros2_robot_inference/robot-camera__20260909T111809.810268Z__ba112518/report.json)。
- 测试后已核验无推理进程，固定7430/7431端口释放；新 GPU 容器空闲保留，旧实验容器停止，小车相机继续运行，原 ros2-dev 未改动。
- 尚未验收真机 RViz 轨迹、多 participant 网络、自动校时、相机/车体标定、底盘运动或群组感知效果；当前无目标探索沿用原始 NoMaD。
