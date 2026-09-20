# NoMaD ROS 2 工作空间

`ament_python` 包包含 `smoke_node`（心跳日志）、`video_publisher`（本地视频图像输入）、
`nomad_node`（NoMaD 无目标探索推理）、`waypoint_controller`（速度预览及软件保护），
以及 `nomad_visualization`（RViz 局部决策图形）和模型就绪等待工具。
工作空间还包含 `ament_cmake` 接口包 `nomad_interfaces`，为真机按次解锁定义 `ArmFor.srv`。
既有视频/本机预览流程已实现图像 → `/waypoint_stamped` → `/cmd_vel_preview`，不向真实底盘发布指令；
新增真机控制入口和安全边界见下一节。

## 校园网 API 模式（2026-09-18 新增）

小车通过 HTTPS 调用 PC 上的 NoMaD，跨设备不再依赖 DDS；机器人保持 DHCP。
当前只有 **两个物理节点**：电脑承担 GPU 推理、API、RViz 和操作台，小车承担感知、API 调用、本地控制、安全网关与底盘执行。
两个电脑容器和多个 ROS 2 软件节点不另算物理设备；组件位置与通信边界见[物理设备与软件运行层](docs/campus_api.md#物理设备与软件运行层)。
当前服务器 `https://10.201.91.120:8443`，专用容器 `nomad-campus-api`；旧热点模式仍保留。
详见 [校园网 API 部署与操作](docs/campus_api.md)。服务端只推理，车端仍由独立控制器和 gateway 保护。
新增入口默认未解锁、0.05m/s，不能将短时相机/API测试当成运动或长期安全验收。
以下为 2026-09-18 首次部署历史记录：当时为无图形窗口 CUDA 服务，固定推理线程、正式故障锁存及分段诊断已部署，524 项回归通过。
当时暂停其他推理后的 180 秒测试取得 540 个有效结果，但仍有 4 次请求失败及 16 次过期；后续短测定位到请求接收延迟，当轮保持待机、未完成连续新鲜度和运动验收。
最新部署状态、暂停恢复策略及启动流程以[校园网启动说明](../docs/校园网导航启动与暂停恢复_20260919-005412.md)为准，不能把上述历史记录当作当前服务运行状态。

## 旧热点模式：真机实时推理 + RViz / 独立机器人控制

PC 只需要专用 `nomad-robot-runtime` 容器，启动推理和 RViz；机器人本机分别运行
`robot_control`、独立 `robot_gateway`、相机、厂商最小底盘驱动及 TF。多个节点保持独立进程，日常只需两个常驻 PowerShell 窗口。
先解除使能、确认车辆静止，并在原终端正常停止旧 PC 预览和机器人节点；统一入口遇到已有相关进程会拒绝校时，不强杀旧会话。

窗口 1：校时 → 启动默认未解锁的机器人节点组 → 控制菜单。Windows 项目根目录运行：

```powershell
.\docker\ros2\start_robot.ps1
```

默认线速度**指令上限**为 `0.2 m/s`；回退可用 `-MaxLinearMps 0.05`。只需校时且不启动相机/底盘/控制器/网关时，用
`.\docker\ros2\start_robot.ps1 -SyncOnly`。按 OpenSSH 提示输入 SSH 密码，连接后另输 sudo 密码，不保存密码。
以 Windows 系统 UTC 为基准，检查 Docker 时间，校时误差加测量不确定性须不超过 `0.1 s`；有旧 ROS/硬件进程时拒绝改时钟。
只暂时停止原先活动的 NTP 服务，不 disable、不改其配置、不写 RTC，保留北京时间时区；结束整组并确认相关 ROS 进程全停后恢复原服务。
`pending_services`、`cleanup_failed` 或恢复不确定的错误必须人工处理，不能把关闭窗口视为恢复成功。

窗口 2：等窗口 1 完成校时并进入菜单，再启动电脑推理/RViz；PC 不发布真实速度、不自动 arm：

```powershell
.\docker\ros2\run_robot_live.ps1 -Seconds 0
```

窗口 1 菜单：`i` 查看状态、`t` 输入 `(0, 10]` 秒、`c` 连续、`s` 停止运动、`q` 停止整组并恢复原校时服务。
每次 `t`/`c` 都要求输入 `RUN`，确认现场看护、急停可用、车辆静止、区域清空及图像/轨迹正常；启动不会自动 arm。
控制台心跳超过 4 秒失联会请求解除使能并收尾，只是软件保护，不是实际 4 秒制动保证。正常结束先 `s` 并确认静止，再关闭窗口 2 的预览，最后 `q`。

以下原始服务命令保留用于排错；使用菜单时不必另开 SSH 操作，也不要在已启动的节点组之外重复运行节点：

机器人控制默认 disarmed；新增 `/arm_for` 可每次选择定时或连续挡位，无需重启控制器。
定时允许 `0 < duration_sec ≤ 10` 秒；连续必须显式指定 `mode: 1, duration_sec: 0.0`，仅取消时长截止，不取消任何停车保护。
现场确认急停可用、区域清空、图像/轨迹正常、车辆静止后，在机器人 SSH 终端按需选择一种请求：

```bash
source /home/wheeltec/nomad_live_control/setup.bash

# 本次最多 5 秒
ros2 service call /nomad/robot_control/arm_for \
  nomad_interfaces/srv/ArmFor '{mode: 0, duration_sec: 5.0}'

# 连续挡位：不能在上一轮仍运行时调用
ros2 service call /nomad/robot_control/arm_for \
  nomad_interfaces/srv/ArmFor '{mode: 1, duration_sec: 0.0}'

# 两种挡位共用停止命令
ros2 service call /nomad/robot_control/disarm std_srvs/srv/Trigger '{}'
```

运行中拒绝续期/切挡；故障后不自动恢复，必须重新手动解锁。旧 `/arm` 的 `Trigger {}` 接口仍使用 `arm_lease_sec`，
默认 3 秒、启动参数可选至多 10 秒。新统一启动器成对配置 controller/gateway 的 `0.2 m/s` 指令上限，odom 线速度异常阈值为 `0.3 m/s`；
原始无参数 controller/gateway 入口和统一入口的 `0.05` 回退仍为 `0.05 m/s` 指令、`0.12 m/s` odom 阈值。
角速度上限 `0.1 rad/s`、转弯半径至少 `1 m`、线加速度上限 `0.025 m/s²` 及全部新鲜度/急停/watchdog 保护保留。
从零达到 `0.2 m/s` 至少约需 8 秒，一次 5 秒定时达不到该上限；限幅不保证真实车速与指令完全相同。
连续推理/RViz 在 PC 使用 `.\docker\ros2\run_robot_live.ps1 -Seconds 0`；默认仍为 600 秒，PC 会话时长与机器人运动时长独立。
PC 无截止入口不自动 arm，资源/输入故障仍会停止；输出卷可用空间低于 2 GiB 时停止或拒绝推理，日志仍会持续增长。
连续挡位需要全程现场看护，不代表可靠避障、无人值守运行或已通过长期实车验收。

部署/启动顺序、停止方式、物理急停要求和未验收边界见 [真机实时控制说明](docs/robot_live_control.md)。
**本轮已部署：325 项回归、0.2 m/s 控制链隔离测试和统一 launch 的两个硬件替身场景通过；实际 `-SyncOnly` 校时误差测量上界约 32.8 ms，退出后原 NTP 服务恢复。尚未进行 0.2 m/s 实车验收。** 详见[本轮验收记录](../outputs/ros2_robot_operator/统一启动校时与限速验收_20260910-143230.md)。

后续 **16:19 退出补丁**：修复 `ros2 run` 外层退出后的底盘/TF 残留、独立 SSH 权限检查及 `closed` 回传；372 项回归和 PC/ARM64 无硬件进程清理测试通过。经授权清理旧会话并恢复 NTP，启动命令与 0.2 m/s 配置不变，本补丁未启动硬件或解锁。见[修复记录](../outputs/ros2_robot_operator/recovery__20260910T080700Z/退出残留修复_20260910-161900.md)。
2026-09-10 此前的按次/连续服务已部署：181 项回归、PC/ARM64 实际服务隔离测试通过，当时使用旧 `0.05 m/s` 配置、本日尚未试动。
[此前软件验收记录](../outputs/ros2_robot_arm_modes_tests/验收摘要_20260910-132427.md)。当时机器人时钟为 2000 年、底盘未启动；每次启动均须校时及完成启动前检查。
用户已确认实时 RViz 轨迹出现、`base_link` 报错消失，并确认首次 3 秒试动“缓慢前进并已停住”。
该次指令峰值 `v=0.05 m/s、w=0.05 rad/s`，里程计反馈峰值约 `0.064 m/s、0.083 rad/s`，存在超调；
末尾命令/里程计均为零，额外 disarm 成功。实际速度/转向标定、制动距离、断联实车及固件 watchdog 仍待验收。
本入口使用新的 PC 双 loopback/Wi-Fi 配置及机器人专用发现配置，旧仅推理配置保持不变；
不应同时运行下方旧推理或视频入口以占用相同固定 DDS 端口。

## 真机相机推理入口（2026-09-09）

真实小车的压缩彩色图像已接入电脑 CUDA NoMaD，输出限定在 `/nomad/robot_preview/...`，不启动控制器。
使用独立 GPU 容器 `nomad-robot-inference`；Windows 启动：

```powershell
.\docker\ros2\run_robot_inference.ps1 -Seconds 60
```

需要先关闭共用固定 DDS 端口的相机 RViz/测试。容器、构建、原始时间戳保护、独立归档与验收边界见
[真机推理说明](docs/robot_inference.md)。以下旧入口仍用于原视频/本机预览，不要直接照搬为真机运动流程。

## RViz 联合预览入口

视频、NoMaD、控制预览和 RViz 图像/轨迹显示已打通，用户截图确认 `Global Status: Ok`。
Windows 启动、重播、停止、图例和 DDS 排错见 [RViz 使用说明](docs/rviz_preview.md)。
`rviz_demo.launch.py` 联合启动核心节点，等待模型加载完成才开始播放视频。
目前只是局部决策显示，不是机器人在录像场景中的闭环仿真。
[30 分钟稳定性验证](docs/stability_validation.md)已通过本机预览门槛：1800.054 秒、计时区间 6460 组 waypoint，
0 次检测到的 ROS 时间回拨，测试进程清理正常。一次观测不连续触发保护并约 1.3 秒恢复，待继续诊断。
这不是 GUI 持续渲染、无间断相机或真机验收。

## 速度预览入口

```bash
ros2 launch nomad_navigation controller.launch.py project_root:=/workspace/SocialEmbodiedAgent
```

先加载下文 ROS/工作空间环境，再按推理说明启动模型和视频。控制器默认 20 Hz，
上限 0.2 m/s、0.4 rad/s；没有有效输入时持续归零。仅实现 dry-run，拒绝真实运动话题重映射。
启动、停止、急停复位、唯一日志目录与验收限制见 [控制器说明](docs/controller_preview.md)。
此前系统 ROS 时间出现回拨；统一校时并重启 WSL/Docker 后，默认真实时钟的短时 GPU 链路复验已通过。
这不是长期稳定性或真机验收，回拨保护仍然保留。
最终回归 65 项单元测试全部通过；本次控制器联调收到 12 组真实 waypoint 和 473 条预览消息。

## NoMaD 推理入口

构建并加载环境后，先启动推理节点：

```bash
ros2 launch nomad_navigation nomad.launch.py project_root:=/workspace/SocialEmbodiedAgent
```

看到 `NoMaD ready` 后，再从另一个加载相同 ROS 环境的终端启动下文的视频节点。
第三个终端可执行 `ros2 topic echo /waypoint --once --no-daemon` 查看二维点。

33 项单元测试和真实 GPU 集成测试已通过，收到 26 组 waypoint，平均推理约 120 ms。
完整启动、参数、尺度、超时和运行记录说明见 [NoMaD 推理说明](docs/nomad_inference.md)。

2026-09-05 已在 `ros2-dev` 构建通过：7 项单元测试全部通过，`ros2 run` 和
launch 两种启动方式均通过节点发现、参数读取、心跳与正常退出验证。
[本次验证报告](../outputs/ros2_package_checks/nomad_navigation__20260905T065806.600973Z/report.json)
同目录保留单元测试 XML、构建日志和两种启动方式的日志。

## 构建（在 ros2-dev 容器内）

```bash
source /opt/ros/humble/setup.bash
cd /workspace/SocialEmbodiedAgent/ros2_ws
colcon build --packages-up-to nomad_navigation
source install/setup.bash
```

`src/` 是源码；`build/`、`install/`、`log/` 是生成目录，不应当作为跨平台交付源码。
构建应在容器 Linux 中执行，不使用 Windows/Conda Python。
`--packages-up-to` 会先构建 `nomad_interfaces`，不能在干净工作空间只选 `nomad_navigation` 而遗漏生成的服务依赖。
机器人接口另在本机 ARM64 的 `/home/wheeltec/nomad_interfaces_ws` 构建；新终端加载
`/home/wheeltec/nomad_live_control/setup.bash`，不要将 PC 的 x86_64 接口生成库复制过去使用。

当前 F 盘 Docker 挂载创建符号链接时报 `Operation not permitted`，因此这里使用普通
`colcon build`，不加 `--symlink-install`。修改源码、配置或启动文件后须重新构建。
迁移到支持 Linux 符号链接的文件系统后，可在新的构建目录中使用 `--symlink-install`。

## 最小节点运行与停止

任选一种启动方式，不必同时运行：

```bash
ros2 run nomad_navigation smoke_node
```

或加载包内默认参数文件：

```bash
ros2 launch nomad_navigation smoke.launch.py
```

在另一个已加载 Humble 和本工作空间的容器终端检查：

```bash
ros2 node list
ros2 node info /nomad_smoke
ros2 param get /nomad_smoke heartbeat_period_sec
```

应看到 `/nomad_smoke`，默认每 2 秒输出一次心跳。按 `Ctrl+C` 正常停止。
节点只有 ROS 自带的日志、参数相关接口，没有业务运动话题。

启动时覆盖参数示例（启动后这些参数只读）：

```bash
ros2 run nomad_navigation smoke_node --ros-args -p heartbeat_period_sec:=1.0
ros2 launch nomad_navigation smoke.launch.py namespace:=demo
```

自定义参数文件使用 `params_file:=/your/path/smoke.yaml`；包内默认配置通过
ROS 包索引定位，没有写死本机项目路径。

## 视频图像发布

构建并加载工作空间后，在容器中运行：

```bash
ros2 launch nomad_navigation video.launch.py \
  video_path:=/workspace/SocialEmbodiedAgent/offline_data/3-r.mp4
```

默认以 4 Hz 发布 `/camera/image_raw`（`sensor_msgs/msg/Image`），编码 `bgr8`，
将本视频等比例缩放为 320×180。视频约 60 秒，默认播放一次，结束后节点和 launch
自动退出，日志显示 `Playback complete`；也可以随时按 `Ctrl+C` 停止。

在另一个容器终端中加载环境后检查（无需停止发布端）：

```bash
source /opt/ros/humble/setup.bash
source /workspace/SocialEmbodiedAgent/ros2_ws/install/setup.bash
ros2 topic hz /camera/image_raw
```

按 `Ctrl+C` 停止帧率检查，再查看一条 header，避免打印整幅图像字节：

```bash
ros2 topic echo /camera/image_raw sensor_msgs/msg/Image \
  --field header --once --qos-reliability best_effort --no-daemon
```

参数均在启动时设置，运行中只读：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `video_path` | 必填 | 容器内可访问的本地文件路径 |
| `image_topic` | `/camera/image_raw` | 可改绝对话题或相对话题；绝对话题不随 namespace 改变 |
| `publish_fps` | `4.0` | 发布频率，须大于 0 且不超过原视频 FPS |
| `loop` | `false` | 是否循环选定的视频区间 |
| `start_sec` | `0.0` | 视频起点，取不早于该时间的首帧 |
| `end_sec` | `-1.0` | 排他终点；-1 表示视频末尾 |
| `resize_width` | `320` | 输出宽度，保持纵横比；0 保留原分辨率 |
| `frame_id` | `camera_optical_frame` | 模拟相机光学坐标系名称，不自动发布 TF |

直接运行入口可以覆盖任意节点参数，例如循环播放前 10 秒：

```bash
ros2 run nomad_navigation video_publisher --ros-args \
  -p video_path:=/workspace/SocialEmbodiedAgent/offline_data/3-r.mp4 \
  -p loop:=true -p start_sec:=0.0 -p end_sec:=10.0 -p publish_fps:=4.0
```

launch 使用包内 `config/video.yaml`，也支持 `params_file:=/your/path/video.yaml`；
`video_path` 仍由 launch 参数传入。修改包内 YAML 后需重新构建，或使用外部参数文件。

### 图像语义与限制

- 当前按恒定源 FPS 的视频时间轴抽帧，不是将 30 FPS 视频逐帧按 4 FPS 慢放。
  若解码或传输跟不上，实际帧率可能低于设定值；变帧率视频需先转为恒定帧率。
- header 使用当前 ROS 时间模拟实时采集，不是原始录像的绝对时间；未启用模拟时钟。
- 使用 sensor-data QoS：best effort、volatile、depth 5。订阅端必须使用兼容 QoS。
  640×360 在当前容器的实测中出现明显丢帧，所以默认降为 320×180；
  原因尚未进一步定位，没有修改网络或 DDS 配置。较高分辨率需重新测量接收帧率。
- 循环时内容会跳回起点，ROS 时间仍向前。后续模型接入时需要处理历史观测跨循环跳变，
  因此默认不循环。此节点不伪造相机标定或发布 `CameraInfo`、TF。
- RViz2 Image 选择该话题并将 Reliability 设为 Best Effort；
  后续联合预览已由用户实际窗口截图确认显示正常，见上文 RViz 说明。

依据：[Humble QoS 兼容性](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)、
[Image 消息语义](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Image.html)。

### 本次视频验证（2026-09-05）

20 项单元测试全部通过。实际视频 2–8 秒片段发布并收到 24/24 帧，
`ros2 topic hz` 为 3.992–4.009 Hz；接收图像与原视频独立解码、缩放后的像素完全一致。
另一测试收到 12 帧并跨过两次循环边界，正常停止后无继续发布。

- [视频验证报告](../outputs/ros2_video_checks/3-r__video-input__20260905T073817.107400Z/report.json)
- [订阅端实际收到的图像](../outputs/ros2_video_checks/3-r__video-input__20260905T073817.107400Z/finite_launch_received.png)

报告目录同时保存参数、header、帧率、逐帧统计和日志，不覆盖历史测试。
可重新执行实际视频集成测试（独立本地 ROS 域）：

```bash
ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 python3 scripts/verify_video.py \
  /workspace/SocialEmbodiedAgent/offline_data/3-r.mp4
```

该脚本针对本项目 `3-r.mp4` 的尺寸及至少 8 秒时长进行验收，不是通用视频检查器。

## 测试

```bash
colcon test --packages-select nomad_navigation --event-handlers console_direct+
colcon test-result --verbose
```

84 项测试覆盖参数、心跳、视频图像、模型推理接口，以及控制限幅、时间戳、超时、
软件急停、预览话题约束、暂停 ROS 时钟时的 watchdog、可视化配对及 Humble RViz 配置。
集成验证脚本 `scripts/verify_package.py` 需在构建并加载工作空间之后运行：

```bash
ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 python3 scripts/verify_package.py
```

脚本检查包和资源发现、`ros2 run`、launch、ROS 图中的节点、参数和退出清理；
报告写入项目的 `outputs/ros2_package_checks/` 独立 UTC 时间戳目录。
测试使用独立 ROS domain，不需要连接机器人。

## 可移植性与后续工作

迁移时同时复制 `src/nomad_navigation` 与 `src/nomad_interfaces` 到目标 ROS 2 工作空间，安装其 `package.xml`
声明的依赖并重新构建；不要直接复制基于旧路径的 symlink-install 产物。
视频/最小节点仅需 ROS 2 和图像依赖；NoMaD 节点还需要 PyTorch、diffusers、
`vint_train`、`diffusion_policy` 等既有模型环境，见 [依赖安装说明](../docker/ros2/README.md)。

包元数据的维护者邮箱是占位值，许可证尚未选择；对外发布前须由项目所有者确认。
视频输入、无目标推理、预览控制器、RViz 显示、联合启动及本机 30 分钟预览测试已完成。
更长时钟稳定性、连续相机观测抖动诊断、rosbag2、拓扑目标导航，以及进一步真机标定/制动距离/断联与硬件保护验证仍待完成。
预览控制器不是机器人安全控制系统。
