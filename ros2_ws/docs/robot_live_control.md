# 真机实时推理、RViz 与按次定时 / 连续控制

更新：2026-09-10。本入口把电脑推理和机器人本地控制分开；启动电脑或恢复网络都不会自动 arm。
目前已接通真机图像与 CUDA 推理，并完成首次 **单次 3 秒使能、缓慢前进并停止** 的实车试动，用户已现场确认。
这是短时功能验证，不代表长期导航、安全避障、实际制动距离或固件 watchdog 已验收，不能无人值守运行。

当前用户已确认 `V550_akm`，与小车现有软件配置一致。不要把旧视频流程的 `waypoint_controller` 或 `/cmd_vel_preview` 接到真实底盘。

本日先完成 `/arm_for` 按次定时和连续挡位；连续只取消运动时长截止，不取消任何停车保护，也不是无人值守模式。
此前接口阶段 **181 项回归与 PC/机器人 ARM64 隔离服务测试通过并完成部署，当时未试动**；这些是旧 `0.05 m/s` 配置的证据。
随后进一步封装“校时 + 机器人节点组 + 解锁菜单”，统一入口的线速度指令上限按用户要求改为 `0.2 m/s`，保留 `0.05 m/s` 回退。
**本轮已部署，325 项回归、0.2 m/s 控制链隔离测试及统一 launch 的两个硬件替身场景通过；实际 `-SyncOnly` 校时通过，未进行 0.2 m/s 实车验收。** 校时后 PC/机器人测量误差上界约 32.8 ms，原 NTP 服务已恢复。重连时曾发现机器人重启后时间再次回到 2000 年；
每次都应正确校时并重新核对输入/零速度链路，前一天的运行状态不代表今天节点仍在运行。

**2026-09-10 16:19 退出修复补丁已部署。** 用户后续运行暴露了 `ros2 run` 外层退出、实际底盘/TF 子节点残留，以及新 SSH 登录进程的权限检查阻断收尾的问题。经授权核验后，旧 PID 21557/21558 已正常结束，原 NTP 服务已恢复；372 项回归、PC/ARM64 无硬件孤儿进程测试和真实 SSH 拒绝请求回传通过。底盘与静态 TF 改为直接执行 ROS libexec，退出跟踪核验整个自有进程树，保留全部运动保护。此补丁未重新启动硬件或解锁。见[退出残留修复记录](../../outputs/ros2_robot_operator/recovery__20260910T080700Z/退出残留修复_20260910-161900.md)。

## 架构与需要启动的容器

电脑只需要一个 `nomad-robot-runtime` 容器，镜像为本地已保存的
`socialembodiedagent/nomad-ros2-runtime:robot-preview-20260909`。机器人本身不需要 Docker 或 GPU。

```text
机器人相机 /camera/color/image_raw/compressed
  → PC 容器：NoMaD + 审计 + Marker（同一 Context，participant 10）
      ├→ 私有 waypoint/status → 机器人 robot_control（独立进程，默认 disarmed）
      │                          → desired_cmd → robot_gateway（独立进程）
      │                                           → /cmd_vel → 厂商底盘驱动
      └→ 图像/Marker → 同容器 RViz（独立进程，participant 11）
机器人 /odom、相机 → robot_control 的本地新鲜度/运动状态检查
```

PC 不创建真实速度发布者、不调用 arm。机器人只有 `robot_gateway` 向 `/cmd_vel` 发布；
`robot_control` 生成带时间戳的 `/nomad/robot_control/desired_cmd`，其 `cmd_vel_preview` 仅用于显示。
独立网关可以在 controller 进程停止更新而网关仍健康时清零；不能防住整台机器人系统停摆。

旧 `ros2-dev`、`nomad-robot-inference` 保留且不运行推理；旧实验容器也不需要启动。
**任一时刻只能有一个 PC 模型进程**。不能并行运行旧相机 RViz、推理测试、camera_check 或带相同 XML 的 ROS CLI/daemon。

| PC 进程 | DDS participant | 固定 UDP 端口 | XML |
| --- | --- | --- | --- |
| NoMaD + 审计 + Marker | 10 | 7430 / 7431 | `docker/ros2/robot_live_udp.xml` |
| RViz | 11 | 7432 / 7433 | `docker/ros2/robot_rviz_udp.xml` |

实时模式的两个 PC profile 均明确通告 `127.0.0.1` 和 Windows Wi-Fi `192.168.0.180`：
前者用于同一个 Docker host 上的模型与 RViz 互通，后者用于机器人 `192.168.0.100` 的跨机通信。
本机两个 participant 通过各自的 loopback 初始发现端口互相发现，避免依赖 Docker Desktop 对 Wi-Fi 地址的回流路径。
这不是重新引入 `192.168.65.*` 内部通告；旧仅推理用的 `robot_wifi_only.xml` 没有改变。

需要 WSL mirrored 与 Docker Desktop host networking。PC 的 XML 不应复制给机器人使用。
机器人另用 `docker/ros2/robot_onboard_udp.xml`，部署到 `/home/wheeltec/nomad_live_control/robot_onboard_udp.xml`：

- 机器人 participant ID/端口保持自动分配；多个本地进程不能共用 PC 的固定 ID 10/11。
- initial peers 包括 PC `192.168.0.180:7430`、`:7432` 以及机器人本机 `192.168.0.100` / `127.0.0.1`。
- `maxInitialPeersRange=32` 扩大本机初始发现扫描范围，解决新 controller/gateway/TF/CLI participant 互相发现不到、只能看到旧相机/底盘进程的问题。
- 机器人白名单使用真实本机 Wi-Fi/loopback 地址；这与 Docker host 上不能绑定 Windows Wi-Fi 地址的情况不同。

地址变化需要重新验证双方 XML 和脚本内的地址约束。更改 XML 只影响之后启动的进程，不会自动重配已运行节点。

## 每次启动前

1. 电脑连接机器人热点。确认 `192.168.0.180` 仍是电脑 Wi-Fi 地址，SSH 可连接 `wheeltec@192.168.0.100`。
2. 分别检查电脑和机器人时间。机器人曾在重启后回到 2000 年；RTC 保持和长期自动校时仍未完成验证。
   时间不对先停止相关节点并正确校时，不重打相机时间戳、不扩大允许超时来绕过保护。
3. 保持物理急停可触及，清空试验区域，确认车辆静止。底层固件 watchdog 尚未验证。
4. 先检查现有节点，避免重复启动相机、厂商底盘驱动、controller 或 gateway。不要同时启动 Nav2、键盘/手柄遥控等其他速度源。
5. 首次启动只验收图像、点、状态和零速度；下面的 arm 步骤是单独的受控运动阶段。

## 日常入口：两个常驻窗口

多个节点仍是独立进程，但不再需要逐个打开终端。先在原终端解除使能、确认车辆静止，正常停止旧的 PC 推理/RViz 和机器人节点；
新入口不会强杀已有进程，也不会接管旧会话。检查到已有相关 ROS/硬件进程、其他 operator 锁或 PC 固定 DDS 端口被占用时会拒绝校时。
不要同时使用本节入口和后文的逐进程排错命令。

### 窗口 1：校时、启动小车节点组和控制菜单

Windows PowerShell：

```powershell
cd F:\SocialEmbodiedAgent
.\docker\ros2\start_robot.ps1
```

默认等价于 `-MaxLinearMps 0.2`。脚本检查 PC 热点地址和专用容器，必要时只启动空闲的 `nomad-robot-runtime` 容器；
随后通过系统 OpenSSH 登录机器人。SSH 密码按提示输入，建立连接后再单独输入 sudo 密码；二者不写入配置或运行日志。
请保持这个菜单窗口打开；它不是运行一次后即可关闭的后台服务安装器。

校时通过后，机器人端的统一 launch 先启动默认未解锁的 controller 与独立 gateway、核对零命令链路，
再启动厂商最小底盘驱动、相机和 `base_footprint → base_link` 静态 TF。启动不会自动 arm，也不启动 Nav2 或其他遥控速度源。
底盘与静态 TF 通过 ROS 包索引定位并直接启动实际程序，不再套用 `ros2 run` 外层；启动前同时检查程序存在及 Linux pidfd 进程身份保护能力。
如只需完成一次校时检查，不启动相机、底盘、控制器或网关：

```powershell
.\docker\ros2\start_robot.ps1 -SyncOnly
```

回退到原低速指令上限：

```powershell
.\docker\ros2\start_robot.ps1 -MaxLinearMps 0.05
```

速度挡只接受 `0.05` 或 `0.2`，在启动整组时选择，不在运动中切换。回退或更换配置前先停止、确认静止并结束原会话。

### 校时的范围和退出恢复

- 以 Windows 系统 UTC 为基准，在 SSH 认证完成后使用持续连接采样；同时检查 Windows 与 Docker 的时间差。
  时间差和往返延迟的不确定性合计必须满足 `≤ 0.1 s`，不满足时拒绝启动机器人节点。不修改 Windows 的 NTP 设置。
- 校时模块仅允许运行于真实拥有 `192.168.0.100` 的 ARM64 机器人、非 root 操作者进程；整段会话持有机器人 operator 锁。
- 记录 `ntp.service`、`ntpsec.service`、`chrony.service`、`systemd-timesyncd.service` 的原状态，只暂时停止原先活动的校时服务；
  不执行 disable、不改配置，发现无法归属的时钟守护进程则拒绝继续。不能仅凭 `timedatectl` 的 `NTP=no` 认定没有独立 ntpd。
- 用 Unix epoch 校时，保留现有 `Asia/Shanghai` 时区；不写 RTC，不声称本次校时解决了机器人关机后的时钟保持。
- 节点运行期间不再改时钟，也不恢复可能重新调整时间的服务。正常结束会话时，先停止本会话 ROS 节点并检查无相关进程，
  再恢复本会话原先活动且被暂停的服务；`-SyncOnly` 则在校时完成后恢复并退出。
- 不能确认进程已结束、服务恢复失败或显示 `pending_services`/`cleanup_failed`/`cleanup_uncertain` 时，必须保留记录并人工核对，
  不要忽略错误或当成已经安全收尾。软件不能保证强关窗口、断网或整机故障之后一定完成 NTP 恢复。

### 窗口 2：电脑推理和实时 RViz

等窗口 1 完成校时并进入菜单后，在另一个 PowerShell 中运行：

```powershell
cd F:\SocialEmbodiedAgent
.\docker\ros2\run_robot_live.ps1 -Seconds 0
```

这仍只使用 `nomad-robot-runtime` 容器。`-Seconds 0` 取消 PC 会话截止，不会解锁车辆；推理、输入、DDS 和磁盘保护仍保留。
先确认 RViz 有新鲜相机画面和蓝绿轨迹/黄色 waypoint，再回到窗口 1 查看状态及操作。

### 窗口 1 的菜单

| 输入 | 作用 |
| --- | --- |
| `i` | 查看 controller/gateway 状态；不能仅凭过期状态认定车辆已静止 |
| `t` | 输入本次 `(0, 10]` 秒，随后输入 `RUN` 确认，申请定时解锁 |
| `c` | 随后输入 `RUN` 确认，申请无时长截止的连续解锁 |
| `s` | 请求 disarm；保留节点和相机，现场确认车辆停止 |
| `q` | 请求 disarm，结束本会话机器人节点组，再恢复原校时服务并退出菜单 |

每次 `t`/`c` 都要求重新确认已在现场、区域清空、物理急停有效、车辆静止且图像/轨迹正常，并明确输入大写 `RUN`；
没有自动 arm、自动续期或故障后自动重连解锁。正常收尾先 `s` 并确认静止，停止窗口 2 的推理/RViz，再在窗口 1 输入 `q`。
`s` 不结束校时会话；`q` 才结束整组。需要再次使用时重新走校时和启动检查。

控制台每秒发送心跳；机器人端超过 **4 秒**未收到控制台联系时，会请求解除使能并结束自己启动的节点组。
这只是额外的软件失联处理，不是 4 秒内实际制动保证，也不能替代本地输入/watchdog 保护或物理急停。
任何失联或停止响应不明确时立即现场确认，必要时使用物理急停；不要靠关闭窗口来执行急停。

若再次出现 `Stop existing ROS/hardware/control processes ... PIDs`，代表校时仍被既存进程保护阻止；不要绕过，也不要直接照抄旧 PID 执行 kill。
应先现场停稳，再核对当前进程身份、所属会话与出生时间。退出是否成功以收到的结构化清理结果为准：SSH 连接结束本身不算成功证明；无法确认时仍会明确报错。原先 `prepare` 已自行回滚的结果现在会正确保留，不再无故追加恢复失败。

## 机器人端：逐进程排错入口（不是日常启动要求）

代码部署目标为 `/home/wheeltec/nomad_live_control/nomad_navigation/`，包括
`robot_control_node.py`、`robot_control_core.py`、`controller_core.py`、`robot_gateway.py` 及包初始化文件。
这部分使用系统 Python/机器人现有 ROS 2，不安装 PyTorch，不加载 NoMaD 权重。

新增服务由 `ros2_ws/src/nomad_interfaces` 中的 `ArmFor.srv` 定义。机器人需要在自己的 ARM64 / Humble 环境构建接口包，
目标工作空间为 `/home/wheeltec/nomad_interfaces_ws`；不要直接复制 PC 容器内 x86_64 的生成库到机器人。
部署后的统一环境脚本会加载 ROS、厂商工作空间、接口工作空间、控制模块路径及机器人专用 DDS 配置。

SSH 登录后，每个用于启动机器人的终端先加载环境：

```bash
source /home/wheeltec/nomad_live_control/setup.bash
date -Is
ros2 interface show nomad_interfaces/srv/ArmFor
```

如果找不到 `nomad_interfaces/srv/ArmFor`，先修复接口部署/环境加载，不要用更改消息类型或把参数写入 `Trigger {}` 来替代。

下面保留逐进程前台命令，供校时完成后的排错和历史流程复现，不需要在统一入口之外重复执行。
原始 controller/gateway 无参数入口仍为 `0.05 m/s`，不会继承 `start_robot.ps1` 的 `0.2 m/s` 配置；
不要只改其中一端的限速，也不要通过逐进程入口绕过校时或零速度检查。已有进程时先核对，不重复启动。

### 1. 相机

```bash
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
```

真实彩色输入为 `/camera/color/image_raw/compressed`，原始 frame 为 `camera_color_optical_frame`。
本机相机约 30 Hz；模型按约 4 Hz 取观测，不需要逐帧处理所有图像。

### 2. 默认解除使能的控制器

```bash
python3 -m nomad_navigation.robot_control_node
```

上述命令保留旧 `/arm` 接口默认单次 **3 秒**。使用新的 `/arm_for` 时，每次请求直接指定时长或连续挡位，
**不需要重启控制器或修改启动参数**，调用方式见下方“手动解锁”。

为兼容旧 `/arm`，仍可在启动时改变它的默认时长，例如 10 秒。只有确需改变旧接口默认值时，才先 disarm、确认车辆静止、
在原控制器终端按 Ctrl+C 正常退出并确认旧进程已结束，再运行；**不要并行启动第二个 controller**：

```bash
python3 -m nomad_navigation.robot_control_node --ros-args -p arm_lease_sec:=10.0
```

`arm_lease_sec` 为启动时可配置的浮点参数，允许 `0 < 值 ≤ 10.0` 秒；默认值仍为 `3.0`。
必须写 `10.0`（浮点数）。参数运行时只读，不能通过 `ros2 param set` 延长当前运行。
启动仍为 disarmed，不会立即运动；稍后单独调用旧 `/arm` 才会使用这个默认时长。
再次从无参数命令启动会恢复默认 3 秒；修改这个参数不改变限速、0.6 秒 waypoint 接收保护或独立网关的 0.25 秒 TTL。
`/arm_for` 的定时上限独立保持 10 秒，不因旧接口默认值为 3 秒而缩短，也不把 `arm_lease_sec=0` 解释为连续运行。

节点名固定为 `/nomad/robot_control`。启动后为 `disarmed`，缺模型、相机或里程计时也只生成零命令。
该进程不直接发布 `/cmd_vel`。它对私有模型输出只订阅 `waypoint_stamped` 和 `status`，与 PC 的订阅者白名单一致。

### 3. 独立速度网关

```bash
python3 -m nomad_navigation.robot_gateway
```

节点名为 `/nomad/robot_gateway`。先确认它处于零命令状态，再启动尚未运行的底盘驱动。
网关在 domain 0 仅允许运行于 `aarch64` 且本机拥有 `192.168.0.100` 的机器人；不要在 PC 容器启动它。
domain 86 + `ROS_LOCALHOST_ONLY=1` 仅用于隔离测试，不是真机运行参数。

### 4. 厂商最小底盘驱动

不要用可能同时启动导航/遥控等额外组件的综合 launch 来替代这个已核对的最小入口：

```bash
ros2 run turn_on_wheeltec_robot wheeltec_robot_node --ros-args -p car_mode:=V550_akm
```

2026-09-09 已确认串口打开、`/odom` 有数据，静止时实测 `v=0、w=0`；`/odom` 的父/子 frame 为 `odom` / `base_footprint`。
当日完成一次短时前进/停止试动，结果见文末；这不是今日底盘已启动的证明。
转向标定、制动距离、断联实车测试和固件超时保护仍未完成。

### 5. 厂商车体静态 TF

最小底盘进程之外，还需要单独补上已核对的厂商 `base_footprint → base_link` 静态关系。
在加载同一机器人环境的独立终端运行；若检查发现现有 TF 发布者仍在，不要重复启动：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.01826 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_footprint --child-frame-id base_link
```

这里的 0.01826 m 取自已核对的厂商车体配置，不是用虚构 TF 掩盖未收到 Marker 的网络问题；
相机外参/模型实际尺度依然需要独立标定。完整连接中要同时具备机器人 TF 和 PC 本机 participant 发现。

另开检查终端，确认 `/cmd_vel` 只有 gateway 一个发布者：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /nomad/robot_control/status --once
ros2 topic echo /nomad/robot_gateway/status --once
ros2 topic echo /odom --once
```

发现额外 `/cmd_vel` 或 desired_cmd 发布者时先排除冲突；不要靠多个进程同时写零值“争抢停车”。

## 电脑端：一条命令启动推理与 RViz

在 Windows PowerShell 的项目根目录执行：

```powershell
cd F:\SocialEmbodiedAgent
.\docker\ros2\run_robot_live.ps1 -Seconds 600
```

启动器按需创建专用 `nomad-robot-runtime`，并核对镜像、host 网络、全部 GPU、8 GiB shm、项目读写挂载和 WSLg 显示套接字。
源码构建到容器内 `/opt/nomad_robot_ws`；不覆盖旧 `ros2_ws/install`，不删除或停止其他容器。
若同名容器配置不匹配或固定端口被占用，脚本拒绝继续。

默认运行 600 秒，有限时长范围 10–3600 秒，从 CUDA 模型就绪后开始计时。需要持续推理/RViz 时，显式使用 `0` 取消 PC 会话截止：

```powershell
.\docker\ros2\run_robot_live.ps1 -Seconds 0
```

**PC 的 `-Seconds` 与机器人运动时长是两套独立设置。** `-Seconds 0` 不会解锁机器人；机器人连续挡位也不会阻止一个
`-Seconds 600` 的 PC 会话在 600 秒结束。PC 停止出点后，机器人仍会因输入过期解除使能，因此希望持续预览时需同时选择 PC 无截止入口。
这不是必须运行到手动停止的承诺：PC 保留输入、推理、DDS、RViz 和资源故障检查，输出所在卷可用空间低于 **2 GiB** 时拒绝或结束推理。
图像/轨迹归档仍持续增长，应主动检查磁盘和日志；目前没有长期实车或无限时长稳定性验收。

初始至少收到 3 张不同的新鲜相机图像才加载模型。
模型/审计/Marker 共用一个进程，RViz 是同容器的另一个进程；不要另开第二个 RViz 或模型入口。
RViz 显示的是机器人坐标系下的局部预测和命令文本，不是已行驶轨迹；当前没有把里程计积分成 RViz 中的运动仿真。
状态文字增加 `TIMED x.x s remaining`、`CONTINUOUS`、`DISARMED`；控制状态过期、非法或回拨时显示 `UNKNOWN`，不表示已经确认停车。

实际运行入口是 `ros2_ws/scripts/run_robot_live.py`；需要在已构建、已加载 Humble/工作区、已设置显示环境的专用容器内启动时：

```bash
source /opt/ros/humble/setup.bash
source /opt/nomad_robot_ws/install/local_setup.bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/SocialEmbodiedAgent/docker/ros2/robot_live_udp.xml \
python3 /workspace/SocialEmbodiedAgent/ros2_ws/scripts/run_robot_live.py --seconds 600
```

容器入口同样使用 `--seconds 0` 取消会话截止。PC 工作空间更新时使用
`colcon build --base-paths /workspace/SocialEmbodiedAgent/ros2_ws/src --packages-up-to nomad_navigation`，
以同时构建依赖的 `nomad_interfaces`，然后重新加载 `/opt/nomad_robot_ws/install/local_setup.bash`。

## 手动解锁：每次选择定时或连续挡位

**这一节不是默认启动步骤。** 每次均须由现场操作者确认物理急停可用、区域清空、零速度链路正常及车辆静止；
先观察 RViz 图像/轨迹和模型状态，再在已加载统一环境的机器人终端选择下面一种解锁方式。连续挡位必须始终有人看护。

| 接口/挡位 | 请求 | 截止方式 |
| --- | --- | --- |
| `/arm_for` 定时 | `mode: 0`，有限 `0 < duration_sec ≤ 10` | 到期自动解除使能；故障可提前停车 |
| `/arm_for` 连续 | `mode: 1`，必须 `duration_sec: 0.0` | 无运行时长截止；手动 disarm 或任一保护条件解除使能 |
| 兼容旧 `/arm` | `Trigger {}` | 使用启动参数 `arm_lease_sec`，默认 3 秒、最高 10 秒 |

例如，本次运行最多 5 秒：

```bash
source /home/wheeltec/nomad_live_control/setup.bash
ros2 service call /nomad/robot_control/arm_for \
  nomad_interfaces/srv/ArmFor '{mode: 0, duration_sec: 5.0}'
```

明确选择连续挡位：

```bash
ros2 service call /nomad/robot_control/arm_for \
  nomad_interfaces/srv/ArmFor '{mode: 1, duration_sec: 0.0}'
```

`mode: 0, duration_sec: 0.0` 是非法定时请求，**不会**解释为无限运行；连续挡位携带非零时长、未知 mode、负值、NaN 或 Infinity 都拒绝。
服务返回 `success: true` 才表示此次解锁被接受，不代表所有后续输入或车辆运动都已通过验收。

继续使用原来的有限时长默认值时：

```bash
ros2 service call /nomad/robot_control/arm std_srvs/srv/Trigger '{}'
```

输入不满足新鲜度、模型状态或静止条件时，任一解锁接口都会拒绝。**正在运行时不允许续期、重新 arm 或切换挡位**；
需要先 disarm、确认车辆完全停止，再重新发起请求。不要用循环 arm 来绕过这一规则。
10 秒是定时挡位的最大值，不保证一定能运动满 10 秒；连续挡位同样会因断图、点间隔过长、里程计异常、急停或 watchdog 等保护提前停车。
到期、故障、断流或 disarm 后都需要新的显式 arm，不会因相机恢复、PC 重连或重启而自动恢复运动。

两种挡位的软件停止命令相同；停止后现场确认车辆静止，异常时立即使用物理急停：

```bash
ros2 service call /nomad/robot_control/disarm std_srvs/srv/Trigger '{}'
```

网关故障可能锁存。应先 disarm、确认车辆静止、排除故障且确认正在收到新鲜零命令，再考虑网关显式复位：

```bash
ros2 service call /nomad/robot_gateway/reset std_srvs/srv/Trigger '{}'
```

复位不是 arm；时钟回拨等故障要求先处理时钟并重启相应进程，不能靠反复复位消除。

## 软件保护与不能承诺的部分

统一入口的**软件指令**限制如下。`0.2 m/s` 是允许上限，不是保证每个 waypoint 都达到的巡航速度；
原始无参数 controller/gateway 入口以及显式 `-MaxLinearMps 0.05` 保持低速回退。

| 项目 | 上限/约束 |
| --- | --- |
| 前进线速度 | 新统一入口 `0 ≤ v ≤ 0.2 m/s`；低速回退 `0 ≤ v ≤ 0.05 m/s`，controller/gateway 成对配置 |
| 角速度 | `|w| ≤ 0.10 rad/s`，同时满足 `|w| ≤ v / 1 m` |
| 转弯半径 | 至少 1 m；厂商 Nav2 配置为 0.35 m，本入口故意采用更保守值 |
| 加速 | 线加速度不超过 `0.025 m/s²` |
| odom 线速度异常阈值 | `0.2` 配置使用 `0.3 m/s`；`0.05` 回退仍为 `0.12 m/s`，超过阈值解除使能 |
| 倒车/原地转向 | 不允许；不满足前向目标条件时清零并解除使能 |
| 手动使能 | 按次定时 `(0, 10]` 秒或显式连续挡位；默认启动 disarmed，运行中不续期/切挡；旧 `/arm` 默认 3 秒 |
| controller | 20 Hz，本地相机/odom、源 waypoint 时间戳及单调时间检查 |
| 独立 gateway | desired_cmd 源时间戳和单调接收时间各 0.25 秒 TTL；健康调度下 20 Hz 输出零命令 |

上表的速度、加速和曲率限制约束的是发出的命令，不保证底盘实测速度或实际转弯半径严格相同。
首次试动已经观察到里程计速度反馈高于指令峰值，详见文末；需要底盘速度/转向标定，不能把指令上限写成实际车速保证。
从零加速到 `0.2 m/s` 在当前加速度上限下至少约需 **8 秒**；一次 5 秒定时运行达不到该上限，理想加速情况下至多约 `0.125 m/s`，
实际还受 waypoint、曲率、时序及保护影响。这里调整 odom 异常阈值是为了识别与新命令范围匹配的异常，不是放弃超速检查或保证实测速度。

PC 保留 1 秒输入/点时间戳保护、0.2 秒未来容差；模型就绪后有 5 秒启动宽限，之后实际 DDS 点停滞超过 2 秒即结束运行。
机器人 controller 的 waypoint 接收超时为 0.6 秒，源时间戳上限为 1 秒；odom 上限 0.5 秒，相机上限 1 秒。
网关的 0.25 秒是其本地 desired_cmd 超时，不应误称为“PC 一断网 0.25 秒内机械上必停”。实际链路还受 controller 检测、调度和底盘制动影响。

NoMaD 输入是缩放后的 `96×96` 图像；当前 waypoint 是原模型部署公式下的**名义尺度**，尚未完成实际距离和相机/车体外参标定。
图像中的障碍物不等于已经完成安全碰撞检查。当前没有人员安全认证、群组感知验证或可靠的避障/制动距离保证。

固件 watchdog 未验证。整个机器人 OS 或 gateway 卡死、硬断电、串口故障等情形下，不能声称软件会自动停车。
必须依赖已知有效的物理急停和现场操作者；不要通过故意断电或杀死整机来验证尚未知的保护。

## 正常停止、断联与日志

统一菜单入口按 `s` → 现场确认静止 → 停止 PC 预览 → `q` 收尾；必须检查节点组退出及原校时服务恢复结果。
本节其余命令保留为排错手段，不取代这一正常顺序。

先在机器人端 disarm 并确认静止，再关闭 PC RViz 或在交互启动终端按 `Ctrl+C`。
PC runner 对 SIGINT/SIGTERM、RViz 异常退出、输入或模型故障都会停止推理并回收自己启动的 RViz。
有界时间结束也会清理进程；`-Seconds 0` 没有该结束条件，需要主动停止或由故障保护退出。容器保留，可再次运行同一 PowerShell 入口。

如果交互终端失联，可从另一 PowerShell 停止这个专用容器：

```powershell
docker stop --time 10 nomad-robot-runtime
```

PC 停止或断网后，仍健康运行的机器人 controller/gateway 应触发本地过期保护；**Ctrl+C、关窗口、docker stop 都不是物理急停**。
不要仅关闭 SSH/PowerShell 窗口就假定远端进程或车辆已经停止。正常收尾在确认静止后再结束 controller、gateway 和厂商驱动。

每次 PC 运行单独归档，不覆盖旧结果：

```text
outputs/ros2_robot_live/run__UTC微秒__UUID/
  manifest.json              # 运行状态、阈值、模型/DDS计数、时延及停止原因
  runner.log / rviz.log
  dds_points.json
  model/.../manifest.json
  model/.../inference.jsonl
  model/.../events.jsonl
  model/.../windows/attempt_*.npz
```

统一入口的 Windows 控制台记录位于 `outputs/ros2_robot_operator/console__UTC微秒__UUID/report.json`；
机器人端会话目录由菜单启动返回的 `output` 字段指出，包含 `bringup.log`、ROS 日志与 `session_result.json`。
这些记录不保存密码；停止和服务恢复失败必须保留并处理，不覆盖旧会话。

`passed` 表示该次有界程序验收通过；`stopped` 表示操作者提前停止；`failed` 查看错误和保护原因。
`rviz_alive_through_bounded_run` 只证明 RViz 进程存活，不证明 GUI 已真实接收并渲染画面/轨迹。
连续会话的日志也单独归档；没有结束时间不等于已经通过长稳测试，不应让磁盘耗尽保护成为日常日志管理方式。

## 验收记录与待办

- 2026-09-10 本轮：新增 `start_robot.ps1`、校时/控制台/operator 与统一 launch，以及成对的 `0.2 m/s` 上限配置和 `0.05 m/s` 回退，已部署并核对文件摘要。325 项回归通过；真实 controller/gateway 的隔离测试达到 0.2 m/s 指令并验证手停、断流与网关锁存；真实 launch/checker 的两个硬件替身场景通过。`-SyncOnly` 在小车上实际通过，PC/机器人误差测量上界 32.8 ms，原 `ntp.service` 已恢复，正确校时后 ARM64 接口重新构建无 clock-skew 警告。收尾无硬件/控制/测试进程遗留。完整记录见[统一启动、校时与限速验收](../../outputs/ros2_robot_operator/统一启动校时与限速验收_20260910-143230.md)。未启动真实底盘/相机或解锁，也未进行新的真机 RViz 联调；0.2 m/s 实车试动、实际制动及长期连续运行仍未验收。
- 2026-09-10 早先阶段：新增接口与控制模块已部署到小车，181 项回归、PC 和 ARM64 原生实际服务隔离测试、旧 10 秒接口及独立网关回归通过；见[本日早先软件验收](../../outputs/ros2_robot_arm_modes_tests/验收摘要_20260910-132427.md)。连续模式两端均保持超过 10.4 秒后成功手停，断流恢复不自动解锁。该阶段未执行实车解锁、真机 RViz 会话或长期连续运动测试；所有测试与硬件运动隔离。当时机器人时钟为 2000 年，原生构建有 clock-skew 警告但接口调用通过；之后已校时并重新构建，见上一条最新记录。收尾无 controller/gateway/底盘/相机/测试进程遗留。
- 2026-09-09：单次租约上限扩展为 10 秒，默认仍为 3 秒；**134 项包回归测试通过**。新的 [10 秒隔离 ROS 验证](../../outputs/ros2_robot_control_tests/domain86__20260909T120427.074973Z__816dcac9/report.json)通过：200 条非零私有期望命令，到期后约 0.038 秒观察到归零；拒绝超过上限的启动值和运行时参数修改，其他安全检查保持通过。这不是真实 10 秒运动测试。
- 2026-09-09 部署时，机器人控制器用 `--ros-args -p arm_lease_sec:=10.0` 重启并保持未解锁；随后 8 秒审计收到 161 条 `/cmd_vel`，全部为零，没有调用 arm。旧模块备份到机器人 `/home/wheeltec/nomad_live_control/backups/20260909T120300Z/`。这是历史记录，不表示重启机器人后节点会自动恢复。
- 上一轮再次打开的 [只读预览](../../outputs/ros2_robot_live/run__20260909T115554.780836Z__dfbe4d82/manifest.json)在约 264.7 秒时因超过 2 秒未出点退出；模型记录过一次 `input_expired_during_recording`。未放宽保护，后续启动试车需重新运行 PC 预览入口；此异常与 10 秒时长参数的变更时间不重叠，不应把它当作已稳定跑满 10 分钟的记录。
- [10 分钟 live 联合验收](../../outputs/ros2_robot_live/run__20260909T114431.069986Z__0be3d4cf/manifest.json)：600.04 秒，含结束排空共 2149 组 DDS waypoint/候选/poses 与模型归档匹配，平均推理 157.84 ms。最大点间隔 0.746 s，可能触发 controller 的 0.6 s 接收保护；该项是推理/RViz 验收，不代表小车连续行驶 10 分钟。新增连续挡位也不扩大该超时来绕过保护。
- 真机相机 → CUDA → 私有 waypoint 已通过先前 60/180 秒测试，见 [仅推理验收](robot_inference.md)。
- PC 新 live 流程采用 `robot_live_udp.xml` + RViz 双 loopback/Wi-Fi locators；用户已确认 **RViz 轨迹出现，`base_link` 报错消失**。这是真实画面/局部轨迹的视觉确认，不等同于长时 GUI 稳定性或实际导航验收。
- 机器人最小驱动已打开串口；静止 `/odom` 为零，frame 确认 `odom` / `base_footprint`。
- 机器人专用发现 profile 和厂商 `base_footprint → base_link` 静态 TF 已接入。
- 2026-09-09 早期包测试 **131 项通过**（当日晚间 10 秒扩展后的 134 项记录见上文）。
- [controller 隔离 domain 86 测试](../../outputs/ros2_robot_control_tests/domain86__20260909T114349.846171Z__3bac2269/report.json)通过：默认 disarmed、短 lease/不续期、加速/曲率限制、输入超时/软件急停/里程计异常清零及无自动恢复；报告为 `hardware_test=false`，不创建 gateway，不触及真实底盘。
- [gateway 隔离 domain 86 测试](../../outputs/ros2_robot_gateway_tests/domain86__20260909T113634.275368Z__290458a6/report.json)通过：源命令静默到观察到零命令约 **0.2996 秒**；该报告明确 `hardware_test=false`，不是实际制动测试。
- [gateway 第二次隔离复测](../../outputs/ros2_robot_gateway_tests/domain86__20260909T114239.916015Z__b3d6abeb/report.json)通过，静默到零命令约 **0.2999 秒**。
- 首次实车非零运动和结束清零已完成，见下节；转向方向/曲率标定、实际速度/距离标定、制动距离、断联实车场景及硬件 watchdog **仍待验收**。

### 首次实车 3 秒试动（2026-09-09）

一次手动 arm 成功，用户明确反馈“缓慢前进并已停住”。审计结束时 `/cmd_vel` 和 `/odom` 的线速度、角速度均为零，
随后额外调用 disarm 也返回成功，controller/gateway 保持解除使能和零命令；未进行第二次自动或持续使能。

| 数据来源 | 峰值线速度 | 峰值角速度 |
| --- | --- | --- |
| `/cmd_vel` 实际发布命令 | 0.050 m/s | 0.050 rad/s |
| `/odom` 反馈 | 约 0.064 m/s | 约 0.083 rad/s |

里程计反馈相对命令峰值存在明显超调。软件按限定范围发出了指令，但这不等于真实车速被精确限制在 0.05 m/s，
也不等于物理转弯半径已经满足名义 1 m；后续应单独标定和复测。

- 30 秒审计记录收到 553 条 `/cmd_vel`、600 条 `/odom`，其中 **60 条非零命令**。
- 首条至末条非零命令跨度约 **2.9483 秒**；首条非零命令至随后首条零命令约 **2.9974 秒**。
  这是软件消息时间关系，不是从发出 stop 到机械完全停止的制动时间测量。
- 整个 30 秒记录的里程计净变化为 `Δx ≈ 0.0465582 m`、`Δy ≈ -0.00067768 m`；
  只是 odom 坐标中的净位移估计，不是独立标定的实际行驶距离或累计路程。
- 用户现场确认车辆自行停住；额外 disarm 成功。此次成功不替代断网/进程失效的实车验证，更不证明整机故障下必停。

原始证据：
[审计摘要](../../outputs/ros2_robot_control_hardware/nomad-control-20260909-lktrZy/audit__20260909T114933.175515Z__fe0999f4/summary.json)、
[逐条命令与里程计记录](../../outputs/ros2_robot_control_hardware/nomad-control-20260909-lktrZy/audit__20260909T114933.175515Z__fe0999f4/events.jsonl)、
[首次 arm 返回](../../outputs/ros2_robot_control_hardware/nomad-control-20260909-lktrZy/first-arm.log)、
[额外 disarm 返回](../../outputs/ros2_robot_control_hardware/nomad-control-20260909-lktrZy/first-disarm.log)、
[网关日志](../../outputs/ros2_robot_control_hardware/nomad-control-20260909-lktrZy/gateway-peers32.log)。

后续受控运动记录至少填写：日期/操作者、软件运行目录、arm/disarm 返回、实际 `/cmd_vel` 和 `/odom` 峰值、运动方向、停止情况、是否触发物理急停；未发生的测试不得记为通过。
