# 校园网 NoMaD API 部署

第一次克隆／换电脑或小车，请先执行[根 README](../../README.md)和[车端首次配置](../../docs/首次部署_车端配置.md)。本页的“已部署”、IP 和历史验收均指作者原环境，不表示新设备已经安装；权重、证书、身份登记和运行报告不随仓库分发。

本入口与旧热点 DDS 流程并存，不修改校园网 DHCP，不关闭代理或防火墙。
这是 **NoMaD 无目标局部探索推理**，不是已具备地图导航、群组感知或安全避障保证的系统。
真实底盘仍须现场监督、清空区域、可用物理急停；连续挡位不等于无人值守。

## 当前状态（2026-09-20，北京时间）

独立 CUDA API 与 RViz 已部署并完成显示验收：用户确认能看到相机与预测轨迹；
实际关闭/重开 `nomad-campus-rviz` 后，`nomad-campus-api` 容器 ID 未变，健康检查仍为 HTTP 200、ready。
预览改用专用 4 MiB tmpfs 内存卷，已核验快照权限 0600；viewer 只读挂载预览且 `--network none`，不持有 API 凭据。
此前 0.05 m/s 暂停恢复版本的记录：664 项回归通过、1 项因 Linux 测试环境无 PowerShell 跳过；对应 PowerShell 模拟测试已在 Windows 通过。

本次将校园 API 的速度上限扩展为 **0.05、0.1、0.2 m/s** 三档，默认仍为 **0.05 m/s**。
三档均支持显式 `-NetworkPauseResume`：仅连续挡，超时输出零速度等待，3 个不同源时间戳的新鲜预测（采集晚于暂停时已见帧）且停稳后，以 0.025 m/s² 的指令加速度上限恢复原行驶意图；手停、急停及硬故障仍取消意图。
默认不开启自动恢复；本次小车控制器、网关及配套启动代码已备份并更新，六个文件 SHA-256 与本地一致。完整回归 744 项通过，Linux 跳过的 1 项 PowerShell 测试已在 Windows 单独补测通过；三档实际 ROS 节点隔离测试均通过。本次没有启动底盘或解锁，无新增真机运动验收；旧测试记录不代表新档位已完成运动验证。以后仅更新 Windows 文件/API 容器不会自动更新车端代码。
部署和使用见[启动与暂停恢复](../../docs/校园网导航启动与暂停恢复_20260919-005412.md)。

**显示与生命周期验收不等于持续闭环验收。** 此前180秒相机 shadow 记录4次新鲜度故障，
不能写作“持续新鲜度已修复”；暂停恢复是在原期限内停车等待，不是让过期预测继续控制。RViz 仅显示 `SERVER PREDICTIONS ONLY`，
不证明车端接受结果或实际执行。详见[独立 RViz 使用说明](campus_rviz.md)。

### 此前正式链路复测（2026-09-18 23:37）

23:35 按用户要求再次启动一次，拟使用0.05m/s持续挡；仍在解锁前触发stale_result并停组。
这次接受2次预测，服务端3次HTTP均200；没有发送解锁/非零速度，进程清理与NTP恢复已复核。
详见[启动复测](../../outputs/ros2_robot_operator/console__20260918T153516.280643Z__30f5e9e7/启动复测_20260918-233711.md)。持续挡已存在，持续新鲜度问题仍未解决。

此前已在现场确认急停可用后完成一次**未解锁零速度联调**：启动零输出检查通过，底盘与相机启动，
正式 API 客户端接受并发布5个有效预测；随后 `stale_result` 锁存退出，required故障关闭整组。
本次6个服务端请求均HTTP 200，不是HTTP超时；持续新鲜度仍未通过，**不要解锁运动**。
校时误差上界30.5ms；本会话进程已退出、原NTP恢复、会话token已移除。
操作台 `cleanup_failed` 中的disarm服务不可用是控制器先退出后的收尾告警，原始错误保留；
没有取得运行阶段连续cmd_vel/odom审计，不能据此宣称全程物理静止或运动验收通过。
详见[零速度联调记录](../../outputs/campus_api/integration__20260918T152137Z__89614c76/零速度联调_20260918-232407.md)。

### 此前 shadow 性能与传输诊断（23:11）

已切换为CUDA后台服务，同名容器 `nomad-campus-api` 保持待机；模型固定在同一个推理线程加载、预热及处理请求，
保留忙时拒绝、不积压请求和原超时。正式客户端故障锁存、HTTP状态码分类和服务端分段日志已部署，524项回归通过。
暂停其他推理后的180秒测试收到540个有效结果，平均模型推理131ms、请求到接受322ms；
仍有4次请求失败、16次旧预测过期，**持续新鲜度未通过，不解锁运动**。

后续60秒诊断短测收到174个有效结果、4次HTTP超时；每次请求约307KB。
两次504的body接收已耗时481/536ms，两次408到600ms尚未收齐，显示本轮失败的主要预算消耗在请求接收阶段。
不据此断言Wi-Fi信号弱；TLS、客户端构建请求等额外耗时也不包含在服务端body计时内。
下一步优化传输/重复上下文数据，独立验证后再考虑最新帧选择；不通过延长超时掩盖问题。

shadow runner自然退出的进程扫描竞态已修复，60秒真机复验completed、cleanup_verified=true。
相机/client已退出，无底盘驱动/控制器/gateway/解锁，NTP为active；CUDA服务保持待机。
证据见[GPU空闲三分钟复测](../../outputs/campus_api/deployment__20260918T141000Z/GPU空闲复测_20260918-230634.md)
及[传输分段诊断](../../outputs/campus_api/deployment__20260918T141000Z/校园网传输诊断_20260918-231119.md)。

### 首次CPU部署记录（22:31，以下为历史状态）

已部署并实际验证机器人通过校园网调用 HTTPS API。当时 `nomad-campus-api` 正在运行，
为**无图形窗口的 CPU 诊断模式**；GPU 被已有训练任务占用，本次未停止或修改该任务。
最新45秒真实相机测试收到111次有效结果、0次请求结果拒绝，平均请求到接受耗时377ms，
但仍出现12次旧预测过期。**暂不要解锁运动**：下一步须释放GPU资源，切换CUDA并完成连续新鲜度测试。

485项回归和两个ROS包的隔离构建通过；校园入口 `-SyncOnly` 在真机完成，校时误差测量上界约33.6ms，
退出后原NTP服务已恢复。测试未启动底盘、控制器或gateway，也未解锁；相机/client已清理。
当时一次带RViz测试因WSLg X11断开导致旧版服务退出，不能据此判断机器人Wi-Fi断线。
这是历史问题；现已改为两个独立容器，并完成 GUI 关闭/重开不影响 API 及图像/轨迹显示验收。
详细证据见[本轮部署验收](../../outputs/campus_api/deployment__20260918T141000Z/校园网API部署验收_20260918-223143.md)。

## 地址与职责

### 物理设备与软件运行层

目前按参与导航的物理设备／部署主机计算，共 **2 个物理节点**：

1. **电脑：计算与操作节点。** Windows 主机提供 GPU 和项目文件；Docker Desktop 的 WSL 2 后端承载 `nomad-campus-api` 与 `nomad-campus-rviz` 两个容器，操作台在 Windows PowerShell 运行。这些软件环境和容器都属于同一台物理电脑，用户安装的 `Ubuntu-22.04` WSL 发行版也不另算一台设备。
2. **小车：感知、控制与执行节点。** 车载计算机运行 ROS 2、相机／里程计相关节点、API 客户端、控制器、独立安全网关和底盘驱动。以整车为一个物理节点统计，不单列车内传感器或底盘 MCU；这不意味着小车内部只有一颗处理器。当前电脑的模型服务容器不部署到小车。

物理节点数、容器数、进程数和 ROS 2 节点数是不同统计口径：多个 ROS 2 软件节点可同处一台设备，启动／停止容器不会改变物理设备数。校园网 AP、交换机和路由器仅作为通信基础设施，不计入这两个导航计算／执行节点。这里描述部署结构，不是实时在线、进程运行或已解锁状态的检查结果。

### 地址与组件位置

当前服务器是 Windows 电脑 `10.201.91.120`，HTTPS 端口 `8443`。
小车当前地址为 `10.201.207.37`。小车保持 DHCP；地址改变时只需给 SSH 操作入口传新 `-RobotHost`，
不需要修改模型 API，也不以动态 IP 作为机器人身份。

- `nomad-campus-api`：独立服务容器，复用已有 ROS2 runtime 镜像与 NoMaD 权重；只返回轨迹，不控制机器人。
- 小车 `campus_api_client`：本地采样相机四帧，调用 HTTPS，验证响应后发布本机带时间戳的 waypoint/模型状态。
- 小车原控制器/独立 gateway：计算速度、限制速度/加速度/曲率、检测超时、执行显式解锁；服务器不发布 `/cmd_vel`。
- `nomad-campus-rviz`：独立 GUI 容器，只读访问最新内存快照，不接入校园网，也不挂载 API token/私钥。
  本机独立 ROS 域 **87** 显示 API 图像和预测轨迹；小车本机使用域 **0**。两端均 `ROS_LOCALHOST_ONLY=1`。
  不依赖跨校园网 DDS、组播、旧 `.100/.180` 通告地址或外部 locator。

### 数据与控制链路

- **推理链路**：小车相机 → 车端选帧／压缩和 API 客户端 → 校园网 HTTPS → 电脑 NoMaD GPU 推理 → 响应返回小车 → 车端校验并发布 waypoint／模型状态。里程计用于车端控制保护，不是当前图像推理 API 必须上传的输入。
- **执行链路**：车端 waypoint → 本地 `robot_control` 计算并限幅速度 → 独立 `robot_gateway` 校验并发布 `/cmd_vel` → 厂商底盘驱动执行。网络恢复不绕过新鲜度、解锁和本地保护检查，电脑 API 不直接驱动车轮。
- **操作链路**：Windows 操作台通过 SSH 连接小车 supervisor，管理启动、人工 RUN、停止及心跳／重连；这与 HTTPS 推理请求是不同连接，适用各自的暂停／恢复规则。
- **预览链路**：电脑 API → 同机专用 tmpfs 预测快照 → `nomad-campus-rviz` 内的预览 ROS 2 节点／RViz。它不通过校园网 DDS 接入小车控制图，也不增加第三个物理节点。

RViz 标注 `SERVER PREDICTIONS ONLY`：这些是服务器预测，不代表小车已接受或实际执行；无全局地图/里程计轨迹。
图像保留原采集时间戳。仅用于本机显示的 Marker 使用绘制时间，不会进入控制话题。
域87内的 `nomad_preview_origin → base_link` 是显示专用单位变换，不是真实定位/里程计；不会发到机器人域0。
预览约1秒无新快照后清空画面/轨迹并显示 `STALE / WAITING`；这个显示 TTL 不是车端数据新鲜度验收。

## 安全与 API

### 图像校验复用（2026-09-20，已部署，待完整负载复测）

内部选帧请求现在使用 `frames[].jpeg_bytes`，不再先编码 Base64 再解码。该结构仅在车端内存中使用，不能直接发给 HTTP；线上 JSON 仍只有 `jpeg_b64`，协议版本不变。

`UploadImagePreparer` 保留原来的 OpenCV 缩小/JPEG85 路径，八条有界缓存同时保存不可变 JPEG 和 Base64。缓存命中时不重新进行像素解码、重压缩或 Base64 编码。生成的输出不在 preparer 末尾重复 Base64 解码校验；HTTPS 发出前仍严格检查一次。服务端接收时严格检查得到的 JPEG bytes 直接交给图像解码器，不再 Base64 解码第二遍。

这不是关闭身份、序号、源时间戳、新鲜度、长度/尺寸和畸形图像检查，也不延长任何期限。只去掉内部重复转换和重复图像解码。图像内容、四帧上下文、模型与控制参数不变。

本地实现证据：`outputs/campus_api/image_boundary__20260920T081720Z/`。随后已在身份/空闲/互斥锁检查通过后部署车端四个配套模块，备份为 `/home/wheeltec/nomad_live_control/backups/latency_fix__20260920T074300Z_optimized2/before.tar`；2026-09-20 16:46 北京时间重启现有 API 容器加载新代码，CUDA ready。ARM 车端六次无 ROS/硬件合成图认证 HTTPS/GPU 推理全部通过，源码哈希一致，原有 0.8 秒预算及客户端新鲜度校验仍有效。本轮没有启动相机、底盘或控制节点，没有解锁；真实相机与未解锁完整负载复验仍待完成。部署证据：`outputs/campus_api/deployment__20260920T084700Z/`。没有轮换 TLS/token 或修改控制配置。

### 线上接口与控制保护

- `GET /healthz`：只返回版本与 ready；ready 表示模型已加载，不表示已完成持续性能/运动验收。
- `POST /v1/infer`：HTTPS + Bearer token；机器人必须验证服务器证书/IP，禁止跳过 TLS 校验。
- 四帧 JPEG + 原始采集时间 + 会话 UUID + 单调序号；响应携带相同标识、waypoint、8×8 候选轨迹。
- 单机器人活动会话；新会话需旧会话空闲五秒，拒绝旧会话/重复序号；每个服务器进程最多128个会话。
- 单个推理占用，忙返回503；最多4个HTTP处理线程；输入体最多4MiB，JPEG最多768KiB/张。
- 服务处理预算0.6秒，机器人完整请求预算0.8秒；原始图像/预测数据期限1秒，waypoint超时0.6秒。
- 图像断流、结果过期、网络断开或身份/协议异常都不能继续沿用旧结果；网络恢复不自动重新解锁。显式暂停模式仅保留已授权连续意图，不调用解锁服务。
- 默认正式 `--control-output` 模式将首次真实故障永久锁存，并退出客户端；required进程退出使整组进入收尾，不能在同一会话恢复为running。
  操作者须检查原因、确认停止后重新启动会话，再独立解锁。shadow模式仍允许恢复，用于统计诊断，不提供运动权限。
- `--network-pause-resume` 仅把列举的网络/相机间断/数据过期改成 `network_wait`；鉴权、身份、格式、回拨、竞争发布者仍硬锁存。控制器独立检查原期限，连续挡暂停时保持 20Hz 零指令，新鲜数据与停稳条件满足后才恢复。
- 三个速度档均允许上述可恢复等待，不延长任何请求/图像/waypoint 期限，不沿用过期指令。定时挡故障锁停不变；手停、急停、时钟回拨或里程计异常取消恢复意图。操作台断联默认取消，另有显式 `--operator-pause-resume` 的有界续行模式（见下）。
- 0.2 m/s 与 1 秒视觉数据年龄可对应 20 cm 理论行进距离，另需考虑制动距离；指令上限不保证实际车速。本系统没有独立障碍物停车保护，升档须在现场安全区域重新验证，不能无人值守。
- 保留原本20Hz本地控制、独立250ms desired-command watchdog、SSH操作心跳、时间回拨和速度保护。
- `--control-output` 需要本机身份登记和近期校时会话 token；手工裸启动正式发布者会被拒绝。
- 不在日志保存密码、Bearer token或原始请求图像；运行报告使用 UTC 时间戳＋随机后缀，不覆盖历史结果。
  独立 RViz 使用**含最新一帧图像的临时内存快照**，不是完全不产生图像文件；它只写专用 tmpfs 卷，
  不写 Windows 项目磁盘、不保存逐帧历史、不含凭据，细节见[数据与隔离](campus_rviz.md#数据与隔离)。
- 服务端 `request_stats` 记录已声明/已接收body字节、body/JSON/service/总耗时和阶段；计时从TLS及HTTP头之后开始。
  `service_ms` 含校验、JPEG解码、模型及回调，不等同纯GPU时间。客户端 `rejection_counts.http_408/http_504` 区分HTTP错误；
  控制状态仍使用 `network_error` 触发原有保护，不能按该状态名直接推断Wi-Fi掉线。

## PC 启动

在 Windows PowerShell：

```powershell
cd F:\SocialEmbodiedAgent
# 首次创建后台服务，空闲GPU显存需至少2GiB；不会停止其他任务。
.\docker\ros2\start_campus_api.ps1
```

GPU被其他任务占用时，只能显式选择诊断CPU模式：

```powershell
.\docker\ros2\start_campus_api.ps1 -Device cpu
```

两种模式均保持所有截止时间，不静默降级或放宽超时。CPU模式能返回预测不等于稳定闭环通过。
入口拒绝替换已存在的同名容器；不要在另一窗口重复创建。已配置容器的普通启动/查看：

```powershell
docker start nomad-campus-api
docker logs --tail 30 nomad-campus-api
python .\ros2_ws\scripts\campus_health.py --server-url https://10.201.91.120:8443 --ca-file "$env:LOCALAPPDATA\SocialEmbodiedAgent\campus-api\server.crt"
```

`docker start`沿用创建时的CPU/GPU设置，不会自动切换。更换模式需先结束机器人操作会话，
确认停车，再显式重建服务容器；不使用全局prune。
2GiB空闲显存预检查只在创建脚本内执行；已有CUDA容器直接 `docker start` 会绕过该检查，
每次重启前仍须单独检查GPU空闲资源，不要假定该命令会保护其他训练任务。显式CPU模式不占用CUDA；当前运行的是CUDA模式。
首次创建时可额外传 `-WithRViz` 请求启动独立 GUI；关闭窗口、WSLg 故障或 viewer 启动失败均不会停止 API。
API 已在运行时，单独打开或重开 RViz，不重建服务：

```powershell
.\docker\ros2\start_campus_rviz.ps1
docker logs --tail 30 nomad-campus-rviz
# 仅关闭预览，不停推理：
docker stop -t 10 nomad-campus-rviz
```

GUI 容器退出后自动移除；已有同名 viewer 时入口拒绝重复创建。未传 `-WithRViz` 的服务仍可按以上命令打开 GUI。
两个入口核验 `socialembodiedagent-campus-preview` 为带专用标签的 4 MiB、0700 tmpfs 卷；
同名卷配置冲突时拒绝使用，不自动删除/改写。API 在容器专用 preview 路径读写此卷，viewer 只读挂载到 `/preview`。

证书/私钥/token位于 Windows `%LOCALAPPDATA%\SocialEmbodiedAgent\campus-api`，目录ACL限当前用户和SYSTEM。
服务器私钥不复制到小车；证书有效90天，服务器IP变化或到期须显式重新签发并安全分发。
不要把凭据目录上传Git。项目源码、模型权重与运行报告仍在项目挂载路径；最新相机预览是上述 tmpfs 内存卷的例外。

## 无运动测试（首次建议）

SSH登录小车后执行：

```bash
source /home/wheeltec/nomad_live_control/campus/setup.bash
python3 /home/wheeltec/nomad_live_control/campus/run_campus_shadow.py --seconds 45
```

这只启动相机和默认 shadow API 客户端，不启动底盘、控制器、gateway，不发布运动指令。
输出是 `/nomad/campus_shadow/*`，不能被正式控制器消费。
运行前检查身份、互斥锁和已有进程；终止时按会话标记、PID出生信息及pidfd清理本次子孙，
不能把父进程退出当成所有相机子进程均退出。报告 `cleanup_verified` 为false时需检查，不要直接重启。

## 正式接入（仍不自动运动）

确认推理服务就绪并先退出 shadow 测试。在 Windows 新开一个操作终端：

```powershell
cd F:\SocialEmbodiedAgent
.\docker\ros2\start_robot_campus.ps1 -RobotHost 10.201.207.37
```

当前 Windows 用户已保存小车凭据，操作台自动填写 SSH 和 sudo 密码；项目、命令参数及运行日志不保存明文密码。入口复用校时、未解锁启动、零输出检查和可靠清理。需要手动输入时添加 `-ManualPassword`（Python 操作台对应 `--manual-password`）；密码自动填写不代替运动确认。
校园网模式默认0.05m/s；旧热点流程默认值保持不变。先核实实时相机/预测和现场安全条件，
再由操作者独立选择 `t` 限时试动，输入时长及 `RUN`。`c` 是持续挡位；未显式开启暂停恢复时，输入/网络故障仍锁停。
`s`解除使能并取消自动恢复意图，`q`结束会话并恢复原校时服务。

凭据由 Windows 凭据管理器按当前用户保存，固定名 `SocialEmbodiedAgent/robot/wheeltec@192.168.0.100` 中的地址是身份别名，不是当前校园网连接地址。同一 Windows 账户下的进程仍可能读取该凭据，不能理解为对这些进程绝对隔离。管理命令如下，密码只通过隐藏输入提供，不作为命令参数：

```powershell
python .\ros2_ws\scripts\robot_credentials.py --status  # 不显示密码
python .\ros2_ws\scripts\robot_credentials.py --save    # 新增或更新
python .\ros2_ws\scripts\robot_credentials.py --delete  # 取消保存
```

自动 SSH 填写仅用于本项目操作台，不改变普通手工 `ssh` 的行为；无需额外 pip 依赖，使用 Windows 自带 OpenSSH 与 .NET Framework 本地助手。已完成真实自动 SSH 握手、只读 `plan` 和会话关闭验证；未执行 `prepare`/`start`/`arm`，没有借此启动硬件或解锁。

需要连续挡在偶发网络超时后停车等待、恢复新鲜预测后继续时，选以下一条命令；启动本身不解锁：

```powershell
# 默认低速档：0.05 m/s
.\docker\ros2\start_robot_campus.ps1 -RobotHost 10.201.207.37 -MaxLinearMps 0.05 -NetworkPauseResume
# 0.1 m/s 档
.\docker\ros2\start_robot_campus.ps1 -RobotHost 10.201.207.37 -MaxLinearMps 0.1 -NetworkPauseResume
# 0.2 m/s 档
.\docker\ros2\start_robot_campus.ps1 -RobotHost 10.201.207.37 -MaxLinearMps 0.2 -NetworkPauseResume
```

每次仅运行一个操作台；换档须先 `s`、现场确认停车、`q` 等待清理，再重新启动。操作台内 `i` 检查状态，确认安全后 `c` → `RUN`。
网络暂停时 `armed=true, motion_paused=true` 表示保留本次连续意图且当前输出零速度；网络恢复后可能自行继续，必须持续看护。
在现场未确认急停和区域安全、新档位代码未部署验证或GPU资源未落实前，不应解锁。不要在行驶中搬动/转动车辆；里程计异常会独立取消行驶意图。
结束顺序：`s` → 现场确认停住 → `q`等清理完成 → `docker stop -t 10 nomad-campus-api`。

## 已部署位置与回退

### 操作台重连协议

部署状态：2026-09-20 晚间六个车端文件已备份更新并核对哈希；最终 Linux 回归 952 通过，真实 SSH 空会话断开/重连/退出检查通过。没有启动硬件或解锁；完整负载和运动验收另行进行。证据见[部署验证记录](../../outputs/operator_reconnect/deployment__20260920T111500Z/操作台重连部署验证_20260920-192300.md)。

正常校园 API 操作台使用 `campus_session_rpc.py` 经 SSH 连接车端 `robot_session_server.py`。车端 supervisor 持有原 `OperatorSession`，SSH proxy 只负责附着连接；EOF 不再等同于销毁相机/推理组。DDS 旧入口和 API `--sync-only` 保持原路径，不采用该运行期协议。

PC 每 0.5 秒回答车端的一次性 challenge。行驶授权到期时间锚定 challenge 发出时的车端单调时钟，最长 2 秒；延迟到达的应答不从到达时重新获得 2 秒。`operator_guard.py` 在控制器内独立验证 session、generation、sequence、deadline 和一次性 arm nonce；本地 supervisor 存活或重复转发心跳不能自行续期。guard 启用时旧 `/arm`、`/arm_for` 不允许绕过这条授权链，`/disarm` 仍可安全调用。

默认策略：操作台断开或授权超时会解除使能并取消网络自动恢复意图。运行会话在监督进程正常工作的前提下保留最多 120 秒用于重连；连接恢复只建立新的未解锁代次，不重放 `prepare`/校时/`start`/`arm`。每次 RUN 使用有时效、单次消费、绑定连接代次的 ticket，断线前的提示也必须重新确认。严重进程故障、等待窗口届满或 `q` 仍按原流程先停本组，再恢复校时；不能把保留窗口理解成失联可继续行驶。

单独的 `-NetworkPauseResume` 只覆盖图像/API 超时且操作台授权仍有效的情况，不改变上述操作台失联后须新 RUN 的默认规则。

新增显式 `-OperatorPauseResume`（`--operator-pause-resume`），必须同时启用 `-NetworkPauseResume`，禁止与 `-SyncOnly` 或旧 DDS 入口组合。三档速度均支持，启动依然未解锁；已确认的连续任务才能在普通 SSH EOF/心跳到期时保留意图。最长 2 秒车端单调授权到期后强制零输出，不沿用失效预测；控制器首次暂停启动独立 **30 秒** 恢复窗口，重复掉线或握手不续期。新 generation 后必须取得 3 个采集时间晚于重连 ROS 时刻的新鲜预测、停稳且全部安全条件正常，才从零加速继续；没有自动发送 arm 请求。

人工 `s`/`q`、急停、协议故障、解锁 ACK 不确定、定时挡、硬件/里程计/时钟硬故障、模型身份改变、30 秒窗口过期均不自动续行。PC 保留断线中的 disarm，并在重连后的首个续租 ping 前确认取消；并发取消使用递增版本，不能被旧 ACK 清除。新握手并不抹掉初始化期间挂起的硬取消。合法同代次迟到心跳只有严格匹配已接受的原过期期限时才可消费序号并维持暂停，不能续租/恢复；异常期限、回放等仍硬取消。

30 秒控制意图窗口与 120 秒会话保留窗口分离。`armed=true, operator_paused=true` 只表示待恢复的任务意图且输出为零；`operator_resume_remaining_sec` 是剩余恢复窗口。省略新选项仍要求重新 RUN。首次实车复测建议：

```powershell
.\docker\ros2\start_robot_campus.ps1 -RobotHost 10.201.207.37 -MaxLinearMps 0.05 -NetworkPauseResume -OperatorPauseResume
```

2026-09-20 本功能车端六文件已备份部署、SHA-256 和 ARM 导入匹配；三档隔离 ROS 节点验证通过（localhost domain 87，无 gateway 或 `/cmd_vel` 发布），不是物理运行验收。记录见[操作台自动续行](../../outputs/operator_reconnect/operator_resume__20260920T121200Z/操作台自动续行部署记录.md)。保持原操作台打开并持续现场看护，未重新确认现场安全前不能借部署操作解锁。

车端 `.sessions/<UUID>/` 是 owner-only 目录，Unix socket 同用户可访问；随机连接凭据只经 SSH stdin 和 socket 传递，不落盘、不进命令参数或环境。原操作台退出/电脑重启后不能从另一个操作台自动找回原会话。人工密码模式后台重连为 `BatchMode=yes`，仅已有 key/agent 认证可用；默认凭据自动填写模式可自动重连。

2026-09-20 晚间追加诊断修复：PC 与车端均需共享 `operator_errors.py`，车端初始化失败保存 0600 的 `failure__<request_id>.json`，PC 仅显示白名单错误码/固定文案和有界数字 PID。任意异常内容及凭据不回显；已确认的拒绝不能被紧接的 closed 消息掩盖，原失败清理及不自动重放规则不变。两文件车端补丁已部署；974 项回归通过，新协议 prepare/sudo 校时/verify/退出通过，未启动硬件。首发 prepare 拒绝的瞬时条件未能复现，不能把诊断修复表述为已确认根因修复，见[记录](../../outputs/operator_reconnect/prepare_fix__20260920T114200Z/prepare拒绝排查与诊断修复_20260920-194700.md)。

部署必须成套更新 PC 的 `robot_console.py`、`campus_session_rpc.py` 和启动 PowerShell，以及车端 `robot_operator.py`、`robot_session_server.py`、`operator_guard.py`、`robot_control_core.py`、`robot_control_node.py`、`robot_startup_check.py`，并保留已部署的 `operator_errors.py`、`campus/run_campus_shadow.py`。只更新 PC 或只重启 API 不会更新机器人；启动检查要求车端报告匹配 session 的 guard 和相同暂停策略，不能静默回退至旧无 guard 控制器。监督进程被强杀后独立控制器仍有到期归零保护，但进程组/NTP 的完整恢复不能据此保证。

2026-09-20 延迟优化已部署：preparer 复用完整校验时得到的原始 JPEG bytes，减少一次重复 Base64 解码；所有边界校验、图像质量和控制期限不变。退出报告新增 `upload_transport.http_transport`，各分段耗时最多保留 4096 个数值，便于区分图像准备、TCP/TLS、服务响应等待和响应校验。完整负载未解锁复测接受 214 个预测，仍有 11 次旧预测过期；通过零输出状态复测不等于持续运动验收。证据见项目 `outputs/campus_api/latency_fix__20260920T074300Z/`，本轮四文件备份见车端 `backups/latency_fix__20260920T074300Z_{metrics,optimized}/before.tar`。

- 小车代码：`/home/wheeltec/nomad_live_control/nomad_navigation/`。
- 校园入口配置：`/home/wheeltec/nomad_live_control/campus/{setup.bash,client.json,identity.json,server.crt,token}`。
- `identity.json`绑定ARM架构、machine-id、永久Wi-Fi MAC、hostname；SSH继续校验此前登记的小车主机密钥。
- 旧代码备份：`/home/wheeltec/nomad_live_control/backups/campus__20260918T141000Z/`；未删除旧热点配置。
- PC服务证据：`F:\SocialEmbodiedAgent\outputs\campus_api\server__*`。
- 小车预览证据：`/home/wheeltec/nomad_live_control/runs/campus_shadow/shadow__*`；正式客户端为`runs/campus/campus_client__*`。
- 校时/操作报告仍在项目 `outputs/ros2_robot_operator/console__*`。

服务停止或改地址不等于硬件急停；固件watchdog、制动距离、速度标定与长期运动验收仍需单独完成。
