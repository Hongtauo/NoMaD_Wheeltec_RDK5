# 小车相机与容器 RViz 接入记录

更新：2026-09-09。当前状态：**图像链路低帧率已修复，原版 Humble/Fast DDS 2.6.12 连续 180 秒接收 5328 帧、约 29.60 Hz，无需升级 DDS；原部署未迁移，RViz 小车画面尚未验收。**

### 最新：真机相机 → NoMaD 推理通过（2026-09-09 19:20）

已从原 ros2-dev 的已装依赖层保存本地快照，创建独立 host+GPU 容器 `nomad-robot-inference`，未改原容器。
NoMaD 节点新增 compressed 输入，保留小车原始时间戳与1秒超时；输出固定到 `/nomad/robot_preview/...`，没有控制器或速度发布者。
单一 Context/participant 中的审计节点实际收到并核对 waypoint；60秒238组、180秒712组，后者平均推理126.78ms。
补齐持续出点检查后的最终版本另做30秒复测通过，115组点。95项包测试与13项入口纯测试通过。
使用方法、各次独立归档及边界见 `ros2_ws/docs/robot_inference.md`。不能同时启动共用7430/7431的相机RViz；真机轨迹GUI、多进程布局、校时保持与底盘仍待验收。

### 最新：仅通告 Wi-Fi 地址，帧率恢复（2026-09-09 18:44）

新增 `docker/ros2/fastdds_extlab/robot_wifi_only.xml`：将默认用户单播与发现单播地址显式设为 192.168.0.180，端口 7431/7430；移除 External Locators，保持 interfaceWhiteList 为空。抓包验证不再通告 .65.*；内核 socket 仍监听 0.0.0.0。
2.10.7 复测 60 秒约 29.60 Hz，切回旧配置 25 秒再次降至 0.321 Hz；原版 2.6.12 的 180 秒测试达到同等 29.60 Hz，零解码错误。小车发送跟踪只剩 .0.180，最长 1.041 ms，原 3.1 秒阻塞消失。

因此当前接收链路不需要升级 DDS。适用前提为 Docker Desktop host networking + host 容器、电脑 Wi-Fi IP 不变、域 0，并且固定端口只启动一个 participant。不能直接用于原 bridge 模式 ros2-dev，也不能让多个 ROS 进程共用固定端口。XML 不是运动访问控制策略，不全局启用。

本轮发现小车刚重启约 3 分钟，时间再次回到 2000 年；已重新校时、写 RTC，并恢复厂商相机（仅相机，无底盘）。NTP 上游仍 reach 0，RTC 重启保持未解决，不能把上轮校时标成永久修复。
配置与命令见 `docker/ros2/fastdds_extlab/README.md`；本轮证据见 `outputs/network_diagnostics/wifi_locator_fix__20260909T104300Z/summary.md`。
以下旧状态保留为历史，不代表当前帧率；下一步为单进程 RViz 图像验收及正式部署布局设计。

### 最新：小车校时完成、低帧率发送阻塞已定位（2026-09-09 16:18）

小车系统时间已从 2000 年对齐电脑并写入 RTC，保持 Asia/Shanghai、RTC UTC。原 ntp.service 已恢复，但上游 reach 为 0；Windows 也未完成 NTP 同步，因此长期自动校时仍待解决。
校时后最小 RTT 2.995 ms 的测量估计小车减电脑偏差为 −0.360 ms（网络中点估计，不代表绝对 UTC 精度）；相机消息已带 2026 年时间戳，未测试重启后 RTC 保持。

校时后相机复测约 0.38–0.42 Hz，最大间隔仍为 3.13 秒。小车压缩发布进程对 .65.* 内部地址及正确 .0.180 地址均发图；12 秒 strace 中对 192.168.65.6 发送累计阻塞 10.04 秒、单次最长 3.106 秒，对 Wi-Fi 地址最长仅 0.675 ms。
内部地址路由走 usb0，网关邻居 INCOMPLETE；External Locators 仅追加正确地址而未移除错误地址，导致发送线程被拖慢。具体内核队列机制未进一步跟踪，尚未实施修复后对照。

本轮只修正时间并诊断帧率，没有修改 DDS、路由或相机配置；底盘未启动，控制器保护未关闭。临时探针/SSH 已结束，实验容器已停止，小车原相机继续运行。
详见 `outputs/network_diagnostics/clock_and_framerate__20260909T081722Z/summary.md`。下方为历史检查记录，时间及“未定位”等旧状态以上述最新结论为准。

### 最新：External Locators 独立兼容性验证（2026-09-09 16:00）

用户授权在独立环境测试支持 External Locators 的 Fast DDS。已建立 `nomad-fastdds-extlab-20260909`，
在 `/opt/extlab` 构建 Fast DDS 2.10.7，并重编译当前 Humble RMW 6.2.10 的两个包，未覆盖系统库。
回环域内原版 2.6.12 与新版双向 String/Trigger 测试通过，动态库路径确认无混载。
通告新增 192.168.0.180:7430/7431 后，成功发现小车并接收彩色压缩图像。
Reliable 60 秒 19 帧、Best Effort 60 秒 18 帧，均 640×480、零解码错误，但约 0.32 Hz，不满足实时导航要求。
小车实际版本另已核验为 Fast DDS 2.6.10、RMW 6.2.7；测试后本地复测 297 帧/10 秒。
本轮未改机器人参数、底盘、时钟或防火墙，也未将新版迁移至原 `ros2-dev`。
详细结果见 `outputs/network_diagnostics/fastdds_external__20260909T080100Z/summary.md`；
复现方式见 `docker/ros2/fastdds_extlab/README.md`。2.10.7 为较旧/EOL 实验版本，不作为生产升级建议。

### 最新 DDS 报文诊断（2026-09-09 15:36）

已通过双侧抓包确认：host 容器的 SPDP 到达小车 7410/7412/7414/7416，但通告的发现返回地址为
192.168.65.3/192.168.65.6:7410，默认数据地址为相同内部 IP 的 7411。小车到这些 IP 的路由走 usb0。
Desktop 将 IP 包头来源转为 192.168.0.180，却未同步修改 SPDP 内的地址。
小车彩色压缩节点实际通告 192.168.0.100:7412（发现）和 :7413（默认用户数据），相机节点为 :7416/:7417。
独立定向 UDP 测试证明电脑 Wi-Fi 地址的 7410/7411 各能收到 5 个包；不能用“所有 UDP 被阻止”解释现象。
已确认通告地址不匹配问题，但未修复后复验，不能断言唯一根因。订阅仍为 0 帧；未改网络或底盘配置。
双侧元数据及完整结论：`outputs/network_diagnostics/dds_locators__20260909T073645Z/summary.md`。

### 最新：主机重启及 host networking 对照测试（2026-09-09 15:25）

- 用户重启主机后，Ubuntu WSL 实测已为 `mirrored`；重新连接小车 Wi-Fi 后，到 192.168.0.100 的路由为 `dev eth4 src 192.168.0.180`。此前 None 状态已解除。
- bridge 容器复测地址为 172.17.0.4，15 秒相机订阅 0 帧；小车本地再次实收压缩图像 149 帧/5 秒，相机持续运行。小车到 172.17.0.4 的路由仍走 usb0。
- 用户开启 Docker Desktop host networking 后，配置实测 `HostNetworkingEnabled=True`。
- 使用原始本地 ROS 镜像创建 `--rm --network host` 临时容器，域 0、UDP-only 静态 peer 指向小车，20 秒相机接收仍为 0 帧、0 相机发布者。SSH banner 正常，连接源地址在容器侧显示为 192.168.65.3。
- 独立 UDP 对照：小车临时绑定 192.168.0.100:45180，host 容器绑定 0.0.0.0:45181；仅交换固定诊断字符串。8 次请求均得到回复，并收到 8 次小车发往 192.168.0.180:45181 的定向测试包。
- 小车看到请求源为 192.168.0.180:60471；容器接收定向 Wi-Fi 测试包时来源显示为 127.0.0.1:38898，表明存在 Desktop 转发。不能把 host 模式视作容器直接拥有 Wi-Fi IP。
- 以上仅证明测试端口的双向 UDP 可用；尚未捕获 DDS 通告地址，不能据此断言唯一根因，也未排除 DDS 端口规则、发现配置或端口转换问题。
- 镜像 Fast DDS 包版本为 2.6.12；下一步需按此版本核验 DDS 通告地址、发现/数据端口，不能直接套用更新版本的 external locator 配置。
- 两个临时测试容器均自动清理，小车 UDP 监听在 35 秒后结束，诊断 SSH 已退出。未启动 RViz（图像验收未通过），未改防火墙或小车配置，未启动底盘。
- 用户应用 Docker 设置后原容器已停止；本轮没有删除、迁移或重建这些原容器，也未恢复其中的工作进程。

### 2026-09-09 重启验证补充

用户已授权重启 WSL/Docker。5 个持久容器完成停止后，执行 Docker 退出与 WSL 重启（其中三个容器退出码 137，两个为 0）。
WSL 实际未进入 mirrored：首次启动报 `CreateInstance/CreateVm/ConfigureNetworking/0x8007054f`，
提示 mirrored 配置失败并回退到 None；`wslinfo --networking-mode` 为 `none`，仅 lo 网卡，无小车路由。
Docker Desktop 另在启动阶段报 `Docker/run/dockerInference` 端点无法访问并退出；端点改名保留尝试也失败，未删除。
随后用户手动启动 Docker 成功，实测引擎 28.5.1；已恢复原有五个持久容器，并以原离线域 180 脚本重建临时 RViz，日志显示 OpenGL 初始化成功。
恢复后 Ubuntu WSL 仍为 `none`；但 ros2-dev 到小车 SSH 可达。域 0 静态 peer 相机订阅复测 12 秒，0 帧、0 相机发布者，DDS 仍未通过。
查看器恢复不等于小车图像接入成功；没有启动视频、NoMaD 模型或运动控制。
未修改用户 mirrored 配置、防火墙或代理/VPN；小车相机为此前实测结果，本轮未重新验收。
详细过程见 `outputs/wsl_docker_restart/mirrored__20260909T062441Z/record.md`。

## 已确认信息

- 用户提供的车型为轮趣科技 R550，热点 IP `192.168.0.100`，SSH 用户 `wheeltec`。不在项目中保存密码。
- SSH 主机密钥与电脑已有 known_hosts 匹配。
- 机载板为 D-Robotics RDK X5，ARM64，Ubuntu 22.04.5，ROS 2 Humble。
- 厂商参数中的 `car_mode` 为 `V550_akm`，与用户提供的车型不一致；保留配置不改，底盘接入前需确认。
- 相机命令：`ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py`。
  已检查实际安装文件与源码一致；选用 Astra 驱动和图像压缩转发，不包含底盘或雷达启动。
- USB 枚举为 Orbbec Astra S。小车侧约 5 秒实收彩色 146 帧（640×480、rgb8，约 29.6 Hz），
  深度 140 帧（16UC1，约 28.1 Hz），压缩彩色 149 帧（JPEG，约 29.5 Hz）。
- 彩色话题 `/camera/color/image_raw`，压缩话题 `/camera/color/image_raw/compressed`，
  frame 为 `camera_color_optical_frame`。检查时 `/cmd_vel` 发布者为 0。
- 小车时区 Asia/Shanghai，系统/RTC 已于 2026-09-09 校正，RTC 使用 UTC；ntp.service active，但上游未同步。
  接入 NoMaD 前仍须解决长期自动同步并验收跨设备时间戳，不能关闭既有超时保护掩盖问题。

## 本次网络检查

1. `ros2-dev` 仍使用 Docker bridge，IP 为 `172.17.0.2`，网关 `172.17.0.1`。
2. 容器到 `192.168.0.100:22` TCP 连接成功，收到 OpenSSH banner；不代表 DDS 已双向连通。
3. 原 `nomad-rviz-viewer` 使用域 180 与 `rviz_udp.xml` 回环白名单，仍保留原配置，不用于小车直连。
4. 临时诊断进程使用域 0、非 localhost-only、默认 Fast DDS，未发现小车业务节点。
5. 新的 UDP-only 静态 peer 配置指定 `192.168.0.100`；12 秒订阅检查收到 0 帧、发现 0 个相机发布者。
6. 小车侧再次确认 `/camera/camera`、两个 republish 节点仍在运行，压缩彩色有 1 个 Reliable/Volatile 发布者。
7. 小车对 `172.17.0.2` 的路由查询返回：`via 192.168.128.1 dev usb0 src 192.168.128.10`。
   这不是通过 Wi-Fi 到容器的已验证返回路径。尚未抓包，不能把失败唯一归因于多播、NAT 或防火墙中的某一个。

## 独立配置（待连通验收）

- `docker/ros2/robot_camera_udp.xml`：仅供域 0 相机接收诊断，指定小车发现 peer，禁用 SHM。
  静态 peer 不能自行解决 NAT 或不可达的数据地址。
- `docker/ros2/robot_camera.rviz`：Image-only 配置，订阅压缩彩色话题，Best Effort/Volatile，
  不含导航目标或初始位姿工具，不启动模型或底盘。
  Humble Image display 从话题后缀选择 compressed 传输，不使用旧 `Transport Hint` 字段。
  依据：[Humble ImageTransportDisplay 实现](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/include/rviz_default_plugins/displays/image/image_transport_display.hpp)。

这两个配置尚未通过小车到容器图像接收验收。没有修改原离线预览配置、WSL 网络配置、Docker 网络配置、
防火墙、小车时钟或相机启动文件；此前诊断订阅进程均已结束。随后进行了上文已授权的 WSL/Docker 重启。

## 待用户确认的后续路线

- 先看图像：建立仅转发相机图像的 SSH 通道，在本机隔离 ROS 域重新发布给 RViz。
  不开放机器人控制接口；这属于图像转发，不等于跨机器 DDS 已打通，尚未实现。
- 原生 ROS 2 联网：先处理本次 WSL 网络初始化与 Docker 启动失败，再确认可路由网络/数据传输方案；
  进一步修改网络设置、临时回退 NAT、重置网络或放开防火墙仍需明确确认。
