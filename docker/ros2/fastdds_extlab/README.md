# Humble + Fast DDS External Locators 独立实验

状态（2026-09-09 18:44）：**已修复图像低帧率。仅通告 Wi-Fi 地址后，原版 Fast DDS 2.6.12 连续 180 秒接收 5328 帧、约 29.60 Hz。无需升级 DDS。**

此前 `robot_external.xml` 的约 0.32 Hz 结果保留为历史对照，不再作为默认使用方案。

没有修改原 `ros2-dev`、模型依赖或机器人 DDS 配置。Fast DDS 2.10.7 是较旧/EOL 分支，
本实验只用于验证 External Locators 和 Humble 适配可行性，不代表生产版本选择或完整 ROS 兼容认证。

## 独立环境

- 容器：`nomad-fastdds-extlab-20260909`，host 网络，项目只读挂载；测试完成后已停止，构建产物保留。
- 基础镜像：`osrf/ros:humble-desktop-full-jammy`，实测 image ID
  `sha256:c10a8420cb097015b6f028d63ec49215faf193a0dd699abedc843f0443e9b118`。
- 原版 `/opt/ros/humble` 保留：Fast DDS 2.6.12、Fast CDR 1.0.29、RMW 6.2.10。
- 新版 `/opt/extlab/fastdds`：Fast DDS 2.10.7，commit `92a890c1dea08682d2fc83974d96cebc4b7feb9b`。
- Overlay `/opt/extlab/rmw_ws/install`：Humble RMW 6.2.10，commit `cbe12977497c9c536317ca6488edf2f23b8c1830`，
  重编译 `rmw_fastrtps_shared_cpp` 和 `rmw_fastrtps_cpp`。
- 沿用 Fast CDR 1.0.29，不伪造 SONAME 软链接、不覆盖系统动态库；`/proc/self/maps` 与 `ldd -r` 已核验加载路径。
- 小车实际包版本：Fast DDS 2.6.10、RMW 6.2.7，未升级。

## 文件

- `build.bash`：固定源码提交的隔离构建步骤，只在新建实验容器执行；构建源和产物约 200 MB。
- `overlay.bash`：仅给当前终端启用新版 RMW，不加入系统 shell 启动配置。
- `robot_wifi_only.xml`：当前已验证配置。显式通告 `192.168.0.180:7430/7431`，实际 UDP socket 监听 `0.0.0.0`；不使用 External Locators，不添加 interfaceWhiteList。
- `robot_external.xml`：旧对照配置，使用时设置域 0、参与者 ID 10、UDP 发现监听 7430/用户数据监听 7431，
  额外通告 `192.168.0.180`；静态发现 peer 为 `192.168.0.100`。
  使用前检查 Wi-Fi IP，变化时必须更新实验配置。**同一 host 网络一次只运行一个使用该固定端口配置的进程。**
- `compatibility_probe.py`：在隔离回环域 220 测试 String 话题和 Trigger 服务；不接入运动接口。
- `camera_check.py`：只订阅压缩图像，记录接收间隔、动态库和解码结果，每次独立 UTC+UUID 输出目录。
- `camera_metrics.py` / `test_camera_metrics.py`：无 ROS 依赖的统计与 7 项单元测试；记录全程/活跃帧率、帧间隔分位数和含时钟偏差的 header age。

### 配置边界

- 仅验证 Docker Desktop host networking 已启用、容器 `--network host`、电脑 Wi-Fi 为 `192.168.0.180` 的组合。**普通 bridge 容器不能直接套用**；原 `ros2-dev` 尚未迁移。
- 每次检查 Wi-Fi IP；变化后更新 XML 中两个电脑地址。`192.168.0.100` 是小车地址。
- 整个共享 host 网络内，participant ID 10、7430/7431 同时只供一个 participant 使用。不要并行启动使用这份配置的测试脚本、RViz、`ros2 topic echo` 或 ROS daemon，即使分属不同容器。
- XML 不锁定 ROS domain，必须在命令中显式设置 `ROS_DOMAIN_ID=0`。
- 保持 `interfaceWhiteList` 为空；电脑 Wi-Fi IP 不是容器网卡 IP，将其加入 whitelist 会改变实际 socket 绑定行为。
- XML 不是访问控制策略，不会阻止 `/cmd_vel`。仅用于相机接收/RViz，**不要全局加入 `.bashrc`，不要启动控制器**。

## 重用现有实验容器

Windows PowerShell：

```powershell
docker start nomad-fastdds-extlab-20260909
docker exec -it nomad-fastdds-extlab-20260909 bash
```

容器内（新开终端，未 source 2.10 overlay；先确认热点连接、电脑 IP 和小车相机仍有效）：

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/SocialEmbodiedAgent/docker/ros2/fastdds_extlab/robot_wifi_only.xml \
python3 /workspace/SocialEmbodiedAgent/docker/ros2/fastdds_extlab/camera_check.py --seconds 60 --reliability best_effort
```

输出保存在测试容器 `/opt/extlab/results/camera__...`，并非项目只读挂载内；用 `docker cp` 导出明确的单次目录。
请不要用本配置启动控制器或直接迁移 NoMaD。小车 18:30 左右重启后再次回到 2000 年，本轮已重新校时并写 RTC，但重启保持与长期 NTP 同步仍未解决；现有控制器时钟/过期保护必须保留。

## 历史：从基础镜像复现 2.10 隔离构建

**图像接收修复不需要以下构建。** 仅在需要复现旧版 External Locators 兼容实验时使用。

如上述容器已存在，请重用，勿覆盖同名容器。以下是首次创建命令：

```powershell
docker run -d --pull never --name nomad-fastdds-extlab-20260909 --network host --mount "type=bind,source=F:/SocialEmbodiedAgent,target=/workspace/SocialEmbodiedAgent,readonly" osrf/ros:humble-desktop-full-jammy sleep infinity
docker exec nomad-fastdds-extlab-20260909 bash /workspace/SocialEmbodiedAgent/docker/ros2/fastdds_extlab/build.bash
```

需 Docker Desktop 已启用 host networking。源码与依赖 submodule 下载需要网络，构建用基础镜像已有编译依赖。
当前镜像的 colcon 不支持 `--allow-overriding` 参数，构建脚本未使用它。

## 当前修复验证

- 新配置 + 2.10.7：60 秒 1773 帧，29.596 Hz，最大间隔 51.63 ms。
- 切回旧配置 + 2.10.7：25 秒 8 帧，0.321 Hz，最大间隔 3.121 s，确认不是重启后偶然恢复。
- 新配置 + 原版 2.6.12：180 秒 5328 帧，29.599 Hz，最大间隔 50.27 ms，零解码错误。
- 新配置 + 原版 2.6.12 + Reliable：60 秒 1776 帧，29.600 Hz，最大间隔 48.80 ms，零解码错误。
- 新旧 DDS 均抓到仅 Wi-Fi 地址通告；内核 socket 表确认 `0.0.0.0:7430/7431` 监听。
- 小车 12 秒 `sendto` 跟踪仅向 192.168.0.180 发送：710 次、最长 1.041 ms，无 .65.* 目标。
- 只验收了单 participant 图像链路；未验收多进程布局、RViz 真机 GUI、NoMaD 真机推理、控制、自动校时或长期稳定性。
- 本轮完整证据：`outputs/network_diagnostics/wifi_locator_fix__20260909T104300Z/summary.md`。

## 历史：External Locators 已测与未测

- 原版 → 新版：15 秒 151 条有效 String 消息、Trigger 服务成功。
- 新版 → 原版：15 秒 150 条有效 String 消息、Trigger 服务成功。
- 新版程序仅加载 Fast DDS 2.10.7，不混载 2.6；仍使用 Humble 的 rclpy、消息类型支持和 Fast CDR 1。
- SPDP 额外通告 192.168.0.180:7430/7431，发现小车相机/压缩节点并收到真实图像；内部地址仍存在于通告列表。
- Reliable 60 秒：19 帧，约 0.3207 Hz，最大间隔 3.135 秒。
- Best Effort 60 秒：18 帧，约 0.3205 Hz，最大间隔 3.131 秒。
- 两组均 640×480、零 JPEG 解码错误；小车在测试后本地 10 秒 297 帧，约 29.60 Hz。
- 未验证 RViz GUI、新版完整官方测试套件、安全 DDS、NoMaD 推理、控制器、长期稳定性或其他 ROS 消息/服务。
- 后续校时并复测仍仅约 0.38–0.42 Hz。12 秒 strace 发现：发布线程向内部地址 192.168.65.6 发送累计阻塞 10.04 秒、单次最长 3.106 秒，向正确 Wi-Fi 地址最长仅 0.675 ms。
  直接原因已定位为向不可达内部 locator 的发送阻塞；External Locators 追加正确地址但未移除内部地址。尚未实施 DDS/路由修复或修复后对照。

校时与帧率跟踪证据：`outputs/network_diagnostics/clock_and_framerate__20260909T081722Z/summary.md`。

原始结果：`outputs/network_diagnostics/fastdds_external__20260909T080100Z/`。

参考：[Fast DDS External Locators](https://fast-dds.docs.eprosima.com/en/2.10.x/fastdds/transport/announced_locators.html)、
[Humble RMW 源码](https://github.com/ros2/rmw_fastrtps/tree/6.2.10)。
