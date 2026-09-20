# NoMaD Wheeltec RDK5

将 NoMaD 无目标局部探索模型接入轮趣 ROS 2 小车：**电脑负责 GPU 推理／HTTPS 服务，小车负责感知、本地控制与执行**。当前是两个物理节点；WSL、Docker、容器和 ROS 2 软件节点不另算物理设备。RViz 展示服务器预测，不证明小车实际执行。

本指南面向**第一次克隆仓库的用户**，不假定你已有作者的镜像、权重、视频、证书、密码或车端部署目录。旧记录中的 `F:\SocialEmbodiedAgent`、`10.201.*` 和容器状态不是你的配置。

> 本系统未实现地图导航或群组感知算法，也没有独立障碍物停车保证，不能无人值守。真机运行必须持续现场看护、清空区域并验证物理急停。部署或看到轨迹不等于允许解锁。

## 复现路线

1. **没有小车**：完成第 1～4 节，验证环境和自己的视频离线推理。
2. **部署模型服务**：完成第 5 节，不必启动机器人底盘。
3. **首次接入小车**：先完成[车端首次部署](docs/首次部署_车端配置.md)，再执行第 6 节。
4. **已配置环境的日常使用**：见第 7 节，不重复生成身份或凭据。

## 1. 环境与适用范围

| 设备 | 本项目使用的环境 | 职责 |
| --- | --- | --- |
| 电脑 | Windows x86_64、Docker Desktop Linux/WSL 2 后端；容器 Ubuntu 22.04、ROS 2 Humble、系统 Python 3.10 | NoMaD、HTTPS API、RViz、Windows 操作台 |
| 小车 | ARM64 RDK/厂商系统、ROS 2 Humble/tROS、`wheeltec` 用户及厂商驱动 | 相机、里程计、API 客户端、控制器、安全网关、底盘 |

- Windows 需 Git、可在 PowerShell 调用的 Python 3.10+ 和 OpenSSH Client。Windows 操作台使用 Python 标准库，无需安装 PyTorch。
- 固定模型依赖包括 PyTorch `2.6.0+cu124`、torchvision `0.21.0+cu124`、NumPy `1.24.4`、diffusers `0.11.1`；完整版本见 [requirements](docker/ros2/requirements.txt) 和 [constraints](docker/ros2/constraints-py310.txt)。不使用 Conda/venv 覆盖容器系统 Python。
- CUDA 需要可用的 NVIDIA GPU、Windows 驱动和 Docker GPU 支持。服务创建脚本只接受**一张可见 GPU**并检查至少 2048 MiB 空闲显存；这不是任务总显存需求保证。
- API 使用 `--network host`：Docker Desktop 4.34+ 中开启 **Settings → Resources → Network → Enable host networking**。[Docker 官方说明](https://docs.docker.com/engine/network/drivers/host/)
- RViz 需要 WSLg；当前脚本挂载 `/mnt/host/wslg/.X11-unix`。没有 WSLg 可先只跑离线推理和无图形 API。
- 小车 → 电脑 TCP 8443、电脑 → 小车 TCP 22 必须可达。校园网客户端隔离／ACL 可能阻断连接；同一 SSID 不代表互通。只放行必要端口和来源，不关闭防火墙。
- 车型、用户、路径和话题目前按 `V550_akm` 厂商系统实现，**不是任意轮趣车型改 IP 即可使用**，详见车端指南。上游 README 的 ROS 1/LoCoBot/Conda 部署不是本项目 ROS 2 安装步骤。

## 2. 克隆并构建电脑环境

在 **Windows PowerShell** 中执行；可以放任意目录，不必使用 F 盘。私有仓库需自己的访问权限，也可使用 SSH URL `git@github.com:Hongtauo/NoMaD_Wheeltec_RDK5.git`。

```powershell
git clone https://github.com/Hongtauo/NoMaD_Wheeltec_RDK5.git
Set-Location NoMaD_Wheeltec_RDK5
$ProjectRoot = (Get-Location).Path
$RuntimeImage = 'socialembodiedagent/nomad-ros2-runtime:robot-preview-20260909'
docker version
python --version
ssh -V
docker build -f docker/ros2/Dockerfile -t $RuntimeImage .
```

**上述镜像名是本地构建标签，不是要求下载作者的私有镜像。** API/RViz 脚本固定使用该标签且 `--pull never`；构建成别的名称会报 `Dedicated runtime image is missing`。首次构建需要访问 Ubuntu/ROS、PyPI、PyTorch 软件源。`prefetch_cuda.py` 只缓存 GPU wheel，不是完整离线安装器。

镜像只含环境，源码和权重须挂载到 `/workspace/SocialEmbodiedAgent`。两份上游源码已经包含，不需要再次 clone、初始化子模块或照上游 Conda 文件重装。

无 GPU 也可以先做导入检查，不加载模型或启动硬件：

```powershell
docker run --rm --network none `
  --mount "type=bind,src=$ProjectRoot,dst=/workspace/SocialEmbodiedAgent,readonly" `
  $RuntimeImage python3 -c "import torch, torchvision, diffusers, vint_train; from diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D; print(torch.__version__)"
```

GPU 用户进一步检查：

```powershell
docker run --rm --network none --gpus all $RuntimeImage nvidia-smi
docker run --rm --network none --gpus all --shm-size=8g `
  --mount "type=bind,src=$ProjectRoot,dst=/workspace/SocialEmbodiedAgent" `
  $RuntimeImage python3 docker/ros2/verify_environment.py
```

成功判据：退出码 0，`outputs/environment_checks/` 中新报告通过。环境验证脚本**强制检查 CUDA**，CPU 用户不要用它作为 CPU 验收命令。

## 3. 下载并核对 NoMaD 权重

从[上游公开模型目录](https://drive.google.com/drive/folders/1a9yWR2iooXFAqjQHetz263--4_2FFggg?usp=sharing)选择 **NoMaD** checkpoint（不是 ViNT/GNM）。这是[上游 README](visualnav-transformer/README.md)给出的来源。仓库与启动脚本不自动下载权重。

```powershell
New-Item -ItemType Directory -Force .\visualnav-transformer\deployment\model_weights | Out-Null
# 将下载的 NoMaD 文件放入此目录，命名为 nomad.pth，再检查：
Get-Item .\visualnav-transformer\deployment\model_weights\nomad.pth | Select-Object Name,Length
Get-FileHash .\visualnav-transformer\deployment\model_weights\nomad.pth -Algorithm SHA256
```

本项目原验证文件：**76,473,631 bytes**，SHA-256：

```text
70f79b8262527e20e56ced64a3e3d7ef91855bc9e7c3fa348d78edcb83c6a333
```

这是本项目文件的核对值，不是上游签名。不同版本须重新确认配置与模型加载，不要只改文件名掩盖不兼容。只加载可信来源的权重。API 固定读取 `visualnav-transformer/deployment/model_weights/nomad.pth` 和 `visualnav-transformer/train/config/nomad.yaml`；离线脚本另支持 `--checkpoint`、`--config`。

## 4. 使用自己的视频做离线推理

先将自己的视频放入 `offline_data/input.mp4`。作者的 `3-r.mp4`、`303.mp4` 和抽帧结果不随仓库分发。

```powershell
New-Item -ItemType Directory -Force .\offline_data | Out-Null
# 放好 input.mp4 后执行：
docker run --rm --network none `
  --mount "type=bind,src=$ProjectRoot,dst=/workspace/SocialEmbodiedAgent" `
  $RuntimeImage python3 visualnav-transformer/prepare_offline_video.py `
  offline_data/input.mp4 --output-dir offline_data/input_frames --fps 4

# CPU 最小模型检查，可较慢；不会发布运动指令。
docker run --rm --network none --cpus 4 `
  --mount "type=bind,src=$ProjectRoot,dst=/workspace/SocialEmbodiedAgent" `
  -e OMP_NUM_THREADS=4 -e MKL_NUM_THREADS=4 `
  $RuntimeImage python3 visualnav-transformer/offline_inference.py `
  offline_data/input_frames --max-windows 1 --device cpu --run-name first-check
```

GPU 运行时给第二条命令加 `--gpus all` 并改为 `--device cuda`；去掉 `--max-windows 1` 才处理完整序列。至少需要 4 帧；预处理输出编号 JPEG 和 `metadata.json`。已有抽帧目录时默认拒绝覆盖，建议换输出目录。

成功判据：`offline_outputs/` 新建唯一运行目录，包含 `manifest.json`（`status=complete`）、`trajectories.npz`、`summary.png`、`plots/`；默认单窗口张量为 `(1, 8, 8, 2)`。这不是实际行驶轨迹，也不是可直接发送给底盘的米制速度命令。

## 5. 首次创建 HTTPS 服务和 RViz

仍在 **Windows 项目根目录**；新开终端需重新设置 `$ProjectRoot`、`$RuntimeImage`。填写本机真实网卡 IPv4，不使用容器／WSL 内部地址或照抄作者地址：

```powershell
Get-NetIPAddress -AddressFamily IPv4 | Select-Object InterfaceAlias,IPAddress
$ServerIP = Read-Host '填写小车可访问的 Windows IPv4'
$RobotIP = Read-Host '填写小车当前 IPv4'
$CredentialsDirectory = Join-Path $env:LOCALAPPDATA 'SocialEmbodiedAgent\campus-api'
.\docker\ros2\start_campus_api.ps1 -ServerIP $ServerIP
docker logs --tail 50 nomad-campus-api
# 等日志出现 ready: true 后再执行下方检查；首次加载模型需要等待。
python .\ros2_ws\scripts\campus_health.py `
  --server-url "https://${ServerIP}:8443" `
  --ca-file (Join-Path $CredentialsDirectory 'server.crt')
```

成功判据：日志 `ready: true`，健康检查 `http_status: 200`、`server.ready: true`。容器创建成功／健康检查通过不等于连续真实图像低延迟验收通过。

首次启动会生成证书、私钥和 token，并限制 Windows 凭据目录权限。只将 `server.crt` 和 `token` 安全分发到小车，**不传 `server.key`**。证书有效 90 天并绑定指定 IP；换 IP／到期需要新凭据目录与车端更新，不能跳过 TLS 校验。

创建脚本拒绝已有同名容器，包括停止状态。参数未变时用 `docker start nomad-campus-api`，不重复创建。CPU 诊断可首次创建时加 `-Device cpu`，但保持时限，不能因此认定 CPU 适合实时行驶；`docker start` 不切换设备模式，也不重新做显存检查。

有 WSLg 时打开独立预览：

```powershell
.\docker\ros2\start_campus_rviz.ps1
docker logs --tail 30 nomad-campus-rviz
```

先由 API 创建预览卷，再启动 RViz。没有小车／有效推理请求时 `STALE / WAITING` 正常，健康检查不会生成画面。关闭 RViz 不影响 API，也不是小车急停。

## 6. 首次接入小车的验收顺序

先完整执行 **[车端首次部署、身份与证书配置](docs/首次部署_车端配置.md)**，包括厂商系统、源码复制、ARM64 接口构建、SSH 指纹、身份登记与证书/token。不能把原机器“已部署”当作新机器已具备这些文件。

1. 确认 Windows 时间正确，正常停止已有小车 ROS／硬件会话。
2. Windows PowerShell 仅校时：

   ```powershell
   .\docker\ros2\start_robot_campus.ps1 -RobotHost $RobotIP -SyncOnly -ManualPassword
   ```

   不启动小车 ROS 节点，但会经 sudo 临时暂停原活动 NTP、设置小车时钟、核验并恢复；不是只读检查。失败／清理不明确时先排查。
3. 小车 `wheeltec` Bash 做相机 shadow：

   ```bash
   source /home/wheeltec/nomad_live_control/campus/setup.bash
   python3 /home/wheeltec/nomad_live_control/campus/run_campus_shadow.py --seconds 45
   ```

   启动真实相机和 API 客户端，**不启动底盘、控制器或网关**。新报告应为 `completed`、`cleanup_verified=true`，并检查子客户端的接受率／新鲜度；不能与既有相机／控制会话并行。
4. 前述检查及现场条件通过后，才启动正式操作台，先保持未解锁观察。普通操作台会启动底盘驱动，不是纯软件测试。

## 7. 已完成首次配置后的日常操作

在 Windows 项目目录执行；先检查 GPU 空闲、电脑时钟和小车现场状态。模型服务／RViz 已运行时不要重启或重复创建。

```powershell
docker start nomad-campus-api
.\docker\ros2\start_campus_rviz.ps1
$RobotIP = Read-Host '填写小车当前 IPv4'
.\docker\ros2\start_robot_campus.ps1 -RobotHost $RobotIP -MaxLinearMps 0.05
```

新电脑没有作者的密码。需要自动填写时运行 `python .\ros2_ws\scripts\robot_credentials.py --save`，通过隐藏输入保存到当前 Windows 用户凭据管理器；也可加 `-ManualPassword` 手动输入。后台重连需要保存的密码或已配置 SSH key/agent，手动密码模式不会后台弹出密码框。

菜单：`i` 状态；`t` 定时（大于 0、至多 10 秒）；`c` 连续；`s` 取消行驶意图并停车、保留节点；`q` 停车、结束小车组并恢复校时服务。现场确认后才输入 `RUN`。不要重复 RUN 来续期或切挡。

速度上限可选 `0.05`、`0.1`、`0.2` m/s，首次复测用低档。暂停恢复默认关闭：

| 选项 | 连续挡行为 |
| --- | --- |
| 不加选项 | 故障取消意图，恢复后须重新确认 |
| `-NetworkPauseResume` | 图像/API 暂时超时先零输出，新鲜预测及停稳检查通过后可继续 |
| 再加 `-OperatorPauseResume` | 另允许 SSH/心跳短暂失联的最多 30 秒续行窗口；新连接后 3 个新鲜预测与全部保护通过才继续 |

心跳最长 2 秒授权及独立网关保护仍保留。30 秒续行窗口／120 秒会话保留都不是失联行驶权限。手停、急停、硬故障、窗口到期、解锁回执不确定会取消续行；定时挡不自动恢复。[详细规则](docs/校园网导航启动与暂停恢复_20260919-005412.md)

结束：`s` → 现场确认停住 → `q` → 等待清理成功，再按需停止电脑服务：

```powershell
docker stop -t 10 nomad-campus-rviz  # 窗口已关闭／容器已移除则跳过
docker stop -t 10 nomad-campus-api
```

不能以关终端、关 RViz 或断网代替停车；无法确认时用物理急停。

## 8. 可选：ROS 2 构建和测试

API／独立 RViz 直接使用挂载源码，不要求电脑先编译 ROS 工作空间。需要接口、`ros2 launch` 或开发测试时再构建。**Windows 挂载目录可能禁止 Linux 符号链接**，将 build/install/log 放入 Linux Docker 命名卷：

```powershell
docker run --rm -it --network none `
  --mount "type=bind,src=$ProjectRoot,dst=/workspace/SocialEmbodiedAgent" `
  --mount 'type=volume,src=nomad-repro-build,dst=/opt/nomad-repro' `
  $RuntimeImage bash
```

在上述**容器 Bash** 中执行：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /opt/nomad-repro/log build \
  --base-paths /workspace/SocialEmbodiedAgent/ros2_ws/src \
  --build-base /opt/nomad-repro/build \
  --install-base /opt/nomad-repro/install --symlink-install
source /opt/nomad-repro/install/setup.bash
ros2 interface show nomad_interfaces/srv/ArmFor
export ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/workspace/SocialEmbodiedAgent/ros2_ws/src/nomad_navigation:/workspace/SocialEmbodiedAgent/ros2_ws/scripts:$PYTHONPATH
python3 -m pytest -q /workspace/SocialEmbodiedAgent/ros2_ws/src/nomad_navigation/test /workspace/SocialEmbodiedAgent/ros2_ws/scripts
```

基础镜像已包含 colcon、pytest 和 ROS 接口生成器。若更换基础镜像导致缺失，应在有软件源访问的开发容器中安装 `python3-colcon-common-extensions python3-pytest ros-humble-rosidl-default-generators`，不能在 `--network none` 容器里下载依赖。新容器重新 source 安装路径；电脑 x86_64 构建结果不能复制到 ARM64 小车。

2026-09-20，从 GitHub 新克隆提交 `197f1d5`，在已有 runtime 镜像、无 GPU/外网的隔离容器中，将构建目录放到 Linux 文件系统后，**2 个包编译成功、1062 项测试通过、2 项因 Linux 无 PowerShell 跳过**；两项随后在 Windows 单独补测通过。PowerShell 测试模拟 Docker/SSH，不接触小车，不是真机验收。开发时须保留上述 `PYTHONPATH` 尾部原值，否则会遮蔽 ROS 模块。

验证边界：这不是基础镜像无缓存重建或另一台小车行驶验收。基础镜像 tag、apt 包尚未完全固定，pip constraints 也不等于逐字节可复现；干净镜像重建、厂商固件／制动及长期跨设备性能须分别验收。

## 9. 排错与其他资料

| 问题 | 先检查 |
| --- | --- |
| runtime 镜像缺失 | 第 2 节精确标签、本地构建；脚本不下载作者镜像 |
| Windows 拒绝运行 `.ps1` | 阅读并确认脚本后，可仅对当前 PowerShell 进程执行 `Set-ExecutionPolicy -Scope Process RemoteSigned`；不修改全局策略，单位管理策略仍需遵守 |
| 权重找不到／加载失败 | 第 3 节路径、大小、SHA-256、配置；不是克隆自带 |
| CUDA／显存失败 | 驱动、`--gpus all`、单 GPU 和空闲显存；不停止别人的任务 |
| API 不通 | Windows 实际 IP、host networking、TCP 8443、校园网隔离／防火墙 |
| TLS 校验失败 | 两端时间、证书 IP／有效期、client.json；不能跳过验证 |
| SSH 拒绝／身份拒绝 | 车端指南的主机密钥、固定别名、ARM64 非 root 用户、0600 文件权限 |
| symlink `Operation not permitted` | 第 8 节 Linux 卷构建，不在 Windows 挂载路径生成 symlink |
| prepare 拒绝／已有 ROS PID | 正常退出旧操作台或厂商自启节点；查报告，不盲杀或反复校时 |
| RViz 无画面／STALE | WSLg、viewer 日志、是否有有效图像请求；健康检查不产生画面 |
| 有轨迹不运动 | 车端 `i` 的 armed／暂停原因、预测新鲜度、里程计与 gateway；不是只看 RViz |

- [API 架构与协议](ros2_ws/docs/campus_api.md)、[RViz 说明](ros2_ws/docs/campus_rviz.md)、[ROS 工作空间](ros2_ws/README.md)。
- [Docker 环境](docker/ros2/README.md)、[核心部署计划](docs/NoMaD_ROS2核心部署计划_20260904-214702.md)。
- [上游来源和许可证](THIRD_PARTY_NOTICES.md)：`visualnav-transformer/`、`diffusion_policy/` 是源码快照，不是子模块。

权重、视频、数据、构建缓存、凭据及历史 `outputs/` 不分发。旧文档的 `outputs/` 链接是作者本机验收记录，不是复现必备下载项；请生成自己的报告。电脑改代码／重启 API 不会同步小车，升级须按同一提交配套更新车端源码。
