# NoMaD Wheeltec RDK5

基于 NoMaD 的轮趣 ROS 2 小车部署项目：电脑提供 GPU 推理与 HTTPS API，小车采集图像、调用 API，并在本地完成 waypoint 到速度的控制、安全检查和底盘执行。

当前有两个物理节点：一台 Windows/GPU 电脑和一台轮趣小车。电脑上的 WSL、Docker、API/RViz 容器以及各端 ROS 2 节点不另算物理设备。RViz 展示服务器预测，不证明小车实际执行；系统不是已具备独立障碍物停车保证的无人值守导航产品，真机运行必须现场看护并保持物理急停可用。

## 文档入口

- [校园网启动、暂停恢复与停止](docs/校园网导航启动与暂停恢复_20260919-005412.md)
- [API 架构与两个物理节点的职责](ros2_ws/docs/campus_api.md#地址与职责)
- [ROS 2 工作空间](ros2_ws/README.md)
- [Docker/ROS 2 环境](docker/ros2/README.md)
- [核心部署计划与验收边界](docs/NoMaD_ROS2核心部署计划_20260904-214702.md)

## 目录

- `ros2_ws/src/`：导航、控制、安全网关及接口包。
- `ros2_ws/scripts/`：操作台、API/预览启动、校时、凭据管理和测试工具。
- `docker/ros2/`：容器与 Windows PowerShell 启动入口、依赖配置。
- `docs/`：部署计划和使用记录。
- `visualnav-transformer/`：上游 NoMaD/ViNT 源码，以及本项目的视频预处理和离线推理脚本。
- `diffusion_policy/`：上游模型依赖源码。

本仓库不包含模型权重、原始视频、离线数据、运行输出、ROS 构建缓存、小车凭据、TLS 私钥或设备身份文件。它们需要按部署文档在目标机器上单独准备；文档中的 IP、绝对路径和已有容器名称是原部署实例，不是克隆后自动可用的配置。历史文档指向 `outputs/` 的验收证据仅保留在原工作目录，不随代码上传。

两个上游目录以源码快照纳入版本管理，不是 Git 子模块；克隆本仓库不需要 `--recurse-submodules`。原始来源、版本与许可证见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
