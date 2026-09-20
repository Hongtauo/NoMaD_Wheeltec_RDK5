# 校园网 API 的独立 RViz 预览

推理服务与 GUI 是两个独立容器。关闭 RViz、WSLg 不可用或预览失败都不会停止推理服务；小车自身的超时、未解锁和停车保护不变。

2026-09-19 已实际验证：关闭/重开 viewer 后 API 容器 ID 未变、健康检查仍为 HTTP 200、ready；用户确认相机和预测轨迹正常。
这只通过显示与生命周期验收；此前180秒相机 shadow 有4次新鲜度故障，**不能据此认定持续新鲜度或实际运动验收通过**。

2026-09-20 的三档扩展允许校园 API 的 0.05、0.1、0.2 m/s 上限均显式启用 `-NetworkPauseResume`（默认关闭，默认速度仍为 0.05 m/s）。车端更新及三档隔离 ROS 软件验证已完成，本次没有启动底盘或解锁，尚无新增真机运动验收。该选项不改变 RViz 的职责：网络超时时车端暂停输出零速度，相机、客户端和 GUI 保持运行；同次连续行驶意图有效、获得 3 个源时间戳不同且采集晚于暂停时已见帧的新鲜预测，并确认停稳后，车端按 0.025 m/s² 指令加速度上限恢复。定时挡不自动恢复；手停、急停、时钟/里程计异常或操作台断联会取消意图。

三档都不延长原截止时间、不允许过期指令继续驱动。0.2 m/s 时，1 秒视觉数据年龄可对应 20 cm 理论行进距离，软件上限不是实际车速/制动距离保证；本系统无独立障碍物停车保护，必须在现场安全区域持续看护。完整启动与三个档位命令见[启动与暂停恢复](../../docs/校园网导航启动与暂停恢复_20260919-005412.md)。

在 Windows PowerShell、项目根目录运行：

```powershell
.\docker\ros2\start_campus_rviz.ps1
```

这会启动临时容器 `nomad-campus-rviz`。需要先启动更新后的 API，让它创建专用内存预览卷。首次启动服务时也可使用 `start_campus_api.ps1 -WithRViz`，该选项只额外请求独立 GUI；GUI 启动失败仍保留 API。

查看 GUI 日志或只关闭 GUI：

```powershell
docker logs nomad-campus-rviz
docker stop -t 10 nomad-campus-rviz
```

关闭窗口后仅自动移除 viewer 容器。不要为了重开窗口而重启 `nomad-campus-api`。旧版本进程不会因磁盘代码更新自动切换生命周期，需要在小车已停用的维护窗口由操作者更新服务。

## 显示内容及限制

- 相机：最近一次服务器成功推理对应的输入 JPEG，在 viewer 中解码为 ROS 原始图像。
- 蓝绿轨迹、黄色 waypoint：服务器模型预测，固定机器人局部显示原点，不是地图、定位或积分运动轨迹。
- 固定标签 `SERVER PREDICTIONS ONLY`；`Client acceptance/motion: UNKNOWN` 表示不能据此判定小车接受了结果或实际执行了运动。
- 没有新输入时，预测线消失、相机显示清空，保留醒目的 `STALE / WAITING` 标签。重开窗口不会把过期快照回放成实时输入。
- 约1秒 TTL 按服务器快照发布时间判断，仅用于显示失效；不是对相机原始采集年龄、车端结果接受或 waypoint 超时的验证。
- viewer 中有画面不代表真机已解锁，也不代表可以无人监督行驶。

## 数据与隔离

API 后台线程只保留一个最新待写预览，原子替换容器内 `outputs/campus_api/preview/latest.json`，内容仅为最新一帧图像及预测。这个路径由专用 Docker 内存卷 `socialembodiedagent-campus-preview` 覆盖，不使用 F 盘绑定目录，**不会把预览相机图像写入 Windows 项目磁盘；仅共享一份包含图像的临时内存快照**。不保存逐帧历史，不包含 API token、服务器私钥、请求认证或控制遥测。文件最大 2 MiB、Linux 权限必须为 0600，临时文件至多另一个同等大小的文件。正常退出只删除仍属于自身 writer_id 的最新快照，不删除后续服务会话的文件。异常结束的旧快照也会因约 1 秒 TTL 而被 viewer 拒绝。

内存卷使用 Docker `local` 驱动，`type=tmpfs`、`device=tmpfs`、`o=size=4m,mode=0700`，并带 `org.socialembodiedagent.role=campus-preview-tmpfs` 标签。两个入口都核验卷名、驱动、标签及完整挂载选项；同名但不匹配时直接拒绝，不修改或删除旧卷。`volume-nocopy` 禁止将覆盖目录的既有内容复制到卷中。卷只存最新快照，生命周期依赖容器挂载；这不等同于关闭系统 swap 的承诺。

viewer 仅只读挂载 `ros2_ws` 与该内存卷（映射到 `/preview`），不挂载 credentials 或模型权重；使用 `--network none`，ROS domain 87、localhost-only。图像和轨迹在同一 viewer 容器内传给 RViz，不跨校园网传 DDS。

如内存卷不能落实 0600、目录锁或原子替换，API 保持运行并报告预览不可用；须解决预览文件系统问题，不能放宽文件访问限制来掩盖它。
