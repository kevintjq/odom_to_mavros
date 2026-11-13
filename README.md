<h1 align="center">Odom to MAVROS</h1>

<p align="center">LIO 位姿桥接 · 机体外参转换 · 单次飞行 LAND 请求保护</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS-1-22314E?style=flat-square" alt="ROS1">
  <img src="https://img.shields.io/badge/C%2B%2B-11-00599C?style=flat-square" alt="C++11">
  <img src="https://img.shields.io/badge/Platform-Linux-555555?style=flat-square" alt="Linux">
  <img src="https://img.shields.io/badge/License-MIT-green?style=flat-square" alt="MIT License">
</p>

<p align="center">
  <a href="#快速开始">快速开始</a> ·
  <a href="#接口与数据">接口与数据</a> ·
  <a href="#外参配置">外参配置</a> ·
  <a href="#失效保护">失效保护</a> ·
  <a href="#参数参考">参数参考</a> ·
  <a href="#常见问题">常见问题</a>
</p>

---

将 LIO 的 `nav_msgs/Odometry` 转换为 MAVROS 可用的完整位姿。节点在内部判断输入是否失效，触发后持续尝试发送 `AUTO.LAND`，成功发送一次即完成本次保护任务。

| 能力 | 说明 |
| --- | --- |
| 位姿桥接 | 发布位置与四元数，默认输出到 `/mavros/vision_pose/pose` |
| 外参转换 | 从 YAML 加载 LIO body 到 FCU body 的 6DoF 外参 |
| 输入检查 | 检查时间戳、数值、协方差、运动连续性及窗口异常比例 |
| 单次任务 | 触发后保持待发送；成功发送一次后不再重复请求 |

## 快速开始

需要 **Linux、ROS1/catkin、Eigen、MAVROS**。先启动 MAVROS 和 LIO，并确保上游持续发布有效里程计。

### 1. 安装依赖

在已经加载 ROS 环境的终端中执行：

```bash
sudo apt install \
  ros-${ROS_DISTRO}-mavros \
  ros-${ROS_DISTRO}-diagnostic-msgs \
  libeigen3-dev
```

### 2. 获取代码

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/kevintjq/odom_to_mavros.git
```

### 3. 编译

```bash
cd ~/catkin_ws
catkin_make
```

### 4. 配置与启动

在 [odom_to_mavros.yaml](config/odom_to_mavros.yaml) 中填写外参和监控参数；需要更换话题时修改 [launch 文件](launch/odom_to_mavros.launch)。

加载工作空间环境：

```bash
source ~/catkin_ws/devel/setup.bash
```

启动桥接节点：

```bash
roslaunch odom_to_mavros odom_to_mavros.launch
```

> 默认开启保护。启动后 **5 秒**仍无首帧有效数据也会触发 LAND 请求，地面冷启动同样适用。请先运行上游，或按实际初始化时间调整 `startup_timeout`。

## 接口与数据

| 接口 | 类型 | 默认用途 |
| --- | --- | --- |
| `/Odometry` | `nav_msgs/Odometry` | LIO 输入 |
| `/mavros/vision_pose/pose` | `geometry_msgs/PoseStamped` | 转换后的完整位姿 |
| `/mavros/set_mode` | `mavros_msgs/SetMode` 服务 | 发送 `AUTO.LAND` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 输入统计与请求状态 |

输入应满足：

- 使用与本机 ROS 时间一致的真实采集时间，不能用转发时间覆盖旧测量。
- 位置单位为米，姿态为有效四元数；里程计应描述配置中定义的 LIO body。
- 上游世界坐标约定与 MAVROS 输入保持一致。`frame_id` 只是标签，不执行世界坐标转换。

不要求额外的 LIO 健康话题，也不要求输入固定频率。节点自动估计输入频率，不发布 TF。

## 外参配置

配置填写 **`T_fcu_lio_body`**：

| 字段 | 含义 |
| --- | --- |
| `translation` | LIO body 原点在 FCU body 坐标系中的位置，米 |
| `rotation` | LIO body 相对 FCU body 的姿态 |
| `unit` | `deg` 或 `rad`，大小写敏感 |

```yaml
extrinsic:
  translation:
    x: 0.12
    y: -0.03
    z: 0.08
  rotation:
    unit: deg
    roll: 0.0
    pitch: 0.0
    yaw: 5.0
```

以上仅为格式示例，实际值应来自安装测量或标定。旋转顺序为 `Rz(yaw) Ry(pitch) Rx(roll)`，输出关系为：

```text
T_world_fcu = T_world_lio_body × inverse(T_fcu_lio_body)
```

避免对已转换过的坐标系重复应用安装外参。启动时会拒绝非有限数值、错误类型和未知单位，并检查刚体变换。缺失数值默认零，缺失单位默认 `rad`；兼容 `degree/degrees/radian/radians`。

## 失效保护

### 什么时候触发

| 判据 | 默认条件 |
| --- | --- |
| 启动超时 | 5 秒内没有首帧有效位姿 |
| 测量过期 | 最后有效测量年龄达到 0.2 秒，包含传输延迟 |
| 连续异常 | 至少 3 帧被拒绝，自第一帧异常起至少 0.05 秒 |
| 间歇性异常 | 最近 0.2 秒内至少 10 帧、跨度至少 0.05 秒，异常比例达到 70% |

检查覆盖零/重复/倒序/过旧/明显超前时间戳、NaN/Inf、无效四元数、异常协方差、非法外参变换结果和不合理运动。单帧异常先丢弃；正常帧会清零连续计数，但不会清空异常比例窗口。

阈值应结合实际链路延迟和飞行日志调整。仅凭里程计不能可靠识别缓慢绝对漂移、时间戳正常更新的数值冻结或数值正常的坐标系错误。

### 触发后怎么处理

| 状态 | 行为 |
| --- | --- |
| `Monitoring` | 检查输入，转发有效位姿，提前准备请求进程 |
| `Pending` | 故障判定已成立；即使里程计恢复，也保持任务并继续重试 |
| `Sent` | 服务返回 `mode_sent=true`；本次请求任务完成，不再重复发送 |

**完成标准是 LAND 请求成功发送。** 本节点不确认飞控实际模式、不接管降落执行；下一次飞行需要重新启动节点。触发后，位姿过滤与转发仍按原有规则工作。

设置 `enable_vision_pose_timeout_protection: false` 可关闭 LAND 触发与预准备进程，保留输入过滤和诊断。

## 参数参考

常用参数在 [配置文件](config/odom_to_mavros.yaml) 中修改：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `max_flight_speed` | `5.0` | 正常飞行速度上限，m/s |
| `max_angular_speed` | `180.0` | 正常角速度上限，deg/s |
| `startup_timeout` | `5.0` | 等待首帧有效数据，秒 |
| `vision_pose_timeout` | `0.2` | 有效测量年龄上限，秒 |
| `invalid_pose_min_count` | `3` | 连续异常条数下限 |
| `invalid_pose_timeout` | `0.05` | 连续异常最短持续时间，秒 |
| `quality_window` | `0.2` | 异常比例统计窗口，秒 |
| `quality_min_samples` | `10` | 窗口样本数下限 |
| `max_invalid_ratio` | `0.7` | 窗口异常比例触发门限 |
| `enable_vision_pose_timeout_protection` | `true` | 是否发送 LAND |
| `land_worker_startup_timeout` | `2.0` | 请求进程准备超时，秒 |
| `land_service_timeout` | `0.3` | 唤醒后调用/退出超时，秒 |

话题参数 `input_topic`、`output_topic` 和消息标签 `frame_id` 位于现有 launch 的节点参数中，不是 launch 命令行参数。

<details>
<summary>实现说明：时钟、运动阈值与请求进程</summary>

**时钟与队列。** 看门狗使用独立回调队列和线程，每 10 ms 检查。超时、重试和频率统计使用单调时钟；ROS 时间用于判断采集时间。输入/输出队列均为 1，输入启用 TCP_NODELAY。

**测量期限。** 已延迟 0.15 秒的数据在 0.2 秒门限下只剩约 0.05 秒，不因刚转发就重新获得完整期限。容忍最多 0.1 秒的未来偏差，不因此延长有效期。

**运动阈值。** 默认允许线速度为 `max_flight_speed × 2`；单帧位移阈值随输入频率学习，并考虑实际有效测量间隔。输入低频本身只告警。六个协方差对角元素要求有限且非负，位置方差上限为 `4 m²`，姿态为 `1 rad²`。

**请求进程。** 使用 `posix_spawn` 提前完成 ROS 初始化，通过私有本地 socket 等待命令。准备和调用分别计时；失败或超时后回收，等待 0.5 秒重建，避免积累阻塞线程。若触发时尚未准备完成，Pending 任务会等待准备或重建。退出节点时清理在途进程。

超时不能撤销已经送出的命令，重试可能再次发送 LAND。独立线程和预准备进程不提供硬实时保证；调度、系统负载、服务发现与通信仍会影响发送延迟。

</details>

## 常见问题

### 没有输出位姿

确认 LIO 输入频率：

```bash
rostopic hz /Odometry
```

再查看节点诊断：

```bash
rostopic echo /diagnostics
```

根据丢弃计数检查时间戳、数值、外参及运动阈值。只修改 `frame_id` 不会修正坐标系方向。

### 刚启动就触发 LAND

检查上游是否已就绪，以及 `startup_timeout` 是否覆盖 LIO 初始化时间。该规则对地面冷启动与空中重启都生效，本节点不判断飞机是否在空中。

### 数据恢复后还在重试

这是单次任务设计：判定失效后保持 Pending，直到成功发送。恢复输入不会取消任务。

## 许可证

本项目采用 [MIT License](LICENSE)。
