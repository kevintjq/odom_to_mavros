# odom_to_mavros

将 SLAM Odom 输出的 `nav_msgs/Odometry` 转发为 MAVROS 可用的 `geometry_msgs/PoseStamped`，并支持通过 YAML 配置雷达到机体系的 6DoF 外参。

默认发布到：

`/mavros/vision_pose/pose`

## 功能

- 订阅 SLAM Odom 里程计
- 从 YAML 读取雷达到机体系外参
- 将雷达位姿变换到机体系
- 发布完整的位置和四元数

## 话题与参数

### 输入输出

- `input_topic`：输入里程计话题，默认 `/Odometry`
- `output_topic`：输出视觉位姿话题，默认 `/mavros/vision_pose/pose`
- `frame_id`：消息头里的坐标系名，默认 `world`

### 简化监控参数

只需要在 `config/odom_to_mavros.yaml` 中配置以下监控参数：

- `max_flight_speed`：无人机最大正常飞行速度，单位为 `m/s`。建议填实际最大速度，不要填理论极限。
- `max_angular_speed`：无人机最大正常角速度，单位为 `deg/s`。程序内部会转换为 `rad/s` 后进行比较。
- `vision_pose_timeout`：最后一次成功发布 vision pose 后的允许中断时间，单位为 `s`，默认 `0.2`，按实际链路延迟调整。
- `enable_vision_pose_timeout_protection`：是否在 vision pose 超时时请求 `AUTO.LAND`，默认 `true`。关闭后只报告诊断错误，不改变飞行模式。

输入频率不需要配置，程序会根据里程计消息的实际到达时间自动估计，适用于 FAST-LIO、其他 LIO 或任何 `nav_msgs/Odometry` 发布器。频率估计完成后会在终端打印自动生成的阈值。

程序会自动计算其余阈值：

```text
max_linear_speed  = max_flight_speed * 2
max_position_jump = max_linear_speed / learned_input_frequency * 2
实际跳变上限 = max(max_position_jump, max_linear_speed * 相邻有效位姿时间差)
min_input_frequency = learned_input_frequency * 0.5
max_message_age = max(3 / learned_input_frequency, 0.2)
```

例如无人机最大正常速度为 `5 m/s`，程序自动测得输入频率为 `10 Hz`：

```yaml
max_flight_speed: 5.0
max_angular_speed: 180.0
vision_pose_timeout: 0.2
enable_vision_pose_timeout_protection: true
```

程序自动得到：最大允许线速度 `10 m/s`、单帧最大跳变 `2 m`、最低输入频率 `5 Hz`、最大消息延迟 `0.3 s`。

协方差阈值、未来时间容差和诊断周期使用代码内固定值。检测到非法数据、定位质量恶化、位姿跳变、速度异常或输入过期时，消息会被丢弃，并通过终端 warning 和 `/diagnostics` 报告。该检测可以发现数据跳变或定位质量恶化，但不能单独证明缓慢的绝对漂移。

### LIO 失效判断与停止发送

程序通过输入消息、时间戳、位姿数值、协方差和运动连续性判断 LIO 输出是否异常。检测到异常时，只会停止发送当前异常消息，单帧异常不会立即切换模式；连续无有效输出达到超时阈值后，会异步请求 AUTO.LAND。

以下情况会丢弃消息，不发布到 `/mavros/vision_pose/pose`：

- LIO 停止发布消息，或者输入长时间没有更新。诊断状态会变为 `Odometry input is stale`。
- 时间戳为零、倒序、没有递增、过旧，或超前当前 ROS 时间过多。
- 位置或四元数包含 `NaN`、`Inf`，或者四元数接近零。
- 协方差无效，或位置/姿态协方差超过程序内置阈值。
- 相邻位姿位置跳变过大。
- 根据相邻位姿计算出的线速度超过自动计算的允许值。
- 根据相邻姿态计算出的角速度超过 `max_angular_speed`（单位 `deg/s`）。
- 外参变换后的位姿包含非法数值。

程序启动后会根据里程计消息的实际到达时间自动估计输入频率。频率估计完成后，自动生成最低输入频率、最大消息延迟和单帧最大位置跳变阈值。输入频率过低会产生诊断告警；如果消息仍在到达，频率过低本身不会单独丢弃消息。

当前程序无法仅凭 LIO 里程计判断以下情况：

- 位姿数值连续但已经发生缓慢的绝对漂移。
- 位姿冻结但时间戳仍持续更新。
- 坐标系方向配置错误但数值看起来正常。

判断绝对漂移需要 GPS/RTK、动捕、UWB 或其他外部定位参考。诊断信息发布在 `/diagnostics`，可以查看输入频率、消息延迟、发布数量和各类丢弃计数。LIO 失效后，bridge 不会发送最后一帧旧位姿，最终是否触发 PX4 failsafe 由 PX4 的 EKF 和 failsafe 参数决定。

检测到协方差恶化、位置跳变、线速度异常或角速度异常时，本节点只会丢弃异常帧并发布诊断，不会立即请求 PX4 切换 `AUTO.LAND`。当 `enable_vision_pose_timeout_protection: true` 时，节点在已经成功发布过 vision pose 后、连续超过 `vision_pose_timeout` 未再成功发布时，才会通过 `/mavros/set_mode` 请求 `AUTO.LAND`；启动时尚未发布任何位姿不会触发。关闭该开关后只会上报诊断错误。只要有效 vision pose 持续发布，本节点不会改变飞行模式。PX4 因估计器、Offboard、遥控器、电池或其他 failsafe 自主进入降落模式，属于飞控侧行为。

### 外参配置

外参由 `config/odom_to_mavros.yaml` 加载：

- `extrinsic/translation/x y z`
- `extrinsic/rotation/roll pitch yaw`
- `extrinsic/rotation/unit`：`rad` 或 `deg`

当前代码约定 YAML 中填写的是 `T_fcu_lio_body`：

- `translation` 表示 LIO body 原点在飞控 FCU body 系下的位置
- `rotation` 表示 LIO body 坐标系相对飞控 FCU body 系的姿态

## 启动

```bash
roslaunch odom_to_mavros odom_to_mavros.launch
```

## 配置示例

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

## 说明

- 这个节点只做里程计转发和坐标变换，不负责 TF 发布
- `frame_id` 只是消息标签，不会触发自动坐标变换
- 如果外参方向填反，输出位姿会整体偏移或旋转

## 超时响应与验证边界

默认连续 0.2 秒无有效位姿后请求 AUTO.LAND，每 0.02 秒检查一次（非实时调度，系统负载可能增加延迟）。请求在独立线程执行，不阻塞里程计回调；失败时在后续检查中重试，同一时间仅允许一个请求。mode_sent 只代表请求已发送，不能证明飞控已进入降落。启动后从未发布过有效位姿时不触发。时间戳持续更新的冻结或缓慢漂移仍无法仅靠本节点识别。

本次修改需要在 ROS 环境编译并通过 SITL 验证断流、异常帧、服务延迟和飞控模式反馈后再用于实机。
