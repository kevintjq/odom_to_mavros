# odom_to_mavros

ROS1 里程计转发与单次飞行失效保护节点。正常时，把 LIO 的 `nav_msgs/Odometry` 经外参转换后发给 MAVROS；判断 LIO 失效后，持续尝试发送 `AUTO.LAND`。成功发送一次即完成本次保护任务，不确认飞控实际模式，也不自动重新准备下一次飞行。

## 启动

将仓库放入 catkin 工作空间 `src`，安装 ROS Noetic、Eigen 和 MAVROS 消息依赖后：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch odom_to_mavros odom_to_mavros.launch
```

配置文件：`config/odom_to_mavros.yaml`。

| 接口 | 默认值/用途 |
| --- | --- |
| `input_topic` | `/Odometry`，输入 LIO 位姿 |
| `output_topic` | `/mavros/vision_pose/pose`，输出完整位置和四元数 |
| `frame_id` | `world`，输出消息标签，不自动变换世界坐标系 |
| `/mavros/set_mode` | 发送 `AUTO.LAND` 请求 |
| `/diagnostics` | 每秒发布输入、拒绝计数及 LAND 发送状态 |

不需要额外健康话题。输入应使用与本机 ROS 时间一致的真实采集时间，不能通过重盖转发时间掩盖延迟。

## LIO 失效判定

| 判据 | 默认条件 |
| --- | --- |
| 启动超时 | 启动后 5 秒仍没有首帧有效位姿 |
| 测量过期 | 最后有效测量年龄达到 0.2 秒；断流和持续旧数据都会触发 |
| 连续异常 | 至少 3 帧被拒绝，并自第一帧异常起持续至少 0.05 秒；有效帧会清零连续计数 |
| 间歇性异常 | 最近 0.2 秒内至少 10 帧、样本跨度至少 0.05 秒，且异常比例达到 70%；偶尔一帧正常数据不会清空窗口 |

启动和空中重启采用同一个超时，因为仅靠输入里程计无法区分两者：地面冷启动超过 `startup_timeout` 没有有效数据也会触发请求。按实际 LIO 启动时间设置它。

检查包括：采集时间戳为零/重复/倒序/过旧/过度超前，非有限位置或四元数，接近零的四元数，无效或过大的协方差，外参变换非法，以及异常位移、线速度和角速度。时间戳拒绝同样计入异常比例。单帧噪声只会丢弃，不单独触发 LAND。

六个协方差对角元素逐项要求有限且非负，位置方差上限为 `4 m²`、姿态为 `1 rad²`；不检验整个矩阵的半正定性。运动阈值随输入频率学习，位移检查考虑相邻有效测量的实际时间间隔，避免高频输入短暂中断后被固定单帧距离阈值持续拒绝。

```text
max_linear_speed = max_flight_speed * 2
max_position_jump = max_linear_speed / learned_input_frequency * 2
实际位移上限 = max(max_position_jump, max_linear_speed * 相邻有效位姿时间差)
max_message_age = min(vision_pose_timeout, max(3 / learned_input_frequency, 0.2))
```

时间阈值为初始默认值，需结合实际输入延迟和飞行日志调整。有效位姿频率降低本身保留原有诊断告警，测量期限和异常比例是实际触发条件。

仅凭里程计不能可靠识别缓慢绝对漂移、时间戳正常更新的数值冻结或数值正常的坐标系错误。

## 请求只需成功发送一次

节点内部状态为：`Monitoring → Pending → Sent`。

- `Monitoring`：转发有效位姿并判断失效，提前准备 LAND 工作进程。
- `Pending`：故障判定已经成立，保留请求任务。即使里程计短暂恢复，也会在调用失败/超时后继续尝试。
- `Sent`：服务调用返回 `mode_sent=true`，本次发送任务完成。进程存活期间不再重复请求；下一次飞行需重新启动节点。

首次失效原因会被保留，后续恢复不会取消任务。保护触发后，位姿过滤和转发仍按原有规则工作；保护任务本身持续到请求发送成功。

LAND 服务在同一个可执行文件的子进程中调用，使用 `posix_spawn`，不经过 shell。子进程先完成 ROS 初始化和客户端创建，然后通过私有本地 socket 通知准备就绪并等待命令；正常阶段不调用 LAND 服务。触发时父进程发送单字节命令，子进程直接进入服务调用，省去临时启动进程和 ROS 初始化的主要开销。实际服务发现/连接、通信和响应仍可能消耗时间，预准备不能保证固定发送延迟。

工作进程准备阶段与实际调用分别使用 `land_worker_startup_timeout` 和 `land_service_timeout`。超时后终止并非阻塞地回收；回收前不创建第二个工作进程，不积累阻塞线程。准备失败或调用失败后等待 0.5 秒重建，重新准备好后继续请求。正常退出节点也清理准备中、等待命令或调用中的工作进程。

如果失效早于工作进程准备完成，请求会保持 Pending 并等待准备完成或重建。请求超时只表示结果未及时返回，不会撤销可能已经发出的命令，因此重试可能再次发送 LAND。

`enable_vision_pose_timeout_protection: false` 会关闭所有 LAND 触发和工作进程预准备，保留位姿过滤和诊断。

## 时钟与队列

看门狗使用独立 ROS 回调队列和独立线程，每 10 ms 检查。超时、重试间隔和输入频率使用单调时钟 `ros::SteadyTime`；ROS 时间只用于与采集时间戳比较。输入/输出队列均为 1，输入启用 TCP_NODELAY，发布前再次校验时间。

有效期限按“当前单调时钟 + 超时阈值 − 测量年龄”计算：已延迟 0.15 秒的数据，在 0.2 秒阈值下只剩约 0.05 秒，不会重新获得完整有效期。可容忍最多 0.1 秒的未来时间偏差，但不会因此延长有效期。

独立线程不是硬实时保证。CPU 饱和、进程停止、系统调度和通信故障仍会延迟请求。实现面向 Linux/ROS1，使用 `/proc/self/exe`、POSIX 进程和信号接口。

## 外参

填写 `T_fcu_lio_body`，即 LIO body 到 FCU body 的变换：平移是 LIO body 原点在 FCU body 中的位置（米）；旋转使用 `Rz(yaw) Ry(pitch) Rx(roll)`。

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

输出公式：`T_world_fcu = T_world_lio_body * inverse(T_fcu_lio_body)`。输入应表示实际 LIO body 的位姿，避免对已经变换过的坐标系重复应用外参。节点不发布 TF。

启动时拒绝 NaN/Inf、错误参数类型、非映射外参和未知角度单位，并检查旋转正交性及行列式。非法配置报错并以非零状态退出。未填写的数值默认零、单位默认 `rad`；单位支持 `deg/degree/degrees` 和 `rad/radian/radians`，大小写敏感。

## 验证范围

仓库只保留运行代码、配置与文档，不包含测试文件或测试构建目标。修改时在仓库外验证失效判据、窗口异常、单次任务状态以及工作进程准备/超时/回收。完整 ROS 编译和 MAVROS/SITL 集成验证尚未完成，默认阈值和真实发送延迟仍需在目标环境验证。
