# odom_to_mavros

ROS1 节点：把 LIO 的 `nav_msgs/Odometry` 转换成 MAVROS 的 `geometry_msgs/PoseStamped`，支持 LIO body 到 FCU body 的 6DoF 外参，并在本节点内判断输入失效、异步请求 `AUTO.LAND`。

## 输入输出

- `input_topic`：默认 `/Odometry`。
- `output_topic`：默认 `/mavros/vision_pose/pose`。
- `frame_id`：默认 `world`，只是输出标签，不会自动变换世界坐标系。
- `/diagnostics`：输入频率、消息延迟、发布/拒绝计数、当前失效原因和降落请求结果，每秒发布。
- `/mavros/set_mode`：触发保护时异步请求 `AUTO.LAND`。

不需要额外健康状态话题，也不修改 LIO 发布器。输入必须使用与本机 ROS 时间一致的采集时间戳，不能用转发时间重盖时间戳。

## 失效判定

正常数据会经过时间戳、位置、四元数、协方差、外参转换和运动连续性检查，然后转发。

| 情况 | 处理 |
| --- | --- |
| 停止接收、持续旧数据、零/重复/倒序时间戳 | 无法更新有效测量期限；达到 `vision_pose_timeout` 后请求 LAND |
| 单帧 NaN/Inf、无效四元数、异常协方差或运动跳变 | 丢弃该帧，不立即请求 LAND |
| 连续无效位姿 | 同时达到 `invalid_pose_min_count` 和 `invalid_pose_timeout` 后请求 LAND；常规测量超时仍独立生效 |
| 收到并通过检查的新鲜位姿 | 更新有效测量期限，清零连续无效计数 |
| 启动以来从未有有效位姿 | 等待输入，不触发 LAND，保留原有启动保护行为 |

连续无效位姿包括非有限位置/姿态、接近零的四元数、负或非有限对角方差、过大方差、非法变换结果、过大位移/线速度/角速度。协方差上限保持为位置 `4 m²`、姿态 `1 rad²`，逐项检查六个对角元素；不检验整个协方差矩阵的半正定性。

无效计数从第一帧异常开始，默认至少 3 帧且持续 0.05 秒；若此后没有有效数据恢复，看门狗也能在该时间到达时判定。相隔超过常规超时的异常帧不合并成一个连续序列。时间戳错误不计入快速异常序列，由有效测量年龄超时覆盖。

**失效判断有边界：**仅靠里程计无法可靠区分静止与“位置冻结但时间戳仍更新”，也无法可靠识别连续的绝对漂移、数值正常的坐标系错误。这些情况不能承诺自动检出。

## 独立看门狗与数据新鲜度

- 看门狗使用独立 ROS 回调队列和一个独立 `AsyncSpinner` 线程，每 10 ms 检查；里程计/诊断回调拥堵不直接占用该队列。
- 超时、重试间隔和输入频率使用 `ros::SteadyTime`，不依赖可跳变的系统墙钟。
- ROS 时间只用于与输入采集时间戳比较。ROS 时间无效、未开始或消息时间异常时不接受数据；已建立的单调时钟期限仍会到期。
- 输入订阅和输出发布队列都为 1，输入启用 TCP_NODELAY。正在处理的回调和网络缓冲仍可能带来延迟，因此发布前再次校验采集时间。
- 期限按“单调时钟当前值 + 超时阈值 − 测量年龄”计算。默认阈值 0.2 秒，一帧已经延迟 0.15 秒的有效数据只剩约 0.05 秒有效期，不会重新获得完整 0.2 秒。
- 消息年龄达到阈值即拒绝，包括启动阶段；频率学习不会把年龄上限放宽。允许的微小未来时间偏差仍为 0.1 秒，但不会给未来消息增加额外有效期。
- 独立线程不提供硬实时保证。CPU 饱和、进程卡死、内核调度、MAVROS/链路延迟仍会影响响应。

## 配置

`launch/odom_to_mavros.launch` 加载 `config/odom_to_mavros.yaml`：

```yaml
max_flight_speed: 5.0       # m/s，正常飞行速度上限
max_angular_speed: 180.0   # deg/s
vision_pose_timeout: 0.2   # 秒，最后有效测量的最大年龄
invalid_pose_min_count: 3  # 连续无效位姿条数，至少 2
invalid_pose_timeout: 0.05 # 秒，连续无效位姿最短持续时间
enable_vision_pose_timeout_protection: true
land_service_timeout: 0.3 # 秒，单次 LAND 工作进程的超时
```

以上为初始默认值，尚未经本机飞行数据标定；应结合正常抖动和故障注入测试调整。总反应时间还包含检查调度、服务调用、通信和飞控模式切换时间。

`enable_vision_pose_timeout_protection: false` 会同时关闭测量超时和连续异常的 LAND 请求，仍保留诊断和异常数据过滤。

输入频率自动学习，运动阈值按以下方式计算：

```text
max_linear_speed = max_flight_speed * 2
max_position_jump = max_linear_speed / learned_input_frequency * 2
实际位移上限 = max(max_position_jump, max_linear_speed * 相邻有效位姿时间差)
min_input_frequency = learned_input_frequency * 0.5
max_message_age = min(vision_pose_timeout, max(3 / learned_input_frequency, 0.2))
```

输入低频本身只告警；如果有效测量已过期仍会触发超时。位移上限考虑实际间隔，避免高频输入短暂中断后因固定单帧距离阈值持续拒绝正常运动。

## 外参

YAML 填写 `T_fcu_lio_body`：LIO body 到 FCU body 的变换。

- `translation`：LIO body 原点在 FCU body 系中的位置，单位米。
- `rotation`：LIO body 相对 FCU body 的姿态，使用 `Rz(yaw) Ry(pitch) Rx(roll)`。
- `unit`：`deg` 或 `rad`，兼容原有 `degree/degrees/radian/radians`；大小写敏感。
- 启动时逐项拒绝 NaN/Inf、错误参数类型和非映射的外参配置；未知角度单位直接报错并以非零状态退出，不再回退到弧度。未填写的数值仍按零处理，未填写单位默认 `rad`。
- 构造后检查变换矩阵是否有限、旋转是否正交且行列式接近 1。检查通过后才建立位姿发布和订阅。

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

输出公式：`T_world_fcu = T_world_lio_body * inverse(T_fcu_lio_body)`。输入应表示实际 LIO body 的位姿，不能把已经完成安装旋转的坐标系再次当作原始雷达系。节点不发布 TF。

## 构建与运行

放入 catkin 工作空间的 `src` 目录，安装 ROS Noetic、Eigen 和所需 ROS/MAVROS 消息依赖后：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch odom_to_mavros odom_to_mavros.launch
```

## 测试与当前限制

无 ROS 环境时可以运行独立监控逻辑测试：

```bash
g++ -std=c++11 -Wall -Wextra -Werror -pthread -Isrc test/safety_monitor_test.cpp -o /tmp/odom_safety_test
/tmp/odom_safety_test
```

单次服务请求/外参校验测试：

```bash
g++ -std=c++11 -Wall -Wextra -Werror -pthread -Isrc test/request_and_extrinsic_test.cpp -o /tmp/request_and_extrinsic_test
/tmp/request_and_extrinsic_test
```

该测试以长时间不退出的真实子进程模拟阻塞调用，验证超时终止、回收、反复重试、成功/失败退出和销毁时清理；还检查六个外参数值的 NaN/Inf 拒绝、角度换算和未知单位拒绝。它不覆盖 ROS 参数服务器的类型读取和真实 MAVROS 服务。

catkin 启用测试时会注册 `safety_monitor_test` 和 `request_and_extrinsic_test`。测试覆盖启动保护、延迟数据期限、过期数据拒绝、单帧异常、连续异常、恢复和并发读写；不能替代完整 ROS 编译、回调队列集成测试或 SITL。

LAND 服务在同一可执行文件的独立子进程中调用，使用 `posix_spawn` 启动，不经过 shell。默认单次进程超时为 0.3 秒，包含启动、ROS 连接、服务调用和退出。父进程使用单调时钟和非阻塞 `waitpid` 检查；超时后终止工作进程，回收后才允许新请求。不存在通过不断分离阻塞线程来重试的资源积累，最多一个在途工作进程。退出节点时也清理在途进程。

失败或超时后，输入仍失效时按原有至少 0.5 秒的尝试间隔重试；不因为服务超时自行改变 LIO 判据。工作进程继承环境，显式沿用父节点的 ROS master、通告主机地址和解析后的服务名。实现面向 Linux/ROS1（使用 `/proc/self/exe` 和 POSIX 进程接口）。

终止本地工作进程无法撤销已发往 MAVROS/飞控的请求，超时只表示本次结果未在期限内返回，重试可能再次发送相同 LAND 请求。`mode_sent` 仍仅表示请求已发送，当前代码不确认飞控实际模式；降落状态锁存和任务节点协同尚未实现。PX4 的位置丢失/Offboard 丢失保护仍需独立配置验证，发出 LAND 请求不能保证降落成功。

建议在 SITL 注入：输入断流、持续延迟、NaN、协方差异常、位置跳变、里程计回调拥堵和 ROS 时间跳变；记录最后有效测量到请求发出的延迟，以及飞控实际模式变化。
