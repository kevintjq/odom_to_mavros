# odom_to_mavros

将 FAST-LIO 输出的 `nav_msgs/Odometry` 转发为 MAVROS 可用的 `geometry_msgs/PoseStamped`，并支持通过 YAML 配置雷达到机体系的 6DoF 外参。

默认发布到：

`/mavros/vision_pose/pose`

## 功能

- 订阅 FAST-LIO 里程计
- 从 YAML 读取雷达到机体系外参
- 将雷达位姿变换到机体系
- 发布完整的位置和四元数

## 话题与参数

### 输入输出

- `input_topic`：输入里程计话题，默认 `/Odometry`
- `output_topic`：输出视觉位姿话题，默认 `/mavros/vision_pose/pose`
- `frame_id`：消息头里的坐标系名，默认 `world`

### 外参配置

外参由 `config/odom_to_mavros.yaml` 加载：

- `extrinsic/translation/x y z`
- `extrinsic/rotation/roll pitch yaw`
- `extrinsic/rotation/unit`：`rad` 或 `deg`

当前代码约定 YAML 中填写的是 `T_body_lidar`：

- `translation` 表示雷达原点在机体系下的位置
- `rotation` 表示雷达坐标系相对机体系的姿态

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

## 如何用 LiDAR_IMU_Init 获取 xyz

如果你的目标只是拿到雷达传感器和飞控之间的相对位置偏移，可以用 `LiDAR_IMU_Init` 先标定出 LiDAR 和飞控 IMU 的外参，再把平移部分填到本节点的 YAML 里。

建议流程：

1. 让 `lid_topic` 指向 Mid360 的点云话题
2. 让 `imu_topic` 指向飞控 IMU 话题，而不是 Mid360 自带 IMU
3. 录一段包含静止和明显转动/平移激励的 bag
4. 跑 `LiDAR_IMU_Init`
5. 从结果文件里读取平移向量 `x y z`

常见注意点：

- 如果标定结果里写的是 `T_body_lidar` 或 “lidar in body frame”，通常可以直接拿来填 `translation`
- 如果结果写成了反方向，比如 `T_lidar_body`，需要先取逆再填
- 如果你已经确认两套坐标轴完全对齐，`rotation` 可以先设为 `0, 0, 0`

对这个节点来说，最终要填的是机体系下的雷达原点位置，也就是：

```text
translation = [x, y, z]
```
