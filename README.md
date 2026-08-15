# odom_to_mavros

将 FAST-LIO 输出的 `nav_msgs/Odometry` 转发为 MAVROS 可用的 `geometry_msgs/PoseStamped`，并支持通过 YAML 配置雷达到机体系的外参。

默认发布到：

`/mavros/vision_pose/pose`

## 功能

- 订阅 FAST-LIO / 里程计话题
- 读取雷达到机体系的 6DoF 外参
- 将雷达位姿变换到机体系
- 发布完整位置 + 四元数姿态

## 话题与参数

### 输入输出

- `input_topic`: 输入里程计话题，默认 `/Odometry`
- `output_topic`: 输出视觉位姿话题，默认 `/mavros/vision_pose/pose`
- `frame_id`: 发布消息头中的坐标系名称，默认 `world`

### 外参配置

外参从 `config/odom_to_mavros.yaml` 加载：

- `extrinsic/translation/x y z`
- `extrinsic/rotation/roll pitch yaw`
- `extrinsic/rotation/unit`: `rad` 或 `deg`

当前代码约定 YAML 填的是 `T_body_lidar`：

- `translation` 表示雷达原点在机体系中的位置
- `rotation` 表示雷达坐标系相对机体系的姿态

## 启动

```bash
roslaunch odom_to_mavros odom_to_mavros.launch
```

## 示例

如果 FAST-LIO 输出话题不是 `/Odometry`，就在 launch 里改：

```xml
<param name="input_topic" value="/fastlio_odom" />
```

如果你的外参是角度制，直接在 YAML 里写：

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

## 注意

- 这个节点只负责转发和坐标变换，不负责 TF 发布。
- `frame_id` 只是消息标签，不会自动做坐标变换。
- 如果外参方向填反，输出位姿会整体偏掉。
