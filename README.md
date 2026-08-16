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

`LiDAR_IMU_Init` 的结果文件里通常会打印两行最关键的信息：

```text
Rotation LiDAR to IMU (degree) = roll pitch yaw
Translation LiDAR to IMU (meter) = x y z
```

如果你的“body”就是飞控 IMU 这个坐标系，那么这组结果**可以直接填进本节点**：

```yaml
extrinsic:
  translation:
    x: x
    y: y
    z: z
  rotation:
    unit: deg
    roll: roll
    pitch: pitch
    yaw: yaw
```

也就是说，你要的 `xyz` 就是 `Translation LiDAR to IMU` 这一行里的三个数。

如果你现在只想先填平移量，那就直接抄 `translation`，`rotation` 可以先置零。

### 读取步骤

1. 用 `LiDAR_IMU_Init` 跑完标定
2. 打开 `result/Initialization_result.txt`
3. 找最后一组 `Translation LiDAR to IMU`
4. 把三个数按 `x / y / z` 填到 `extrinsic.translation`

### 方向怎么换

如果你拿到的是 `T_imu_lidar`，而你要的是反方向 `T_lidar_imu`，用下面这个逆变换：

```text
R_inv = R^T
t_inv = -R^T * t
```

其中：

- `R` 是 3x3 旋转矩阵
- `t` 是平移向量 `[x, y, z]`

### 脚本方式

本仓库额外提供了一个小脚本，可以把 `Initialization_result.txt` 直接转成 YAML 片段：

先打开脚本，修改文件开头的配置：

```python
RESULT_FILE = r"C:\path\to\LiDAR_IMU_Init\result\Initialization_result.txt"
OUTPUT_INVERSE = False
OUTPUT_UNIT = "deg"
OUTPUT_FORMAT = "yaml"
```

然后运行：

```bash
python tools/convert_lidar_imu_init.py
```

脚本默认读取结果文件里最后一组 `Rotation LiDAR to IMU` 和 `Translation LiDAR to IMU`。

默认输出 YAML。比如：

```text
extrinsic:
  translation:
    x: 0.123456
    y: -0.234567
    z: 0.345678
  rotation:
    unit: deg
    roll: 0.000000
    pitch: 0.000000
    yaw: 1.234567
```

如果你想输出反方向，直接加 `--inverse`：

```bash
python tools/convert_lidar_imu_init.py --inverse
```

也可以直接把脚本顶部的 `OUTPUT_INVERSE` 改成 `True`。
