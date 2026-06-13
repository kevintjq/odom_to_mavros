# odom_to_mavros

将 FAST-LIO 输出的里程计 `nav_msgs/Odometry` 转发为 MAVROS 可用的视觉位姿 `geometry_msgs/PoseStamped`，发布到：

`/mavros/vision_pose/pose`

## 功能

- 订阅 FAST-LIO 里程计话题
- 只提取 `x / y / z` 位置
- 将位置封装成 `PoseStamped`
- 发布到 MAVROS 的视觉定位输入话题

## 默认配置

节点默认参数如下：

- `input_topic`: `/Odometry`
- `output_topic`: `/mavros/vision_pose/pose`
- `frame_id`: `world`

如果你的 FAST-LIO 里程计话题不是 `/Odometry`，可以在 launch 文件里修改 `input_topic`。

## 启动方式

```bash
roslaunch odom_to_mavros odom_to_mavros.launch
```

## FAST-LIO 对接示例

如果 FAST-LIO 实际发布的话题是：

```bash
/Odometry
```

那当前配置可以直接使用。

如果你的话题名是别的，例如：

```bash
/fastlio_odom
```

把 `launch/odom_to_mavros.launch` 里的参数改成：

```xml
<param name="input_topic" value="/fastlio_odom" />
```

