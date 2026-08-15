# odom_to_mavros

Forward FAST-LIO `nav_msgs/Odometry` into MAVROS `geometry_msgs/PoseStamped`, with optional 6DoF extrinsic calibration loaded from YAML.

Default output topic:

`/mavros/vision_pose/pose`

## What it does

- Subscribes to FAST-LIO odometry
- Loads lidar-to-body extrinsics from YAML
- Transforms lidar pose into body pose
- Publishes full position and quaternion

## Topics and parameters

### Input / output

- `input_topic`: input odometry topic, default `/Odometry`
- `output_topic`: output vision pose topic, default `/mavros/vision_pose/pose`
- `frame_id`: message frame label, default `world`

### Extrinsic config

Loaded from `config/odom_to_mavros.yaml`:

- `extrinsic/translation/x y z`
- `extrinsic/rotation/roll pitch yaw`
- `extrinsic/rotation/unit`: `rad` or `deg`

Current code expects `T_body_lidar` in YAML:

- `translation` is the lidar origin in body frame
- `rotation` is the lidar frame pose relative to body frame

## Launch

```bash
roslaunch odom_to_mavros odom_to_mavros.launch
```

## Example config

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

## Notes

- This node only republishes odometry and applies frame transforms
- `frame_id` is a message label only; it does not create TF or trigger transforms
- If the extrinsic direction is wrong, the output pose will be rotated or shifted
