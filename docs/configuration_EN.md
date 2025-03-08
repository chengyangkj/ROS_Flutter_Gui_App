# Configuration Guide

## Connection Settings

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| ROS Bridge URL | WebSocket server address | ws://localhost:9090 |
| Reconnect Interval | Time interval for reconnection (seconds) | 3 |

## Topic Configuration

| Parameter | Message Type | Description | Default Value |
|-----------|--------------|-------------|---------------|
| battery_topic | sensor_msgs/BatteryState | Battery status topic | /battery_state |
| mapTopic | nav_msgs/OccupancyGrid | Map topic | /map |
| laserTopic | sensor_msgs/LaserScan | Laser scan topic | /scan |
| odomTopic | nav_msgs/Odometry | Odometry topic | /odom |
| relocTopic | geometry_msgs/PoseWithCovarianceStamped | Relocation topic | /initialpose |
| navGoalTopic | geometry_msgs/PoseStamped | Navigation goal topic | /move_base_simple/goal |
| SpeedCtrlTopic | geometry_msgs/Twist | Speed control topic | /cmd_vel |

## Camera Configuration

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| imagePort | web_video_server port | 8080 |
| imageTopic | Camera topic | /camera/rgb/image_raw |
| imageWidth | Image width | 640 |
| imageHeight | Image height | 480 |

## Robot Parameters

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| maxVx | Maximum forward speed (m/s) | 1.0 |
| maxVy | Maximum lateral speed (m/s) | 1.0 |
| maxVw | Maximum rotation speed (rad/s) | 1.0 |
| mapFrameName | Map coordinate frame | map |
| baseLinkFrameName | Robot base coordinate frame | base_link | 