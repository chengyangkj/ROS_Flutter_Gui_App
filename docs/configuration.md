# 配置说明

## 连接配置

| 参数 | 说明 | 默认值 |
|------|------|--------|
| ROS Bridge URL | WebSocket服务器地址 | ws://localhost:9090 |
| 重连间隔 | 断线重连时间间隔(秒) | 3 |

## Topic配置

| 配置项 | 消息类型 | 说明 | 默认值 |
|--------|----------|------|---------|
| battery_topic | sensor_msgs/BatteryState | 电池状态话题 | /battery_state |
| mapTopic | nav_msgs/OccupancyGrid | 地图话题 | /map |
| laserTopic | sensor_msgs/LaserScan | 激光扫描话题 | /scan |
| odomTopic | nav_msgs/Odometry | 里程计话题 | /odom |
| relocTopic | geometry_msgs/PoseWithCovarianceStamped | 重定位话题 | /initialpose |
| navGoalTopic | geometry_msgs/PoseStamped | 导航目标点话题 | /move_base_simple/goal |
| SpeedCtrlTopic | geometry_msgs/Twist | 速度控制话题 | /cmd_vel |
| localCostmapTopic | nav_msgs/OccupancyGrid |局部代价地图话题| /local_costmap/costmap |


## 相机配置

| 配置项 | 说明 | 默认值 |
|--------|------|---------|
| imagePort | web_video_server端口 | 8080 |
| imageTopic | 相机话题 | /camera/rgb/image_raw |
| imageWidth | 图像宽度 | 640 |
| imageHeight | 图像高度 | 480 |

## 机器人参数

| 配置项 | 说明 | 默认值 |
|--------|------|---------|
| maxVx | 最大前进速度(m/s) | 1.0 |
| maxVy | 最大横向速度(m/s) | 1.0 |
| maxVw | 最大旋转速度(rad/s) | 1.0 |
| mapFrameName | 地图坐标系 | map |
| baseLinkFrameName | 机器人坐标系 | base_link | 