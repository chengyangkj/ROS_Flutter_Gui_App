# 使用教程

## 启动ROS环境

1. 启动rosbridge:

```bash
# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

2. 启动web_video_server(可选):

```bash
# ROS1
rosrun web_video_server web_video_server

# ROS2
ros2 run web_video_server web_video_server
```

## 连接设置

1. 点击右上角设置按钮:

![setting_button](../doc/image/setting_button.png)

2. 进入设置界面:

![setting_list](../doc/image/setting_list.png)

3. 输入ROS Bridge WebSocket地址并点击"Connect"按钮连接

## 功能使用

### 地图编辑

点击右上角图标进入地图编辑模式，编辑模式下可添加导航点，障碍物绘制（后续支持）

### 机器人控制

- 左侧摇杆控制前进/后退/左右移动
- 右侧摇杆控制旋转
- 点击重定位按钮可手动设置机器人位置

![main](../doc/image/main.gif)

### 相机显示

1. 确保已启动web_video_server
2. 在设置中配置相机话题和端口
3. 点击相机按钮查看图像流

![camera](../doc/image/camera.png) 