# User Guide

## Launch ROS Environment

1. Start rosbridge:

```bash
# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

2. Start web_video_server (optional):

```bash
# ROS1
rosrun web_video_server web_video_server

# ROS2
ros2 run web_video_server web_video_server
```

## Connection Setup

1. Click the settings button in the top right corner:

![setting_button](../doc/image/setting_button.png)

2. Enter the settings interface:

![setting_list](../doc/image/setting_list.png)

3. Enter the ROS Bridge WebSocket address and click "Connect" button

## Features Usage

### Map Display and Navigation

- Click anywhere on the map to set navigation goal
- Long press the navigation point button to manage waypoints
- Use mouse wheel or touch gestures to zoom the map

![mapping](../doc/image/mapping.gif)

### Robot Control

- Left joystick controls forward/backward/left/right movement
- Right joystick controls rotation
- Click the relocation button to manually set robot position

![main](../doc/image/main.gif)

### Camera Display

1. Make sure web_video_server is running
2. Configure camera topic and port in settings
3. Click camera button to view image stream

![camera](../doc/image/camera.png) 