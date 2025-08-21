import 'package:flame/game.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:fluttertoast/fluttertoast.dart';
import 'package:ros_flutter_gui_app/page/main_flame.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/basic/action_status.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';


class MainFlamePage extends StatefulWidget {
  @override
  _MainFlamePageState createState() => _MainFlamePageState();
}

class _MainFlamePageState extends State<MainFlamePage> {
  late MainFlame game;
  bool showLayerControl = false;
  bool showCamera = false;

  @override
  void initState() {
    super.initState();
    final rosChannel = Provider.of<RosChannel>(context, listen: false);
    game = MainFlame(rosChannel: rosChannel);
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Scaffold(
      body: Stack(
        children: [
          
          GameWidget(game: game),

          // 顶部菜单栏
          _buildTopMenuBar(context, theme),

          // 左侧工具栏
          _buildLeftToolbar(context, theme),

          // 右侧工具栏
          _buildRightToolbar(context, theme),

          // 底部控制栏
          _buildBottomControls(context, theme),
        ],
      ),
    );
  }

  Widget _buildTopMenuBar(BuildContext context, ThemeData theme) {
    return Positioned(
      left: 5,
      top: 1,
      child: Container(
        height: 50,
        padding: const EdgeInsets.symmetric(vertical: 8),
        child: SingleChildScrollView(
          scrollDirection: Axis.horizontal,
          child: Row(
            children: [
              // 线速度显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: RawChip(
                  avatar: Icon(
                    const IconData(0xe606, fontFamily: "Speed"),
                    color: Colors.green[400],
                  ),
                  label: ValueListenableBuilder<RobotSpeed>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: true)
                            .robotSpeed_,
                    builder: (context, speed, child) {
                      return Text('${(speed.vx).toStringAsFixed(2)} m/s');
                    },
                  ),
                ),
              ),
              // 角速度显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: RawChip(
                  avatar: const Icon(IconData(0xe680, fontFamily: "Speed")),
                  label: ValueListenableBuilder<RobotSpeed>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: true)
                            .robotSpeed_,
                    builder: (context, speed, child) {
                      return Text(
                          '${rad2deg(speed.vw).toStringAsFixed(2)} deg/s');
                    },
                  ),
                ),
              ),
              // 电池电量显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: RawChip(
                  avatar: Icon(
                    const IconData(0xe995, fontFamily: "Battery"),
                    color: Colors.amber[300],
                  ),
                  label: ValueListenableBuilder<double>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: false)
                            .battery_,
                    builder: (context, battery, child) {
                      return Text('${battery.toStringAsFixed(0)} %');
                    },
                  ),
                ),
              ),
              // 导航状态显示
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 4.0),
                child: RawChip(
                  avatar: const Icon(
                    Icons.navigation,
                    color: Colors.green,
                    size: 16,
                  ),
                  label: ValueListenableBuilder<ActionStatus>(
                    valueListenable:
                        Provider.of<RosChannel>(context, listen: true)
                            .navStatus_,
                    builder: (context, navStatus, child) {
                      return Text('${navStatus.toString()}');
                    },
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildLeftToolbar(BuildContext context, ThemeData theme) {
    return Positioned(
      left: 5,
      top: 60,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // 图层开关控制
          Card(
            elevation: 10,
            child: Container(
              child: Row(
                children: [
                  IconButton(
                    icon: Icon(Icons.layers),
                    color:
                        showLayerControl ? Colors.green : theme.iconTheme.color,
                    onPressed: () {
                      setState(() {
                        showLayerControl = !showLayerControl;
                      });
                    },
                  ),
                  if (showLayerControl) ...[
                    IconButton(
                      icon: Icon(Icons.map, size: 20),
                      color: Colors.green,
                      onPressed: () {
                        // 全局代价地图切换逻辑
                      },
                    ),
                    IconButton(
                      icon: Icon(Icons.map_outlined, size: 20),
                      color: Colors.green,
                      onPressed: () {
                        // 局部代价地图切换逻辑
                      },
                    ),
                    IconButton(
                      icon: Icon(Icons.radar, size: 20),
                      color: Colors.green,
                      onPressed: () {
                        // 激光雷达数据切换逻辑
                      },
                    ),
                    IconButton(
                      icon: Icon(Icons.cloud, size: 20),
                      color: Colors.green,
                      onPressed: () {
                        // 点云数据切换逻辑
                      },
                    ),
                  ],
                ],
              ),
            ),
          ),

          // 重定位工具
          Card(
            elevation: 10,
            child: Container(
              child: Row(
                children: [
                  IconButton(
                    onPressed: () {
                      var globalState =
                          Provider.of<GlobalState>(context, listen: false);
                      if (globalState.mode.value == Mode.reloc) {
                        globalState.mode.value = Mode.normal;
                      } else {
                        globalState.mode.value = Mode.reloc;
                      }
                      setState(() {});
                    },
                    icon: Icon(
                      const IconData(0xe60f, fontFamily: "Reloc"),
                      color: Provider.of<GlobalState>(context, listen: false)
                                  .mode
                                  .value ==
                              Mode.reloc
                          ? Colors.green
                          : theme.iconTheme.color,
                    ),
                  ),
                  if (Provider.of<GlobalState>(context, listen: false)
                          .mode
                          .value ==
                      Mode.reloc) ...[
                    IconButton(
                      onPressed: () {
                        // 确认重定位逻辑
                        Provider.of<GlobalState>(context, listen: false)
                            .mode
                            .value = Mode.normal;
                        setState(() {});
                      },
                      icon: Icon(Icons.check, color: Colors.green),
                    ),
                    IconButton(
                      onPressed: () {
                        // 取消重定位逻辑
                        Provider.of<GlobalState>(context, listen: false)
                            .mode
                            .value = Mode.normal;
                        setState(() {});
                      },
                      icon: Icon(Icons.close, color: Colors.red),
                    ),
                  ],
                ],
              ),
            ),
          ),

          // 设置导航目标点
          Card(
            elevation: 10,
            child: IconButton(
              icon: Icon(
                const IconData(0xeba1, fontFamily: "NavPoint"),
                color: (Provider.of<GlobalState>(context, listen: false)
                            .mode
                            .value ==
                        Mode.addNavPoint)
                    ? Colors.green
                    : theme.iconTheme.color,
              ),
              onPressed: () {
                var globalState =
                    Provider.of<GlobalState>(context, listen: false);
                if (globalState.mode.value == Mode.addNavPoint) {
                  globalState.mode.value = Mode.normal;
                } else {
                  globalState.mode.value = Mode.addNavPoint;
                }
                setState(() {});
              },
            ),
          ),

          // 显示相机图像
          Card(
            elevation: 10,
            child: IconButton(
              icon: Icon(Icons.camera_alt),
              color: showCamera ? Colors.green : theme.iconTheme.color,
              onPressed: () {
                setState(() {
                  showCamera = !showCamera;
                });
              },
            ),
          ),

          // 手动控制
          Card(
            elevation: 10,
            child: IconButton(
              icon: Icon(
                const IconData(0xea45, fontFamily: "GamePad"),
                color: Provider.of<GlobalState>(context, listen: false)
                        .isManualCtrl
                        .value
                    ? Colors.green
                    : theme.iconTheme.color,
              ),
              onPressed: () {
                var globalState =
                    Provider.of<GlobalState>(context, listen: false);
                globalState.isManualCtrl.value =
                    !globalState.isManualCtrl.value;
                setState(() {});
              },
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRightToolbar(BuildContext context, ThemeData theme) {
    return Positioned(
      right: 5,
      top: 30,
      child: Column(
        children: [
          // 放大按钮
          IconButton(
            onPressed: () {
              game.zoomIn();
            },
            icon: const Icon(Icons.zoom_in),
          ),
          // 缩小按钮
          IconButton(
            onPressed: () {
              game.zoomOut();
            },
            icon: const Icon(Icons.zoom_out),
          ),
          // 定位到机器人按钮
          IconButton(
            onPressed: () {
              var globalState =
                  Provider.of<GlobalState>(context, listen: false);
              if (globalState.mode.value == Mode.robotFixedCenter) {
                globalState.mode.value = Mode.normal;
              } else {
                globalState.mode.value = Mode.robotFixedCenter;
                game.centerOnRobot();
              }
              setState(() {});
            },
            icon: Icon(
              Icons.location_searching,
              color:
                  Provider.of<GlobalState>(context, listen: false).mode.value ==
                          Mode.robotFixedCenter
                      ? Colors.green
                      : theme.iconTheme.color,
            ),
          ),
          // 退出按钮
          IconButton(
            onPressed: () {
              Navigator.pop(context);
            },
            icon: const Icon(Icons.exit_to_app),
            tooltip: '退出',
          ),
        ],
      ),
    );
  }

  Widget _buildBottomControls(BuildContext context, ThemeData theme) {
    return Positioned(
      left: 5,
      bottom: 10,
      child: Consumer<GlobalState>(
        builder: (context, globalState, child) {
          return Visibility(
            visible: !globalState.isManualCtrl.value,
            child: Row(
              children: [
                // 急停按钮
                Card(
                  color: Colors.red,
                  child: Container(
                    width: 100,
                    height: 50,
                    child: TextButton(
                      child: const Text(
                        "STOP",
                        style: TextStyle(color: Colors.white),
                      ),
                      onPressed: () {
                        Provider.of<RosChannel>(context, listen: false)
                            .sendEmergencyStop();
                        Fluttertoast.showToast(
                          msg: '急停已触发',
                          toastLength: Toast.LENGTH_SHORT,
                          gravity: ToastGravity.BOTTOM,
                        );
                      },
                    ),
                  ),
                ),
                // 停止导航按钮
                Consumer<RosChannel>(
                  builder: (context, rosChannel, child) {
                    return ValueListenableBuilder<ActionStatus>(
                      valueListenable: rosChannel.navStatus_,
                      builder: (context, navStatus, child) {
                        return Visibility(
                          visible: navStatus == ActionStatus.executing ||
                              navStatus == ActionStatus.accepted,
                          child: Card(
                            color: Colors.blue,
                            child: Container(
                              width: 50,
                              height: 50,
                              child: IconButton(
                                icon: const Icon(
                                  Icons.stop_circle,
                                  size: 30,
                                  color: Colors.white,
                                ),
                                onPressed: () {
                                  Provider.of<RosChannel>(context,
                                          listen: false)
                                      .sendCancelNav();
                                  Fluttertoast.showToast(
                                    msg: '导航已停止',
                                    toastLength: Toast.LENGTH_SHORT,
                                    gravity: ToastGravity.BOTTOM,
                                  );
                                },
                              ),
                            ),
                          ),
                        );
                      },
                    );
                  },
                ),
              ],
            ),
          );
        },
      ),
    );
  }
}






