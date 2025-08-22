import 'package:flutter/material.dart';
import 'package:flame/game.dart';
import 'package:provider/provider.dart';
import 'package:fluttertoast/fluttertoast.dart';
import 'package:ros_flutter_gui_app/page/connect_page.dart';
import 'package:ros_flutter_gui_app/page/main_flame.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/basic/action_status.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/page/map_edit_page.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';


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
    final globalState = Provider.of<GlobalState>(context, listen: false);
    game = MainFlame(rosChannel: rosChannel, globalState: globalState);
    // 加载图层设置
    Provider.of<GlobalState>(context, listen: false).loadLayerSettings();
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final globalState = Provider.of<GlobalState>(context, listen: false);
    final isMapEditMode = globalState.mode.value == Mode.mapEdit;

    return Scaffold(
      body: Stack(
        children: [
          // 游戏画布
          GameWidget(game: game),
          
          // 根据模式显示不同的工具栏
          if (!isMapEditMode) ...[
            // 正常模式下的工具栏
            _buildTopMenuBar(context, theme),
            _buildLeftToolbar(context, theme),
            _buildRightToolbar(context, theme),
            _buildBottomControls(context, theme),
          ] else ...[
            // 地图编辑模式下只显示退出按钮
            Positioned(
              right: 20,
              top: 20,
              child: FloatingActionButton(
                onPressed: () {
                  globalState.exitMapEditMode();
                  Navigator.pop(context);
                },
                backgroundColor: Colors.red,
                child: const Icon(Icons.close, color: Colors.white),
                tooltip: '退出地图编辑模式',
              ),
            ),
          ],
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
                    // 使用循环生成图层控制按钮
                    ...Provider.of<GlobalState>(context, listen: true).layerNames.map((layerName) {
                      // 定义每个图层的图标和颜色配置
                      final layerConfig = <String, Map<String, dynamic>>{
                        'showGrid': {
                          'icon': Icons.grid_on,
                          'iconOff': Icons.grid_off,
                          'color': Colors.green,
                          'tooltip': '网格图层',
                        },
                        'showGlobalCostmap': {
                          'icon': Icons.map,
                          'iconOff': Icons.map_outlined,
                          'color': Colors.green,
                          'tooltip': '全局代价地图',
                        },
                        'showLocalCostmap': {
                          'icon': Icons.map_outlined,
                          'iconOff': Icons.map_outlined,
                          'color': Colors.green,
                          'tooltip': '局部代价地图',
                        },
                        'showLaser': {
                          'icon': Icons.radar,
                          'iconOff': Icons.radar_outlined,
                          'color': Colors.green,
                          'tooltip': '激光雷达数据',
                        },
                        'showPointCloud': {
                          'icon': Icons.cloud,
                          'iconOff': Icons.cloud_outlined,
                          'color': Colors.green,
                          'tooltip': '点云数据',
                        },
                        'showGlobalPath': {
                          'icon': Icons.timeline,
                          'iconOff': Icons.timeline_outlined,
                          'color': Colors.blue,
                          'tooltip': '全局路径',
                        },
                        'showLocalPath': {
                          'icon': Icons.timeline,
                          'iconOff': Icons.timeline_outlined,
                          'color': Colors.green,
                          'tooltip': '局部路径',
                        },
                        'showTopology': {
                          'icon': Icons.account_tree,
                          'iconOff': Icons.account_tree_outlined,
                          'color': Colors.orange,
                          'tooltip': '拓扑地图',
                        },
                        'showRobotFootprint': {
                          'icon': Icons.person_outline,
                          'iconOff': Icons.person_outline,
                          'color': Colors.blue,
                          'tooltip': '机器人轮廓',
                        },
                      };
                      
                      final config = layerConfig[layerName];
                      if (config == null) return const SizedBox.shrink();
                      
                      return Tooltip(
                        message: config['tooltip'] as String,
                        child: ValueListenableBuilder<bool>(
                          valueListenable: Provider.of<GlobalState>(context, listen: true).getLayerState(layerName),
                          builder: (context, isVisible, child) {
                            return IconButton(
                              icon: Icon(
                                isVisible ? config['icon'] as IconData : config['iconOff'] as IconData,
                                size: 20,
                              ),
                              color: isVisible ? config['color'] as Color : Colors.grey,
                              onPressed: () {
                                Provider.of<GlobalState>(context, listen: false).toggleLayer(layerName);
                              },
                            );
                          },
                        ),
                      );
                    }).toList(),
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
          // 地图编辑按钮
          Card(
            elevation: 10,
            child: IconButton(
              icon: Icon(
                Icons.edit_document,
                color: (Provider.of<GlobalState>(context, listen: false)
                            .mode
                            .value ==
                        Mode.mapEdit)
                    ? Colors.orange
                    : theme.iconTheme.color,
              ),
              onPressed: () {
                var globalState =
                    Provider.of<GlobalState>(context, listen: false);
                if (globalState.mode.value == Mode.mapEdit) {
                  globalState.exitMapEditMode();
                } else {
                  globalState.enterMapEditMode();
                  // 跳转到地图编辑页面
                  Navigator.push(
                    context,
                    MaterialPageRoute(
                      builder: (context) => const MapEditPage(),
                    ),
                  );
                }
                setState(() {});
              },
              tooltip: '地图编辑',
            ),
          ),
          
          const SizedBox(height: 8),
          
          // 放大按钮
          IconButton(
            onPressed: () {
              game.zoomIn();
            },
            icon: const Icon(Icons.zoom_in),
            tooltip: '放大',
          ),
          // 缩小按钮
          IconButton(
            onPressed: () {
              game.zoomOut();
            },
            icon: const Icon(Icons.zoom_out),
            tooltip: '缩小',
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
              Navigator.push(context, MaterialPageRoute(builder: (context) => ConnectPage()));
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






