import 'package:flutter/material.dart';
import 'package:flame/game.dart';
import 'package:flame/components.dart';
import 'package:flutter/gestures.dart';
import 'package:provider/provider.dart';
import 'package:ros_flutter_gui_app/page/connect_page.dart';
import 'package:ros_flutter_gui_app/page/main_flame.dart';
import 'package:ros_flutter_gui_app/provider/global_state.dart';
import 'package:ros_flutter_gui_app/provider/ros_channel.dart';
import 'package:ros_flutter_gui_app/basic/action_status.dart';
import 'package:ros_flutter_gui_app/basic/RobotPose.dart';
import 'package:ros_flutter_gui_app/page/map_edit_page.dart';
import 'package:ros_flutter_gui_app/provider/nav_point_manager.dart';
import 'package:ros_flutter_gui_app/language/l10n/gen/app_localizations.dart';
import 'package:ros_flutter_gui_app/basic/nav_point.dart';



class MainFlamePage extends StatefulWidget {
  @override
  _MainFlamePageState createState() => _MainFlamePageState();
}

class _MainFlamePageState extends State<MainFlamePage> {
  late MainFlame game;
  bool showLayerControl = false;
  bool showCamera = false;
  NavPoint? selectedNavPoint;

  @override
  void initState() {
    super.initState();
    final rosChannel = Provider.of<RosChannel>(context, listen: false);
    final globalState = Provider.of<GlobalState>(context, listen: false);
    final navPointManager = Provider.of<NavPointManager>(context, listen: false);
    game = MainFlame(
      rosChannel: rosChannel, 
      globalState: globalState,
      navPointManager: navPointManager,
    );

    // 设置信息面板更新回调
    game.onNavPointTap = (NavPoint point) {
      setState(() {
        selectedNavPoint = point;
      });
    };
    
    // 加载图层设置
    Provider.of<GlobalState>(context, listen: false).loadLayerSettings();
  }
  
  // 重新加载导航点和地图数据
  Future<void> _reloadData() async {
    if (game != null) {
      await game.reloadNavPointsAndMap();
    }
  }

    @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    
    return Scaffold(
          body: Stack(
            children: [
              // 游戏画布
              Listener(
                onPointerSignal: (pointerSignal) {
                  if (pointerSignal is PointerScrollEvent) {
                    final position = Vector2(pointerSignal.position.dx, pointerSignal.position.dy);
                    game.onScroll(pointerSignal.scrollDelta.dy, position);
                  }
                },
                child: GestureDetector(
                  onScaleStart: (details) {
                    final position = Vector2(details.localFocalPoint.dx, details.localFocalPoint.dy);
                    game.onScaleStart(position);
                  },
                  onScaleUpdate: (details) {
                    final position = Vector2(details.localFocalPoint.dx, details.localFocalPoint.dy);
                    game.onScaleUpdate(details.scale, position);
                  },
                  onScaleEnd: (details) {
                    game.onScaleEnd();
                  },
                  onTapDown: (details) {
                    // 处理点击事件，检测waypoint
                    game.onTap(details.localPosition);
                  },
                  child: GameWidget(game: game),
                ),
              ),
              _buildTopMenuBar(context, theme),
              _buildLeftToolbar(context, theme),
              _buildRightToolbar(context, theme),
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
                          'icon': Icons.smart_toy,
                          'iconOff': Icons.smart_toy_outlined,
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
      child: Row(
        children: [
          // 右侧信息面板
          if (game.showInfoPanel && game.selectedNavPoint != null)
            Container(
              width: 340,
              margin: const EdgeInsets.only(right: 20),
              child: Card(
                elevation: 16,
                shadowColor: Colors.black.withOpacity(0.3),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(16),
                ),
                child: Container(
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(16),
                    gradient: LinearGradient(
                      begin: Alignment.topLeft,
                      end: Alignment.bottomRight,
                      colors: [
                        Colors.white,
                        Colors.blue[50]!,
                      ],
                    ),
                  ),
                  child: Padding(
                    padding: const EdgeInsets.all(24.0),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        // 标题栏
                        Row(
                          mainAxisAlignment: MainAxisAlignment.spaceBetween,
                          children: [
                            Row(
                              children: [
                                Container(
                                  padding: const EdgeInsets.all(8),
                                  decoration: BoxDecoration(
                                    color: Colors.blue[100],
                                    borderRadius: BorderRadius.circular(12),
                                  ),
                                  child: Icon(
                                    Icons.location_on,
                                    color: Colors.blue[700],
                                    size: 24,
                                  ),
                                ),
                                const SizedBox(width: 12),
                                Text(
                                  '导航点信息',
                                  style: theme.textTheme.titleLarge?.copyWith(
                                    fontWeight: FontWeight.bold,
                                    color: Colors.grey[800],
                                  ),
                                ),
                              ],
                            ),
                            Container(
                              decoration: BoxDecoration(
                                color: Colors.grey[100],
                                borderRadius: BorderRadius.circular(20),
                              ),
                              child: IconButton(
                                icon: const Icon(Icons.close, size: 20),
                                onPressed: () {
                                  game.hideInfoPanel();
                                },
                                tooltip: '关闭',
                                style: IconButton.styleFrom(
                                  padding: const EdgeInsets.all(8),
                                ),
                              ),
                            ),
                          ],
                        ),
                         Divider(height: 32, thickness: 1, color: Colors.grey[300]),
                        
                        // 导航点名称和类型
                        Container(
                          padding: const EdgeInsets.all(16),
                          decoration: BoxDecoration(
                            color: Colors.blue[50],
                            borderRadius: BorderRadius.circular(12),
                            border: Border.all(color: Colors.blue[200]!, width: 1.5),
                            boxShadow: [
                              BoxShadow(
                                color: Colors.blue[100]!.withOpacity(0.3),
                                blurRadius: 8,
                                offset: Offset(0, 2),
                              ),
                            ],
                          ),
                          child: Row(
                            children: [
                              Container(
                                padding: const EdgeInsets.all(8),
                                decoration: BoxDecoration(
                                  color: Colors.blue[100],
                                  borderRadius: BorderRadius.circular(8),
                                ),
                                child: Icon(
                                  Icons.label,
                                  color: Colors.blue[700],
                                  size: 20,
                                ),
                              ),
                              const SizedBox(width: 12),
                              Expanded(
                                child: Column(
                                  crossAxisAlignment: CrossAxisAlignment.start,
                                  children: [
                                    Text(
                                      game.selectedNavPoint!.name,
                                      style: theme.textTheme.titleMedium?.copyWith(
                                        fontWeight: FontWeight.bold,
                                        color: Colors.blue[800],
                                      ),
                                    ),
                                    const SizedBox(height: 8),
                                    Container(
                                      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 4),
                                      decoration: BoxDecoration(
                                        color: _getTypeColor(game.selectedNavPoint!.type),
                                        borderRadius: BorderRadius.circular(16),
                                        boxShadow: [
                                          BoxShadow(
                                            color: _getTypeColor(game.selectedNavPoint!.type).withOpacity(0.3),
                                            blurRadius: 4,
                                            offset: Offset(0, 1),
                                          ),
                                        ],
                                      ),
                                      child: Text(
                                        _getTypeText(game.selectedNavPoint!.type),
                                        style: theme.textTheme.bodySmall?.copyWith(
                                          color: Colors.white,
                                          fontWeight: FontWeight.w600,
                                        ),
                                      ),
                                    ),
                                  ],
                                ),
                              ),
                            ],
                          ),
                        ),
                        
                        const SizedBox(height: 24),
                        
                        // 坐标信息
                        _buildInfoSection(
                          context,
                          theme,
                          '位置坐标',
                          Icons.gps_fixed,
                          [
                            _buildInfoRow('X坐标', '${game.selectedNavPoint!.x.toStringAsFixed(2)} m'),
                            _buildInfoRow('Y坐标', '${game.selectedNavPoint!.y.toStringAsFixed(2)} m'),
                            _buildInfoRow('方向', '${(game.selectedNavPoint!.theta * 180 / 3.14159).toStringAsFixed(1)}°'),
                          ],
                        ),
                        
                        const SizedBox(height: 24),
                        
                        // 导航按钮
                        Container(
                          width: double.infinity,
                          decoration: BoxDecoration(
                            borderRadius: BorderRadius.circular(12),
                            boxShadow: [
                              BoxShadow(
                                color: Colors.blue[400]!.withOpacity(0.3),
                                blurRadius: 8,
                                offset: Offset(0, 4),
                              ),
                            ],
                          ),
                          child: ElevatedButton.icon(
                            onPressed: () async {
                              // 使用RosChannel发送导航目标
                              Provider.of<RosChannel>(context, listen: false).sendNavigationGoal(
                                RobotPose(
                                  game.selectedNavPoint!.x, 
                                  game.selectedNavPoint!.y, 
                                  game.selectedNavPoint!.theta
                                )
                              );
                              
                              // 使用ScaffoldMessenger显示成功消息，替代Fluttertoast
                              ScaffoldMessenger.of(context).showSnackBar(
                                SnackBar(
                                  content: Row(
                                    children: [
                                      Icon(Icons.check_circle, color: Colors.white, size: 20),
                                      const SizedBox(width: 8),
                                      Text('已发送导航目标到 ${game.selectedNavPoint!.name}'),
                                    ],
                                  ),
                                  backgroundColor: Colors.green[600],
                                  duration: const Duration(seconds: 3),
                                  behavior: SnackBarBehavior.floating,
                                  shape: RoundedRectangleBorder(
                                    borderRadius: BorderRadius.circular(8),
                                  ),
                                  margin: const EdgeInsets.all(16),
                                ),
                              );
                              
                              // 发送导航目标后自动关闭信息面板
                              game.hideInfoPanel();
                            },
                            icon: const Icon(Icons.navigation, size: 22),
                            label: const Text(
                              '发送导航目标',
                              style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600),
                            ),
                            style: ElevatedButton.styleFrom(
                              backgroundColor: Colors.blue[600],
                              foregroundColor: Colors.white,
                              padding: const EdgeInsets.symmetric(vertical: 18),
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(12),
                              ),
                              elevation: 0,
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ),
            ),
          
          // 原有的工具栏按钮
          Column(
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
                     Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => MapEditPage(
                            onExit: () {
                              // 地图编辑界面退出时，重新加载数据
                              _reloadData();
                            },
                          ),
                        ),
                      );
                    setState(() {});
                  },
                  tooltip: '地图编辑',
                ),
              ),
              
              const SizedBox(height: 8),
              
              // 放大按钮
              Card(
                elevation: 10,
                child: IconButton(
                  onPressed: () {
                    game.zoomIn();
                  },
                  icon: Icon(
                    Icons.zoom_in,
                    color: theme.iconTheme.color,
                  ),
                  tooltip: '放大',
                ),
              ),
              // 缩小按钮
              Card(
                elevation: 10,
                child: IconButton(
                  onPressed: () {
                    game.zoomOut();
                  },
                  icon: Icon(
                    Icons.zoom_out,
                    color: theme.iconTheme.color,
                  ),
                  tooltip: '缩小',
                ),
              ),
              // 定位到机器人按钮
              Card(
                elevation: 10,
                child: IconButton(
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
              ),
              // 退出按钮
              Card(
                elevation: 10,
                child: IconButton(
                  onPressed: () {
                    Navigator.push(context, MaterialPageRoute(builder: (context) => ConnectPage()));
                  },
                  icon: Icon(
                    Icons.exit_to_app,
                    color: theme.iconTheme.color,
                  ),
                  tooltip: '退出',
                ),
              ),
            ],
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
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(
                            content: Row(
                              children: [
                                Icon(Icons.warning, color: Colors.white, size: 20),
                                const SizedBox(width: 8),
                                Text('急停已触发'),
                              ],
                            ),
                            backgroundColor: Colors.red[600],
                            duration: const Duration(seconds: 2),
                            behavior: SnackBarBehavior.floating,
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(8),
                            ),
                            margin: const EdgeInsets.all(16),
                          ),
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
                                  ScaffoldMessenger.of(context).showSnackBar(
                                    SnackBar(
                                      content: Row(
                                        children: [
                                          Icon(Icons.stop_circle, color: Colors.white, size: 20),
                                          const SizedBox(width: 8),
                                          Text('导航已停止'),
                                        ],
                                      ),
                                      backgroundColor: Colors.blue[600],
                                      duration: const Duration(seconds: 2),
                                      behavior: SnackBarBehavior.floating,
                                      shape: RoundedRectangleBorder(
                                        borderRadius: BorderRadius.circular(8),
                                      ),
                                      margin: const EdgeInsets.all(16),
                                    ),
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

  Widget _buildInfoSection(BuildContext context, ThemeData theme, String title, IconData icon, List<Widget> children) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Row(
          children: [
            Icon(icon, color: Colors.grey[600], size: 18),
            const SizedBox(width: 8),
            Text(
              title,
              style: theme.textTheme.titleSmall?.copyWith(
                fontWeight: FontWeight.w600,
                color: Colors.grey[700],
              ),
            ),
          ],
        ),
        const SizedBox(height: 12),
        Container(
          padding: const EdgeInsets.all(12),
          decoration: BoxDecoration(
            color: Colors.grey[50],
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: Colors.grey[200]!),
          ),
          child: Column(
            children: children,
          ),
        ),
      ],
    );
  }

  Widget _buildInfoRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4.0),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            label, 
            style: Theme.of(context).textTheme.bodyMedium?.copyWith(
              color: Colors.grey[600],
            ),
          ),
          Text(
            value, 
            style: Theme.of(context).textTheme.bodyMedium?.copyWith(
              fontWeight: FontWeight.w600,
              color: Colors.grey[800],
            ),
          ),
        ],
      ),
    );
  }

  String _formatDateTime(DateTime dateTime) {
    return '${dateTime.year}-${dateTime.month.toString().padLeft(2, '0')}-${dateTime.day.toString().padLeft(2, '0')} ${dateTime.hour.toString().padLeft(2, '0')}:${dateTime.minute.toString().padLeft(2, '0')}:${dateTime.second.toString().padLeft(2, '0')}';
  }

  Color _getTypeColor(NavPointType type) {
    switch (type) {
      case NavPointType.navGoal:
        return Colors.blue[600]!;
      case NavPointType.chargeStation:
        return Colors.green[600]!;
      default:
        return Colors.grey[600]!;
    }
  }

  String _getTypeText(NavPointType type) {
    switch (type) {
      case NavPointType.navGoal:
        return '导航目标';
      case NavPointType.chargeStation:
        return '充电站';
      default:
        return '未知类型';
    }
  }
}




